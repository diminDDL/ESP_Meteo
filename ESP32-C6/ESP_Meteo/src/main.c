#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif
#include "bme280.h"
#include "i2c_bus.h"
#include "secret.h"

#define LED_GPIO            15
#define BME_GND_GPIO        GPIO_NUM_22
#define I2C_MASTER_SCL_IO   GPIO_NUM_20      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   GPIO_NUM_19      /*!< gpio number for I2C master data */
#define I2C_MASTER_NUM      I2C_NUM_0       /*!< I2C port number for master */
#define I2C_MASTER_FREQ_HZ  100000          /*!< I2C master clock frequency */

// ESP32-C6: IO0 maps to ADC1 channel 0.
#define ADC1_CH0_GPIO       GPIO_NUM_0
#define ADC1_UNIT           ADC_UNIT_1
#define ADC1_CH0_CHANNEL    ADC_CHANNEL_0
#define ADC1_CH0_ATTEN      ADC_ATTEN_DB_12
#define ADC1_BITWIDTH       ADC_BITWIDTH_DEFAULT

static const char *TAG = "ESP_Meteo";

// Debug / power flags (compile-time)
// - When DEBUG_ENABLE_OUTPUT=0, all printf/ESP_LOG* are disabled in this file.
// - When DEBUG_LED_SOLID_ON=1, the LED stays ON while the device is awake.
//   When DEBUG_LED_SOLID_ON=0, the LED is kept OFF (no blinking).
// - When DEBUG_DUMP_SENSOR=1, BME280 values are continuously printed for debugging
#ifndef DEBUG_ENABLE_OUTPUT
#define DEBUG_ENABLE_OUTPUT 0
#endif

#ifndef DEBUG_LED_SOLID_ON
#define DEBUG_LED_SOLID_ON 0
#endif

#ifndef DEBUG_DUMP_SENSOR
#define DEBUG_DUMP_SENSOR 0
#endif

#if DEBUG_ENABLE_OUTPUT
#define DBG_PRINTF(...)            printf(__VA_ARGS__)
#define DBG_LOGI(tag, fmt, ...)    ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define DBG_LOGW(tag, fmt, ...)    ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#define DBG_LOGE(tag, fmt, ...)    ESP_LOGE(tag, fmt, ##__VA_ARGS__)
#else
#define DBG_PRINTF(...)            do { } while (0)
#define DBG_LOGI(tag, fmt, ...)    do { } while (0)
#define DBG_LOGW(tag, fmt, ...)    do { } while (0)
#define DBG_LOGE(tag, fmt, ...)    do { } while (0)
#endif

#define TRY_RETURN(expr)                               \
    do {                                               \
        esp_err_t __e = (expr);                        \
        if (__e != ESP_OK) {                           \
            DBG_LOGW(TAG, "%s failed: %s", #expr, esp_err_to_name(__e)); \
            return __e;                                \
        }                                              \
    } while (0)

#define TRY_GOTO(expr, label)                          \
    do {                                               \
        esp_err_t __e = (expr);                        \
        if (__e != ESP_OK) {                           \
            DBG_LOGW(TAG, "%s failed: %s", #expr, esp_err_to_name(__e)); \
            goto label;                                \
        }                                              \
    } while (0)

static volatile bool s_wifi_connected = false;

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_cali_handle = NULL;
static bool s_adc_cali_enabled = false;

static esp_err_t adc_gpio0_init(void)
{
    // Note: oneshot driver configures the ADC channel; no explicit GPIO config needed.
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC1_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC1_CH0_ATTEN,
        .bitwidth = ADC1_BITWIDTH,
    };
    err = adc_oneshot_config_channel(s_adc_handle, ADC1_CH0_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        (void)adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return err;
    }

    s_adc_cali_enabled = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC1_UNIT,
        .chan = ADC1_CH0_CHANNEL,
        .atten = ADC1_CH0_ATTEN,
        .bitwidth = ADC1_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali_handle) == ESP_OK) {
        s_adc_cali_enabled = true;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC1_UNIT,
        .atten = ADC1_CH0_ATTEN,
        .bitwidth = ADC1_BITWIDTH,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali_handle) == ESP_OK) {
        s_adc_cali_enabled = true;
    }
#endif

    if (!s_adc_cali_enabled) {
        DBG_LOGW(TAG, "ADC calibration not available; only raw readings will be shown");
    }

    DBG_LOGI(TAG, "ADC ready: GPIO%d (ADC1_CH0), atten=%d", (int)ADC1_CH0_GPIO, (int)ADC1_CH0_ATTEN);
    return ESP_OK;
}

static void adc_gpio0_deinit(void)
{
    if (s_adc_cali_handle != NULL) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        (void)adc_cali_delete_scheme_curve_fitting(s_adc_cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        (void)adc_cali_delete_scheme_line_fitting(s_adc_cali_handle);
#endif
        s_adc_cali_handle = NULL;
        s_adc_cali_enabled = false;
    }

    if (s_adc_handle != NULL) {
        (void)adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }
}

static esp_err_t adc_gpio0_read_raw(int *out_raw)
{
    if (s_adc_handle == NULL || out_raw == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return adc_oneshot_read(s_adc_handle, ADC1_CH0_CHANNEL, out_raw);
}

static esp_err_t adc_gpio0_read_voltage_mv(int *out_mv, int *out_raw)
{
    if (out_mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int raw = 0;
    esp_err_t err = adc_gpio0_read_raw(&raw);
    if (err != ESP_OK) {
        return err;
    }
    if (out_raw != NULL) {
        *out_raw = raw;
    }

    if (!s_adc_cali_enabled || s_adc_cali_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int mv = 0;
    err = adc_cali_raw_to_voltage(s_adc_cali_handle, raw, &mv);
    if (err != ESP_OK) {
        return err;
    }

    *out_mv = mv;
    return ESP_OK;
}

static esp_err_t url_append(char *buf, size_t buf_size, size_t *offset, const char *fmt, ...)
{
    if (buf == NULL || offset == NULL || fmt == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (*offset >= buf_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    va_list ap;
    va_start(ap, fmt);
    int written = vsnprintf(buf + *offset, buf_size - *offset, fmt, ap);
    va_end(ap);

    if (written < 0 || (size_t)written >= (buf_size - *offset)) {
        return ESP_ERR_INVALID_SIZE;
    }

    *offset += (size_t)written;
    return ESP_OK;
}

static esp_err_t thingspeak_send_readings_optional(
    const float *temperature_c,
    const float *humidity_pct,
    const float *pressure_hpa,
    const float *battery_v)
{
    char url[256];
    size_t off = 0;

    esp_err_t err = url_append(
        url,
        sizeof(url),
        &off,
        "https://api.thingspeak.com/update?api_key=%s",
        THINKSPEAK_API_KEY);
    if (err != ESP_OK) {
        return err;
    }

    if (temperature_c != NULL) {
        err = url_append(url, sizeof(url), &off, "&field1=%.2f", *temperature_c);
        if (err != ESP_OK) {
            return err;
        }
    }
    if (humidity_pct != NULL) {
        err = url_append(url, sizeof(url), &off, "&field2=%.2f", *humidity_pct);
        if (err != ESP_OK) {
            return err;
        }
    }
    if (pressure_hpa != NULL) {
        // Convert hPa -> mmHg
        const float pressure_mmhg = (*pressure_hpa) * 0.750061683f;
        err = url_append(url, sizeof(url), &off, "&field3=%.2f", pressure_mmhg);
        if (err != ESP_OK) {
            return err;
        }
    }
    if (battery_v != NULL) {
        err = url_append(url, sizeof(url), &off, "&field8=%.2f", *battery_v);
        if (err != ESP_OK) {
            return err;
        }
    }

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 8000,
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
        .crt_bundle_attach = esp_crt_bundle_attach,
#endif
    };

#if !CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
    DBG_LOGW(TAG, "ThingSpeak HTTPS needs CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y (see sdkconfig.defaults)");
    return ESP_ERR_INVALID_STATE;
#endif

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_FAIL;
    }

    DBG_LOGI(TAG, "ThingSpeak: GET %s", url);
    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status == 200) {
            DBG_LOGI(TAG, "ThingSpeak: status=%d", status);
        } else {
            DBG_LOGW(TAG, "ThingSpeak: HTTP status=%d", status);
            err = ESP_FAIL;
        }
    } else {
        DBG_LOGW(TAG, "ThingSpeak: request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        DBG_LOGI(TAG, "Wi-Fi: STA start, connecting...");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            DBG_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        }
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        DBG_LOGW(TAG, "Wi-Fi: disconnected, reconnecting...");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            DBG_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        s_wifi_connected = true;
        DBG_LOGI(TAG, "Wi-Fi: got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        return;
    }
}

static esp_err_t wifi_init_sta_from_secret(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        TRY_RETURN(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    TRY_RETURN(esp_netif_init());
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    if (esp_netif_create_default_wifi_sta() == NULL) {
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    TRY_RETURN(esp_wifi_set_mode(WIFI_MODE_STA));
    TRY_RETURN(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    TRY_RETURN(esp_wifi_start());

    DBG_LOGI(TAG, "Wi-Fi init done (SSID=%s)", WIFI_SSID);
    return ESP_OK;
}

static void led_init_set_awake_state(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&io_conf);

    // No blinking in low-power mode.
    gpio_set_level(LED_GPIO, DEBUG_LED_SOLID_ON ? 1 : 0);
}

static void enable_bme280_power(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BME_GND_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&io_conf);

    // Set GND pin LOW to power BME280
    gpio_set_level(BME_GND_GPIO, 0);
}

static void disable_bme280_power(void)
{
    // High means power off
    gpio_set_level(BME_GND_GPIO, 1);

    // Disconnect the pin completely
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BME_GND_GPIO),
        .mode = GPIO_MODE_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&io_conf);
}

static void bme280_read_avg(
    bme280_handle_t bme280,
    int samples,
    uint32_t inter_sample_delay_ms,
    float *out_temperature_c,
    bool *out_ok_temp,
    float *out_humidity_pct,
    bool *out_ok_hum,
    float *out_pressure_hpa,
    bool *out_ok_press)
{
    float sum_t = 0.0f;
    float sum_h = 0.0f;
    float sum_p = 0.0f;
    int cnt_t = 0;
    int cnt_h = 0;
    int cnt_p = 0;

    for (int i = 0; i < samples; i++) {
        float t = 0.0f;
        float h = 0.0f;
        float p = 0.0f;

        if (ESP_OK == bme280_read_temperature(bme280, &t)) {
            sum_t += t;
            cnt_t++;
        }
        usleep(100 * 1000);

        if (ESP_OK == bme280_read_humidity(bme280, &h)) {
            sum_h += h;
            cnt_h++;
        }
        usleep(100 * 1000);

        if (ESP_OK == bme280_read_pressure(bme280, &p)) {
            sum_p += p;
            cnt_p++;
        }

        if (inter_sample_delay_ms > 0 && i != (samples - 1)) {
            usleep((useconds_t)inter_sample_delay_ms * 1000);
        }
    }

    if (out_ok_temp != NULL) {
        *out_ok_temp = (cnt_t > 0);
    }
    if (out_ok_hum != NULL) {
        *out_ok_hum = (cnt_h > 0);
    }
    if (out_ok_press != NULL) {
        *out_ok_press = (cnt_p > 0);
    }

    if (out_temperature_c != NULL) {
        *out_temperature_c = (cnt_t > 0) ? (sum_t / cnt_t) : 0.0f;
    }
    if (out_humidity_pct != NULL) {
        *out_humidity_pct = (cnt_h > 0) ? (sum_h / cnt_h) : 0.0f;
    }
    if (out_pressure_hpa != NULL) {
        *out_pressure_hpa = (cnt_p > 0) ? (sum_p / cnt_p) : 0.0f;
    }
}

static bool wait_for_wifi_connected(uint32_t timeout_ms)
{
    const int64_t start_us = esp_timer_get_time();
    while (!s_wifi_connected) {
        const int64_t elapsed_ms = (esp_timer_get_time() - start_us) / 1000;
        if (elapsed_ms >= (int64_t)timeout_ms) {
            break;
        }
        usleep(100 * 1000);
    }
    return s_wifi_connected;
}

void app_main() {
    // Power on BME280 sensor as early as possible
    enable_bme280_power();

    #if DEBUG_ENABLE_OUTPUT
        // Wait for ~5 seconds to allow attaching the USB serial console.
        usleep(5000 * 1000);
    #endif

    #if DEBUG_DUMP_SENSOR
        esp_err_t err_dbg = ESP_OK;
        i2c_bus_handle_t i2c_bus_dbg = NULL;
        bme280_handle_t bme280_dbg = NULL;
        float temperature_dbg = 0.0f;
        float humidity_dbg = 0.0f;
        float pressure_dbg = 0.0f;
        while(true){
            enable_bme280_power();
            // usleep(500 * 1000);
            // Initialize I2C bus for BME280
            i2c_config_t conf = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = I2C_MASTER_SDA_IO,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_io_num = I2C_MASTER_SCL_IO,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = I2C_MASTER_FREQ_HZ,
            };
            i2c_bus_dbg = i2c_bus_create(I2C_MASTER_NUM, &conf);
            if (i2c_bus_dbg == NULL) {
                DBG_LOGW(TAG, "i2c_bus_create failed");
            }
            // Initialize BME280 sensor
            bme280_dbg = bme280_create(i2c_bus_dbg, BME280_I2C_ADDRESS_DEFAULT);
            if (bme280_dbg == NULL) {
                DBG_LOGW(TAG, "bme280_create failed");
            }
            err_dbg = bme280_default_init(bme280_dbg);
            if (err_dbg != ESP_OK) {
                DBG_LOGW(TAG, "bme280_default_init failed: %s", esp_err_to_name(err_dbg));
                bme280_delete(&bme280_dbg);
            }
            usleep(50 * 1000);
            if (bme280_dbg != NULL) {
                usleep(50 * 1000);
                bme280_read_temperature(bme280_dbg, &temperature_dbg);
                usleep(50 * 1000);
                bme280_read_humidity(bme280_dbg, &humidity_dbg);
                usleep(50 * 1000);
                bme280_read_pressure(bme280_dbg, &pressure_dbg);
                usleep(50 * 1000);
                DBG_PRINTF("BME280: Temp=%.2f C, Hum=%.2f %%, Pres=%.2f hPa\n", temperature_dbg, humidity_dbg, pressure_dbg);
            }

            bme280_delete(&bme280_dbg);
            i2c_bus_delete(&i2c_bus_dbg);

            disable_bme280_power();
            usleep(30000 * 1000);
        }
    #endif

    #if DEBUG_LED_SOLID_ON
        led_init_set_awake_state();
    #endif

    esp_err_t err = ESP_OK;
    i2c_bus_handle_t i2c_bus = NULL;
    bme280_handle_t bme280 = NULL;
    bool wifi_started = false;
    bool sent_ok = false;

    // Initialize ADC sampling on IO0 / ADC1_CH0 as early as possible so we can
    // still report battery voltage even if BME280 init/read fails.
    err = adc_gpio0_init();
    if (err != ESP_OK) {
        DBG_LOGW(TAG, "ADC init failed: %s", esp_err_to_name(err));
        // Not fatal; we may still send BME280 data.
    }

    // Sample ADC for battery voltage
    float battery_v = 0.0f;
    bool ok_batt = false;
    {
        int adc_raw = 0;
        int adc_mv = 0;
        esp_err_t adc_err = adc_gpio0_read_voltage_mv(&adc_mv, &adc_raw);
        if (adc_err == ESP_OK) {
            battery_v = (adc_mv * 2) / 1000.0f;
            ok_batt = true;
            DBG_PRINTF("ADC IO0: raw=%d, adc=%d mV, batt=%.2f V\n", adc_raw, adc_mv, battery_v);
        } else {
            // Calibration may not be available; keep low power behavior (skip battery_v).
            (void)adc_gpio0_read_raw(&adc_raw);
            DBG_PRINTF("ADC IO0: raw=%d (no calibrated voltage: %s)\n", adc_raw, esp_err_to_name(adc_err));
        }
    }
    
    // Initialize I2C bus for BME280
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    if (i2c_bus == NULL) {
        DBG_LOGW(TAG, "i2c_bus_create failed");
        // Not fatal; we can still send battery voltage to ThingSpeak.
        goto after_bme_init;
    }
    // Give the BME some time to wake up
    usleep(100 * 1000);
    // Initialize BME280 sensor
    bme280 = bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT);
    if (bme280 == NULL) {
        DBG_LOGW(TAG, "bme280_create failed");
        // Not fatal; we can still send battery voltage to ThingSpeak.
        goto after_bme_init;
    }
    err = bme280_default_init(bme280);
    if (err != ESP_OK) {
        DBG_LOGW(TAG, "bme280_default_init failed: %s", esp_err_to_name(err));
        // Not fatal; we can still send battery voltage to ThingSpeak.
        bme280_delete(&bme280);
        goto after_bme_init;
    }

after_bme_init:

    // 2) Sample BME280 before starting Wi-Fi to avoid voltage dropouts.
    float temperature = 0.0f;
    float humidity = 0.0f;
    float pressure = 0.0f;
    bool ok_temp = false;
    bool ok_hum = false;
    bool ok_press = false;
    // BME needs some idle time before we can measure it
    usleep(100 * 1000);
    if (bme280 != NULL) {
        bme280_read_avg(
            bme280,
            1,
            50,
            &temperature,
            &ok_temp,
            &humidity,
            &ok_hum,
            &pressure,
            &ok_press);
    } else {
        DBG_LOGW(TAG, "BME280 not available; sending battery only");
    }

    if (ok_temp) {
        DBG_PRINTF("Temperature(avg): %.2f C\n", temperature);
    }
    if (ok_hum) {
        DBG_PRINTF("Humidity(avg): %.2f %%\n", humidity);
    }
    if (ok_press) {
        DBG_PRINTF("Pressure(avg): %.2f hPa\n", pressure);
    }

    // 1) Start connecting to Wi-Fi after everything is sampled
    const int64_t wifi_start_us = esp_timer_get_time();
    err = wifi_init_sta_from_secret();
    if (err == ESP_OK) {
        wifi_started = true;
    } else {
        // If Wi-Fi can't start, sleeping immediately is safest for battery.
        DBG_LOGW(TAG, "Wi-Fi init failed: %s", esp_err_to_name(err));
        goto cleanup_sleep;
    }

    // 1/2 overlap) Wait for Wi-Fi connection up to a fixed timeout, including time spent sampling.
    const uint32_t wifi_timeout_ms = 20000;
    int64_t elapsed_ms = (esp_timer_get_time() - wifi_start_us) / 1000;
    if (wifi_started && !s_wifi_connected && elapsed_ms < (int64_t)wifi_timeout_ms) {
        (void)wait_for_wifi_connected((uint32_t)(wifi_timeout_ms - (uint32_t)elapsed_ms));
    }

    // 4/5) If Wi-Fi is established, send all successfully sampled fields in one go.
    if (wifi_started && s_wifi_connected) {
        const float *t_ptr = ok_temp ? &temperature : NULL;
        const float *h_ptr = ok_hum ? &humidity : NULL;
        const float *p_ptr = ok_press ? &pressure : NULL;
        const float *b_ptr = ok_batt ? &battery_v : NULL;

        if (t_ptr == NULL && h_ptr == NULL && p_ptr == NULL && b_ptr == NULL) {
            DBG_LOGW(TAG, "No readings available to send");
        } else {
            for (int attempt = 0; attempt < 3; attempt++) {
                esp_err_t send_err = thingspeak_send_readings_optional(t_ptr, h_ptr, p_ptr, b_ptr);
                if (send_err == ESP_OK) {
                    sent_ok = true;
                    break;
                }
                DBG_LOGW(TAG, "ThingSpeak send failed (attempt %d/3): %s", attempt + 1, esp_err_to_name(send_err));

                // Small delay before retry.
                usleep(2000 * 1000);
                if (!s_wifi_connected) {
                    (void)esp_wifi_connect();
                    (void)wait_for_wifi_connected(5000);
                }
            }
        }
    } else {
        DBG_LOGW(TAG, "Wi-Fi not connected; skipping ThingSpeak update");
    }

    // Clean up
cleanup_sleep:
    if (bme280 != NULL) {
        // Put BME280 into sleep mode before ESP deep sleep (redundant if we power off using GPIO).
        // esp_err_t sleep_result = bme280_set_sampling(bme280,
        //                                              BME280_MODE_SLEEP,
        //                                              BME280_SAMPLING_NONE,
        //                                              BME280_SAMPLING_NONE,
        //                                              BME280_SAMPLING_NONE,
        //                                              BME280_FILTER_OFF,
        //                                              BME280_STANDBY_MS_0_5);
        // if (sleep_result != ESP_OK) {
        //     DBG_PRINTF("Warning: failed to put BME280 into sleep mode (%d)\n", (int)sleep_result);
        // }
        bme280_delete(&bme280);
    }
    if (i2c_bus != NULL) {
        i2c_bus_delete(&i2c_bus);
    }
    adc_gpio0_deinit();

    disable_bme280_power();

    // Stop Wi-Fi before entering deep sleep.
    if (wifi_started) {
        (void)esp_wifi_stop();
    }

    // 6) Sleep for 30 minutes.
    DBG_PRINTF("Done (sent=%d). Sleeping for 30 minutes...\n", sent_ok);
    #if DEBUG_LED_SOLID_ON
        // Turn off LED before sleep.
        gpio_set_level(LED_GPIO, 0);
    #endif
    esp_deep_sleep(30ULL * 60ULL * 1000000ULL);
}