#include <stdio.h>
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
#include "esp_wifi.h"
#include "nvs_flash.h"

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif
#include "bme280.h"
#include "i2c_bus.h"
#include "secret.h"

#define LED_GPIO            15
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
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC1_CH0_ATTEN,
        .bitwidth = ADC1_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC1_CH0_CHANNEL, &chan_cfg));

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
        ESP_LOGW(TAG, "ADC calibration not available; only raw readings will be shown");
    }

    ESP_LOGI(TAG, "ADC ready: GPIO%d (ADC1_CH0), atten=%d", (int)ADC1_CH0_GPIO, (int)ADC1_CH0_ATTEN);
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

static esp_err_t thingspeak_send_readings(float temperature_c, float humidity_pct, float pressure_hpa, float battery_v)
{
    // Convert hPa -> mmHg
    const float pressure_mmhg = pressure_hpa * 0.750061683f;

    char url[256];
    int written = snprintf(
        url,
        sizeof(url),
        "https://api.thingspeak.com/update?api_key=%s&field1=%.2f&field2=%.2f&field3=%.2f&field8=%.2f",
        THINKSPEAK_API_KEY,
        temperature_c,
        humidity_pct,
        pressure_mmhg,
        battery_v);
    if (written < 0 || written >= (int)sizeof(url)) {
        return ESP_ERR_INVALID_SIZE;
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
    ESP_LOGW(TAG, "ThingSpeak HTTPS needs CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y (see sdkconfig.defaults)");
    return ESP_ERR_INVALID_STATE;
#endif

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ThingSpeak: GET %s", url);
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "ThingSpeak: status=%d", status);
    } else {
        ESP_LOGW(TAG, "ThingSpeak: request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi: STA start, connecting...");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        }
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        ESP_LOGW(TAG, "Wi-Fi: disconnected, reconnecting...");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        s_wifi_connected = true;
        ESP_LOGI(TAG, "Wi-Fi: got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        return;
    }
}

static esp_err_t wifi_init_sta_from_secret(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    (void)esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init done (SSID=%s)", WIFI_SSID);
    return ESP_OK;
}

void app_main() {
    usleep(1000000);  // 1000ms delay to allow for any initial setup

    // Start Wi-Fi (event-driven; no user-created FreeRTOS tasks/event groups)
    ESP_ERROR_CHECK(wifi_init_sta_from_secret());

    // Configure LED GPIO as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    gpio_config(&io_conf);
    
    // Initialize I2C bus for BME280
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus_handle_t i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    
    // Initialize BME280 sensor
    bme280_handle_t bme280 = bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT);
    esp_err_t init_result = bme280_default_init(bme280);
    if (init_result != ESP_OK) {
        printf("Failed to initialize BME280 sensor (%d)\n", (int)init_result);
        return;
    }
    
    printf("LED %d blinking with BME280 readings\n", LED_GPIO);

    // Initialize ADC sampling on IO0 / ADC1_CH0.
    ESP_ERROR_CHECK(adc_gpio0_init());

    size_t count = 0;
    
    // Blink LED and read sensor data
    while (count < 10) {
        // Turn LED on
        gpio_set_level(LED_GPIO, 1);
        
        // Read sensor data
        float temperature = 0.0, humidity = 0.0, pressure = 0.0;
        bool ok_temp = false;
        bool ok_hum = false;
        bool ok_press = false;
        float battery_v = 0.0f;
        bool ok_batt = false;
        
        if (ESP_OK == bme280_read_temperature(bme280, &temperature)) {
            printf("Temperature: %.2f °C\n", temperature);
            ok_temp = true;
        }
        usleep(100000);  // 100ms delay
        
        if (ESP_OK == bme280_read_humidity(bme280, &humidity)) {
            printf("Humidity: %.2f %%\n", humidity);
            ok_hum = true;
        }
        usleep(100000);  // 100ms delay
        
        if (ESP_OK == bme280_read_pressure(bme280, &pressure)) {
            printf("Pressure: %.2f hPa\n", pressure);
            ok_press = true;
        }

        // Read the analog voltage on IO0 (ADC1_CH0).
        int adc_raw = 0;
        int adc_mv = 0;
        esp_err_t adc_err = adc_gpio0_read_voltage_mv(&adc_mv, &adc_raw);
        if (adc_err == ESP_OK) {
            printf("ADC IO0 (ADC1_CH0): raw=%d, voltage=%d mV\n", adc_raw, adc_mv);
            battery_v = (adc_mv * 2) / 1000.0f;
            ok_batt = true;
            printf("Battery voltage: %.2f V\n", battery_v);
        } else {
            // Calibration may not be available on all builds; show raw in that case.
            esp_err_t raw_err = adc_gpio0_read_raw(&adc_raw);
            if (raw_err == ESP_OK) {
                printf("ADC IO0 (ADC1_CH0): raw=%d (calibrated voltage unavailable: %s)\n",
                       adc_raw, esp_err_to_name(adc_err));
            } else {
                printf("ADC IO0 (ADC1_CH0): read failed: %s\n", esp_err_to_name(raw_err));
            }
        }
        
        usleep(300000);  // 300ms delay
        
        // Turn LED off
        gpio_set_level(LED_GPIO, 0);
        usleep(500000);  // 500ms delay
        
        printf("LED toggled (count: %zu)\n\n", count + 1);
        if (s_wifi_connected) {
            ESP_LOGI(TAG, "Wi-Fi connected");
        } else {
            ESP_LOGI(TAG, "Wi-Fi not connected yet");
        }

        // On the 10th (final) reading, send telemetry to ThingSpeak.
        if (count == 9) {
            if (!s_wifi_connected) {
                ESP_LOGW(TAG, "Skipping ThingSpeak update: Wi-Fi not connected");
            } else if (!(ok_temp && ok_hum && ok_press && ok_batt)) {
                ESP_LOGW(TAG, "Skipping ThingSpeak update: missing readings (t=%d h=%d p=%d batt=%d)",
                         ok_temp, ok_hum, ok_press, ok_batt);
            } else {
                (void)thingspeak_send_readings(temperature, humidity, pressure, battery_v);
            }
        }
        count++;
    }

    // Put BME280 into sleep mode before ESP deep sleep
    esp_err_t sleep_result = bme280_set_sampling(bme280,
                                                 BME280_MODE_SLEEP,
                                                 BME280_SAMPLING_NONE,
                                                 BME280_SAMPLING_NONE,
                                                 BME280_SAMPLING_NONE,
                                                 BME280_FILTER_OFF,
                                                 BME280_STANDBY_MS_0_5);
    if (sleep_result != ESP_OK) {
        printf("Warning: failed to put BME280 into sleep mode (%d)\n", (int)sleep_result);
    }

    // Clean up
    bme280_delete(&bme280);
    i2c_bus_delete(&i2c_bus);
    adc_gpio0_deinit();

    // Go into deep sleep and wake back up after 10 seconds
    printf("Entering deep sleep for 10 seconds...\n");
    esp_deep_sleep(10 * 1000000); // Sleep for 10 seconds
    printf("Woke up from deep sleep\n");
}