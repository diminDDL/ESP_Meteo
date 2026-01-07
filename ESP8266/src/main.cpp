#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
// #include <WiFi.h>
#include <WiFiClientSecure.h>
#include "secrets.h"
#include "ThingSpeak.h"

int keyIndex = 0;            
WiFiClient  client;

unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;

// ESP8266 specific
#define SDA_PIN D2
#define SCL_PIN D1

// ESP32 specific
// #define SDA_PIN 21
// #define SCL_PIN 22

//////////////////////////////

Adafruit_BME280 bme; // I2C

void go_to_sleep(unsigned int time = 1800000 * 4){
  ESP.deepSleep(time * 1000000);
}

int batADC;
float batVolt;
float h;
float t;
float p;

void setup() {
  Serial.begin(115200);
  delay(10);
  // Connect to WiFi network
  Serial.print("Connecting to ");
  Serial.println(SECRET_SSID);
 
  WiFi.begin(SECRET_SSID, SECRET_PASS);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if(millis()>6000){
      Serial.println("WiFi connection failed. Going to sleep.");
      go_to_sleep();
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");

  ThingSpeak.begin(client);
  Serial.println("Connecting to BME");
    Wire.begin(SDA_PIN, SCL_PIN);
    unsigned status;
    status = bme.begin(0x76, &Wire);

    bme.setSampling(Adafruit_BME280::MODE_FORCED,
        Adafruit_BME280::SAMPLING_X1, // temperature
        Adafruit_BME280::SAMPLING_X1, // pressure
        Adafruit_BME280::SAMPLING_X1, // humidity
        Adafruit_BME280::FILTER_OFF   );


  Serial.println("\nConnected.");
  //Read battery voltage
  batADC = analogRead(A0);
  batVolt = batADC/96.97;
  //read BME
  bme.takeForcedMeasurement();

  h = bme.readHumidity();
  t = bme.readTemperature();
  p = bme.readPressure() / 100.0F * 0.75006168270;
  if (h == 0.00 && t == 0.00 && p == 0.00 || !status) {
    Serial.println(F("Failed, a value was NaN!"));
    ThingSpeak.setStatus("Failed, a value was NaN!");
    bme.setSampling(Adafruit_BME280::MODE_SLEEP,
        Adafruit_BME280::SAMPLING_X1, // temperature
        Adafruit_BME280::SAMPLING_X1, // pressure
        Adafruit_BME280::SAMPLING_X1, // humidity
        Adafruit_BME280::FILTER_OFF   );
    go_to_sleep();
  }
  ThingSpeak.setField(1, t);
  ThingSpeak.setField(2, h);
  ThingSpeak.setField(3, p);
  ThingSpeak.setField(8, batVolt);
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  Serial.print(F("Battery voltage: "));
  Serial.print(batVolt);
  Serial.print(F(" Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("C  Pressure: "));
  Serial.print(p);
  Serial.println("mmHg");
  bme.setSampling(Adafruit_BME280::MODE_SLEEP,
        Adafruit_BME280::SAMPLING_X1, // temperature
        Adafruit_BME280::SAMPLING_X1, // pressure
        Adafruit_BME280::SAMPLING_X1, // humidity
        Adafruit_BME280::FILTER_OFF   );
  go_to_sleep();
}

void loop() {
}