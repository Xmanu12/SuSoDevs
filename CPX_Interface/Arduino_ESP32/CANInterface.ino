const char compile_date[] = __DATE__ " " __TIME__;
#define ESP_DRD_USE_LITTLEFS    false
#define ESP_DRD_USE_SPIFFS      false
#define ESP_DRD_USE_EEPROM      true

#define DOUBLERESETDETECTOR_DEBUG       true  //false

#include <ESP_DoubleResetDetector.h>    //https://github.com/khoih-prog/ESP_DoubleResetDetector
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>                   //https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h>          //https://github.com/me-no-dev/ESPAsyncWebServer
#include <AsyncElegantOTA.h>            //https://github.com/ayushsharma82/AsyncElegantOTA
#include <ESP32CAN.h>                   //https://github.com/nhatuan84/arduino-esp32-can-demo
#include <CAN_config.h>

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 5

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

DoubleResetDetector* drd;
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

#ifndef LED_BUILTIN
#define LED_BUILTIN       2         // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED
#endif

#define LED_OFF     LOW
#define LED_ON      HIGH

#define VOLTAGE_PIN 36

const char* ssid = "ESP32_UPDATE";
const char* password = "12345678";

AsyncWebServer server(80);
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

CAN_device_t CAN_cfg;               // CAN Config
const int rx_queue_size = 10;       // Receive Queue size

void setup(void) {
  Serial.begin(250000);
  SerialBT.begin("ESP32_CPX"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(LED_BUILTIN, OUTPUT);
  CAN_cfg.speed = CAN_SPEED_250KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();

  Serial.println("\nESP_DoubleResetDetector");
  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  if (drd->detectDoubleReset())
  {
    Serial.println("Double Reset Detected");
    digitalWrite(LED_BUILTIN, LED_ON);
    //SerialBT.end();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "Hi! I am ESP32 Interface for CPX Motorcycle.");
    });

    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    Serial.println("HTTP server started");
  }
  else
  {
    Serial.println("No Double Reset Detected");
    digitalWrite(LED_BUILTIN, LED_OFF);
    WiFi.mode(WIFI_OFF);
  }

  SerialBT.begin("ESP32_CPX");

  float voltage = 0.0;        
  pinMode(VOLTAGE_PIN, INPUT);
  voltage = ((analogRead(VOLTAGE_PIN)) * 0.00174 );
  Serial.print("Power Supply Voltage: "); Serial.println(voltage, 3);
  Serial.print("Compile timestamp: ");
  Serial.println(compile_date);
  Serial.print("Internal Core Temperature: ");
  Serial.print((temprature_sens_read() - 32) / 1.8);
  Serial.println(" ºC");
  delay(4000);
  SerialBT.print("Power Supply Voltage: ");
  SerialBT.println(voltage, 3);
  SerialBT.print("Compile timestamp: ");
  SerialBT.println(compile_date);
  SerialBT.print("Internal Core Temperature: ");
  SerialBT.print((temprature_sens_read() - 32) / 1.8);
  SerialBT.println(" C");
  SerialBT.write(0x0D);
}

void loop(void) {
  CAN_frame_t rx_frame;
  //receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    if (rx_frame.FIR.B.FF == CAN_frame_std) {
      //printf("New standard frame");
    }
    else {
      printf("New extended frame");
    }

    if (rx_frame.FIR.B.RTR == CAN_RTR) {
      printf(" RTR from 0x%04X, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    }
    else {
      printf(" from 0x%04X, DLC %d, Data ", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
      SerialBT.write(rx_frame.MsgID);
      SerialBT.write(0x88);
      for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
        printf("0x%02X ", rx_frame.data.u8[i]);
        SerialBT.write(rx_frame.data.u8[i]);
      }
      printf("\n");
      SerialBT.write(0x0D);
    }
    delay(70);
    //respond to sender
    //ESP32Can.CANWriteFrame(&rx_frame);
  }
  //drd->loop();
}
