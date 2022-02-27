const char compile_date[] = __DATE__ " " __TIME__;
#define ESP_DRD_USE_LITTLEFS false
#define ESP_DRD_USE_SPIFFS false
#define ESP_DRD_USE_EEPROM true

#define DOUBLERESETDETECTOR_DEBUG false  //false

#include <ESP_DoubleResetDetector.h>  //https://github.com/khoih-prog/ESP_DoubleResetDetector
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>           //https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h>  //https://github.com/me-no-dev/ESPAsyncWebServer
#include <AsyncElegantOTA.h>    //https://github.com/ayushsharma82/AsyncElegantOTA
#include <ESP32CAN.h>           //https://github.com/miwagner/ESP32-Arduino-CAN
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
float temp = 0.0;

volatile int interruptCounter;
hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer0() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // Pin D2 mapped to pin GPIO2/ADC12 of ESP32, control on-board LED
#endif

#define LED_OFF LOW
#define LED_ON HIGH

#define VOLTAGE_PIN 36
float voltage = 0.0;

const char* ssid = "ESP32_UPDATE";
const char* password = "12345678";

AsyncWebServer server(80);
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

CAN_device_t CAN_cfg;          // CAN Config
const int rx_queue_size = 10;  // Receive Queue size

void setup(void) {
  Serial.begin(250000);
  pinMode(LED_BUILTIN, OUTPUT);
  timer = timerBegin(0, 40000, true);  // 40MHz Xtal clock
  timerAttachInterrupt(timer, &onTimer0, true);
  timerAlarmWrite(timer, 12000, true);  // 120000 = 6 seconds
  timerAlarmEnable(timer);
  CAN_cfg.speed = CAN_SPEED_250KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));

  // Set CAN Filter
  // See in the SJA1000 Datasheet chapter "6.4.15 Acceptance filter"
  // and the APPLICATION NOTE AN97076 chapter "4.1.2 Acceptance Filter"
  // for PeliCAN Mode
  CAN_filter_t p_filter;
  p_filter.FM = Single_Mode;

  p_filter.ACR0 = 0x500;
  p_filter.ACR1 = 0;
  p_filter.ACR2 = 0;
  p_filter.ACR3 = 0;

  p_filter.AMR0 = 0xFF;
  p_filter.AMR1 = 0xFF;
  p_filter.AMR2 = 0xFF;
  p_filter.AMR3 = 0xFF;
  ESP32Can.CANConfigFilter(&p_filter);

  // Init CAN Module
  ESP32Can.CANInit();

  Serial.println("\nESP_DoubleResetDetector");
  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  if (drd->detectDoubleReset()) {
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
      request->send(200, "text/plain", "Hi! I am ESP32 Interface for CPX Motorcycle. \nFirmware: " __FILE__ "\nCompiled: " __DATE__ " " __TIME__);
    });

    AsyncElegantOTA.begin(&server);  // Start ElegantOTA
    server.begin();
    Serial.println("HTTP server started");
  } else {
    Serial.println("No Double Reset Detected");
    digitalWrite(LED_BUILTIN, LED_OFF);
    WiFi.mode(WIFI_OFF);
  }

  SerialBT.begin("ESP32_CPX");
  Serial.println("The device started, now you can pair it with bluetooth!");

  pinMode(VOLTAGE_PIN, INPUT);
  Serial.print("Compile timestamp: ");
  Serial.println(compile_date);
  voltage = ((analogRead(VOLTAGE_PIN)) * 0.00174);
  Serial.print("Power Supply Voltage: ");
  Serial.print(voltage, 3);
  Serial.println(" V");
  Serial.print("Internal Core Temperature: ");
  Serial.print((temprature_sens_read() - 32) / 1.8);
  Serial.println(" ºC");
  delay(4000);
}

void loop(void) {
  CAN_frame_t rx_frame;
  //receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    if (rx_frame.FIR.B.FF == CAN_frame_std) {
      //printf("New standard frame");
    } else {
      //printf("New extended frame");
    }

    if (rx_frame.FIR.B.RTR == CAN_RTR) {
      //printf(" RTR from 0x%04X, DLC %d\r\n", rx_frame.MsgID, rx_frame.FIR.B.DLC);
    } else {
      if ((0x500 != rx_frame.MsgID) && (0x501 != rx_frame.MsgID) && (0x503 != rx_frame.MsgID) && (0x510 != rx_frame.MsgID)) {
        //printf(" from 0x%04X, DLC %d, Data ", rx_frame.MsgID, rx_frame.FIR.B.DLC);
        SerialBT.write((rx_frame.MsgID >> 8) & 255);
        delay(5);
        SerialBT.write(rx_frame.MsgID & 255);
        delay(20);
        SerialBT.write(0x88);
        delay(20);
        for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
          //printf("0x%02X ", rx_frame.data.u8[i]);
          SerialBT.write(rx_frame.data.u8[i]);
        }
        //printf("\n");
        delay(20);
        SerialBT.write(0x0D);
        delay(20);
      }
    }
    delay(70);
  }
  drd->loop();
  if (interruptCounter > 19) {  // 120 secs pass
    portENTER_CRITICAL(&timerMux);
    interruptCounter = 0;
    portEXIT_CRITICAL(&timerMux);
    voltage = ((analogRead(VOLTAGE_PIN)) * 0.00174);
    unsigned int n = voltage * 100;
    byte V1 =  n >> 8;
    byte V2 =  n & 0xFF;
    temp = ((temprature_sens_read() - 32) / 1.8);
    unsigned int t = temp * 10;
    byte T1 =  t >> 8;
    byte T2 =  t & 0xFF;
    SerialBT.write(0x0D);
    SerialBT.write(0x05);
    SerialBT.write(0x50);
    SerialBT.write(0x88);
    SerialBT.write(T2);
    SerialBT.write(T1);
    SerialBT.write(0x0);
    SerialBT.write(0x0);
    SerialBT.write(0x0);
    SerialBT.write(V2);
    SerialBT.write(V1);
    SerialBT.write(0x0);
    SerialBT.write(0x0D);
  }
}
