#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include "PubSubClient.h"

#define WDT_TIMEOUT 2

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

TaskHandle_t ScannerTaskHandle;
TaskHandle_t WifiTaskHandle;

const char *ssid = "WEDO_devday_1";
const char *wifi_password = "developerday";

const char *mqtt_server = "192.168.2.102";
const char *mqtt_topic = "sensor/uwb";
const char *mqtt_username = "wedo";    // MQTT username
const char *mqtt_password = "123456";  // MQTT password

char clientID[7] = "";

WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient);

uint8_t ManufacturerDataBuffer[100] = { 0 };

std::string UWBtoManufacturer(BLEAdvertisedDevice& device) {
  return BLEUtils::buildHexData(ManufacturerDataBuffer, (uint8_t*)device.getManufacturerData().data(), device.getManufacturerData().length());
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    BLEAddress deviceAddress = advertisedDevice.getAddress();
    String deviceAddressString = String(deviceAddress.toString().c_str());
    String deviceManufacturingString = String(UWBtoManufacturer(advertisedDevice).c_str());

    if ((advertisedDevice.getName() == "UWB DO") && (deviceManufacturingString.indexOf("444f444f") >= 0) && client.connected()) {
      Serial.print("Timestamp: ");
      Serial.println(xTaskGetTickCount());
      Serial.print("UWB Device: ");
      Serial.println(deviceManufacturingString);
      Serial.println(deviceManufacturingString.c_str());

      if (WiFi.isConnected() && client.connected()) {
        client.publish(mqtt_topic, deviceManufacturingString.c_str(), deviceManufacturingString.length());
      }
      else {
        ESP.restart();
      }
    }
  }
};

void ScannerTask(void *pvParameters) {
  Serial.print("Scanner running on core ");
  Serial.println(xPortGetCoreID());

  BLEDevice::init("UWB Scanner");
  BLEDevice::setPower(ESP_PWR_LVL_P7, ESP_BLE_PWR_TYPE_SCAN);

  BLEScan *pBLEScan;
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true, true);
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(100);

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  for (;;) {
    esp_task_wdt_reset();
    BLEScanResults foundDevices = pBLEScan->start(1, false);
    pBLEScan->clearResults();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ConnectWifi() {
  static unsigned long t = 0;
  WiFi.begin(ssid, wifi_password);
  for (uint i = 0; i < 50; i++) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(100));
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("IP Address (AP): ");
      Serial.println(WiFi.localIP());
      return;
    }
  }
}


void ConnectMQTT() {
  int random_clientID = random(1000000);
  snprintf(clientID, sizeof(clientID), "%06d", random_clientID);
  if (!client.connected()) {
    if (client.connect(clientID, mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker!");
    } else {
      Serial.println("Connection to MQTT Broker failed...");
    }
  }
}


void WifiTask(void *pvParameters) {
  Serial.print("Wifi running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    if (!(WiFi.isConnected() && client.connected())) {
      ESP.restart();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void setup() {
  Serial.begin(115200);
  ConnectWifi();
  ConnectMQTT();

  xTaskCreatePinnedToCore(
    WifiTask,        /* Task function. */
    "WIFI",          /* name of task. */
    0x2000,          /* Stack size of task */
    NULL,            /* parameter of the task */
    1,               /* priority of the task */
    &WifiTaskHandle, /* Task handle to keep track of created task */
    0);              /* pin task to core 0 */

  xTaskCreatePinnedToCore(
    ScannerTask,        /* Task function. */
    "SENS",             /* name of task. */
    0x2000,             /* Stack size of task */
    NULL,               /* parameter of the task */
    1,                  /* priority of the task */
    &ScannerTaskHandle, /* Task handle to keep track of created task */
    1);                 /* pin task to core 0 */
}


void loop() {
}
