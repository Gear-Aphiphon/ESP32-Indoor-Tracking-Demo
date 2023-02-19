#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include "PubSubClient.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

#define WDT_TIMEOUT 2

TaskHandle_t ScannerTaskHandle;

const char *ssid = "IndoorTracker";
const char *wifi_password = "developerday";

const char *mqtt_server = "192.168.161.133";
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

    if ((advertisedDevice.getName() == "UWB DO") && (deviceManufacturingString.indexOf("444f444f") >= 0)) {
      Serial.print("Timestamp: ");
      Serial.println(xTaskGetTickCount());
      Serial.print("UWB Device: ");
      Serial.println(deviceManufacturingString);
      
      uint8_t dataPacket[advertisedDevice.getManufacturerData().length()];
      memset(dataPacket, 0, advertisedDevice.getManufacturerData().length());
      memcpy(dataPacket, advertisedDevice.getManufacturerData().data(), advertisedDevice.getManufacturerData().length());

      int16_t positionX = (dataPacket[6] << 8) | dataPacket[7];
      int16_t positionY = (dataPacket[8] << 8) | dataPacket[9];
      int16_t positionZ = (dataPacket[10] << 8) | dataPacket[11];

      Serial.print("\tX = ");
      Serial.print(positionX/100.0f);
      Serial.println("m");

      Serial.print("\tY = ");
      Serial.print(positionY/100.0f);
      Serial.println("m");

      Serial.print("\tZ = ");
      Serial.print(positionZ/100.0f);
      Serial.println("m");

      if (WiFi.isConnected() && client.connected()) {
        client.publish(mqtt_topic, deviceManufacturingString.c_str(), deviceManufacturingString.length());
      }
      else {
        ESP.restart();
      }
    }
  }
};

static volatile boolean isScanning = false;

void scanComplete(BLEScanResults scanResults)
{
  Serial.print("Scan complete: ");
  Serial.println(millis());
  isScanning = false;
}

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
    if (!isScanning) {
      isScanning = true;
      pBLEScan->stop();
      pBLEScan->start(1, scanComplete, false);
    }
    
    esp_task_wdt_reset();
    client.loop();
  }
}

void WiFiConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Successfully connected to Access Point");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("WIFI is connected!");
  Serial.println("IP anchorAddress: ");
  Serial.println(WiFi.localIP());
}

void WiFiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WIFI access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Reconnecting...");
  WiFi.begin(ssid, wifi_password);
}

void ConnectWifi() {
  WiFi.onEvent(WiFiConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.begin(ssid, wifi_password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED);
}


void ConnectMQTT() {
  int random_clientID = random(1000000);
  snprintf(clientID, sizeof(clientID), "%06d", random_clientID);
  while (!client.connected()) {
    if (client.connect(clientID, mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker!");
    } else {
      Serial.println("Connection to MQTT Broker failed...");
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


void setup() {
  Serial.begin(115200);
  ConnectWifi();
  ConnectMQTT();

  xTaskCreate(
    ScannerTask,        /* Task function. */
    "SCAN",             /* name of task. */
    0x2000,             /* Stack size of task */
    NULL,               /* parameter of the task */
    1,                  /* priority of the task */
    &ScannerTaskHandle);
}


void loop() {
}
