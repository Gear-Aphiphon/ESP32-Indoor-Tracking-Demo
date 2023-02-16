#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <esp_task_wdt.h>

#define WDT_TIMEOUT 5

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

TaskHandle_t SensorTaskHandle;

uint8_t ManufacturerDataBuffer[100] = { 0 };

std::string UWBtoManufacturer(BLEAdvertisedDevice& device) {
  return BLEUtils::buildHexData(ManufacturerDataBuffer, (uint8_t*)device.getManufacturerData().data(), device.getManufacturerData().length());
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    BLEAddress deviceAddress = advertisedDevice.getAddress();
    String deviceAddressString = String(deviceAddress.toString().c_str());
    String deviceManufacturingString = String(UWBtoManufacturer(advertisedDevice).c_str());

    if ((advertisedDevice.getName() == "UWB DO") && (deviceManufacturingString.indexOf("444f444f8") >= 0)) {
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
    }
  }
};

void SensorTask(void *pvParameters) {
  Serial.print("Sensor running on core ");
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
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void setup() {
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    SensorTask,        /* Task function. */
    "SENS",            /* name of task. */
    0x2000,            /* Stack size of task */
    NULL,              /* parameter of the task */
    1,                 /* priority of the task */
    &SensorTaskHandle, /* Task handle to keep track of created task */
    0);                /* pin task to core 0 */
}


void loop() {

}
