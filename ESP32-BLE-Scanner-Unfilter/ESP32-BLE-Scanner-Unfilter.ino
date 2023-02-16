#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <esp_task_wdt.h>

#define WDT_TIMEOUT 2

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

TaskHandle_t ScannerTaskHandle;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    BLEAddress deviceAddress = advertisedDevice.getAddress();
    String deviceName = String(advertisedDevice.getName().c_str());
    if (!deviceName.isEmpty()) {
      Serial.print("Found Device: ");
      Serial.println(deviceName);
    }
  }
};

void ScannerTask(void *pvParameters) {
  Serial.print("Scanner running on core ");
  Serial.println(xPortGetCoreID());

  BLEDevice::init("BLE Scanner");
  BLEDevice::setPower(ESP_PWR_LVL_P7, ESP_BLE_PWR_TYPE_SCAN);

  BLEScan *pBLEScan;
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), false, true);
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


void setup() {
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    ScannerTask,        /* Task function. */
    "SCAN",             /* name of task. */
    0x2000,             /* Stack size of task */
    NULL,               /* parameter of the task */
    1,                  /* priority of the task */
    &ScannerTaskHandle, /* Task handle to keep track of created task */
    0);                 /* pin task to core 0 */
}


void loop() {

}
