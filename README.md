# ESP32-Indoor-Tracking-Demo

# Prerequisites for Software Development

1. Arduino IDE 
- https://wiki-content.arduino.cc/en/software

2. ESP32 Development for Arduino IDE
- https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/installing.html

3. MQTT Library
- https://github.com/knolleary/pubsubclient


# Example

- ESP32 BLE scanning for read BLE advertising data from WEDO Indoor tracker.
- Parse the BLE advertising data to XYZ position.
- Send position data to MQTT server via WIFI.
