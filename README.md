# agriFW

Repo for the FW of the Agri Edge device capable of sending Temp and Humidity data to AWS MQTT Broker.

### MCU and SDK Based on 

* MCU: ESP32 WROOM.
* SDK: FREERTOS.

### Sensors

* SHT31 Temp/Hum Sensor
* Ph sensor Included

### Features

* FreeRTOS based Real time system
* MQTT based topic logging and Subscription
* MQTT based OTA Update
* Motor Control 
* Improved Error and Connection Handling

### Future Work

* MQTT TLS implementation
* Sleep and other power saving routines.
* Saving of PKI stuff in eeprom
* Backup of data incase of losing connection. 

