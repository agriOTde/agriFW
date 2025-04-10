# agriFW

Repo for the FW of the Agri Edge device capable of sending Temp and Humidity data to AWS MQTT Broker.

### MCU and SDK Based on 

* MCU: ESP32 WROOM.
* SDK: FREERTOS.

### Sensors

* SHT31 Temp/Hum Sensor

### Future Work

* Addition of PH Sensor
* Addition of Real time controlled Motor for Water scheduling.
* Sleep and other power saving routines.
* Saving of PKI stuff in eeprom
* Backup of data incase of losing connection. 
* Centralization of MQTT broker within the local Network
