#include <Arduino.h>
#include <Wire.h>
#include <magnetometer.hpp>

Magnetometer106 compass_main; // create an instance of the Magnetometer106 class called compass_main
MagPacket mag_packet_main; // create one instance of the magPacket which gets updated by the read() method

void setup() {
    Serial.begin(115200);
    delay(500);

    Wire.begin(0, 1); // default I2C pins on ESP32C3 are SDA GPIO8 and SCL GPIO9
    Wire.setClock(400000); // set I2C clock to 400 kHz for faster data
    EEPROM.begin(24); // save 24 bytes which is enough for 6 floats
    
    compass_main.initialize();
}