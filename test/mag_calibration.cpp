#include <Arduino.h>
#include <Wire.h>

#include "ahrs.hpp"
#include <magnetometer.hpp>
#include <packets_vars_functions.hpp>

void setup() {
    Serial.begin(115200);
    delay(500);

    Wire.begin(0, 1); // default I2C pins on ESP32C3 are SDA GPIO8 and SCL GPIO9
    Wire.setClock(400000); // set I2C clock to 400 kHz for faster data
    EEPROM.begin(24); // save 24 bytes which is enough for 6 floats
    
    compass_main.initialize();

    compass_main.manual_calibration = true; // change to false to use
    compass_main.calibrate();

    Serial.println("Calibration finished");
}

void loop() { }