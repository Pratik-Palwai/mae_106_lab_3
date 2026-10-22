#include <Arduino.h>
#include <Wire.h>

#include "servo.hpp"
#include "magnetometer.hpp"
#include "packets_vars_functions.hpp"

// set this to 1 to run the code for lab 3 part 1. set this to 2 to run the code for part 2
const int LAB_SECTION = 2;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Wire.begin(3, 4); // default I2C pins on ESP32C3: SDA GPIO8 and SCL GPIO9
    Wire.setClock(400000); // Set I2C clock to 400kHz Fast Mode
    EEPROM.begin(24); // save 24 bytes: 3 axes * 2 values (scaling + offset) * 4 bytes per float
    
    compass_main.initialize();
    compass_main.manual_calibration = false; // change to false to use EEPROM values

    if (LAB_SECTION == 1) {
        compass_main.calibrate();

        Serial.println("Calibration finished");
    }

    else if (LAB_SECTION == 2) {
        compass_main.calibrate();

        steering_servo.attach(SERVO_PIN);
        steering_correction.SetOutputLimits(-40, 40);
        steering_correction.SetMode(AUTOMATIC);

        xTaskCreate(updateAHRS, "AHRS", 4096, NULL, 6, NULL);
        xTaskCreate(steerRobot, "STEER", 4096, NULL, 4, NULL);
        xTaskCreate(serialOutput, "SERIAL", 4096, NULL, 1, NULL);
    }
}

void loop() { } // nothing needs to be in loop() because FreeRTOS handles all