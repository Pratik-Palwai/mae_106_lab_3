#ifndef MAGNETOMETER_HPP
#define MAGNETOMETER_HPP

#include <Arduino.h>
#include <EEPROM.h>
#include <LIS3MDL.h>
#include <filters.h>

#include "packets_vars_functions.hpp"

#define EEPROM_MAG_X_OFFSET_ADDRESS 0 // a float uses 4 bytes of memory so the addresses are spaced 4 bytes apart
#define EEPROM_MAG_X_SCALING_ADDRESS 4
#define EEPROM_MAG_Y_OFFSET_ADDRESS 8
#define EEPROM_MAG_Y_SCALING_ADDRESS 12
#define EEPROM_MAG_Z_OFFSET_ADDRESS 16
#define EEPROM_MAG_Z_SCALING_ADDRESS 20

const int MAG_CAL_SAMPLES = 10000;

const float CUTOFF_FREQ = 20.0; // Hz
const float SAMPLING_TIME = 0.005; // sec
IIR::ORDER order = IIR::ORDER::OD3;

class Magnetometer106 {
    LIS3MDL sensor; // core sensor, the STM LIS3MDL

    float x_offset = 0.0, x_scaling = 1.0; // offsets are for hard-iron calibration
    float y_offset = 0.0, y_scaling = 1.0; // scaling factors are for soft-iron calibration
    float z_offset = 0.0, z_scaling = 1.0;

    Filter x_filter{CUTOFF_FREQ, SAMPLING_TIME, order};
    Filter y_filter{CUTOFF_FREQ, SAMPLING_TIME, order};

    void calibrateToEEPROM() {
        float x_min = 2147483646, x_max = -2147483646; // 32-bit integer limits
        float y_min = 2147483646, y_max = - 2147483646;
        float z_min = 2147483646, z_max = - 2147483646;

        Serial.print("Starting magnetometer calibration loop ... ");

        for (int i = 0; i < MAG_CAL_SAMPLES; ++i) {
            sensor.read();
        
            if (sensor.m.x < x_min) { x_min = sensor.m.x; }
            if (sensor.m.x > x_max) { x_max = sensor.m.x; }
            if (sensor.m.y < y_min) { y_min = sensor.m.y; }
            if (sensor.m.y > y_max) { y_max = sensor.m.y; }
            if (sensor.m.z < z_min) { z_min = sensor.m.z; }
            if (sensor.m.z > z_max) { z_max = sensor.m.z; }

            delay(1);
        }

        // xyz offsets move the calibration sphere center to the origin (hard-iron)
        x_offset = (x_max + x_min) / 2.0;
        y_offset = (y_max + y_min) / 2.0;
        z_offset = (z_max + z_min) / 2.0;

        float radius_x = (x_max - x_min) / 2.0;
        float radius_y = (y_max - y_min) / 2.0;
        float radius_z = (z_max - z_min) / 2.0;
        float radius_avg = (radius_x + radius_y + radius_z) / 3.0;

        // if the magnetic field is distorted the scaling factors shape the ellipse into a sphere (soft-iron)
        x_scaling = radius_avg / radius_x;
        y_scaling = radius_avg / radius_y;
        z_scaling = radius_avg / radius_z;

        Serial.println("finished");
        
        Serial.println("    mag_x_offset:" + String(x_offset) + " | mag_x_scaling:" + String(x_scaling));
        Serial.println("    mag_y_offset:" + String(y_offset) + " | mag_y_scaling:" + String(y_scaling));
        Serial.println("    mag_z_offset:" + String(z_offset) + " | mag_z_scaling:" + String(z_scaling));

        EEPROM.put(EEPROM_MAG_X_OFFSET_ADDRESS, x_offset);
        EEPROM.put(EEPROM_MAG_X_SCALING_ADDRESS, x_scaling);
        EEPROM.put(EEPROM_MAG_Y_OFFSET_ADDRESS, y_offset);
        EEPROM.put(EEPROM_MAG_Y_SCALING_ADDRESS, y_scaling);
        EEPROM.put(EEPROM_MAG_Z_OFFSET_ADDRESS, z_offset);
        EEPROM.put(EEPROM_MAG_Z_SCALING_ADDRESS, z_scaling);

        EEPROM.commit();
        Serial.println("    saved calibrations to EEPROM");
    }

    void calibrateFromEEPROM() {
        Serial.println("Pulling compass calibrations from EEPROM ...");

        EEPROM.get(EEPROM_MAG_X_OFFSET_ADDRESS, x_offset);
        EEPROM.get(EEPROM_MAG_X_SCALING_ADDRESS, x_scaling);
        EEPROM.get(EEPROM_MAG_Y_OFFSET_ADDRESS, y_offset);
        EEPROM.get(EEPROM_MAG_Y_SCALING_ADDRESS, y_scaling);
        EEPROM.get(EEPROM_MAG_Z_OFFSET_ADDRESS, z_offset);
        EEPROM.get(EEPROM_MAG_Z_SCALING_ADDRESS, z_scaling);

        Serial.println("    finished");

        Serial.println("    mag_x_offset:" + String(x_offset) + " | mag_x_scaling:" + String(x_scaling));
        Serial.println("    mag_y_offset:" + String(y_offset) + " | mag_y_scaling:" + String(y_scaling));
        Serial.println("    mag_z_offset:" + String(z_offset) + " | mag_z_scaling:" + String(z_scaling));
    }

public:
    bool manual_calibration = false; // decide whether to perform a manual calibration or pull previous values from EEPROM

    void initialize() {
        if (!sensor.init()) { // initializes the I2C communcation between the ESP32 and the sensor
            Serial.println("Failed to detect/initialize LIS3MDL");
            while(1);
        }

        else { Serial.println("Successfully detected and initialized LIS3MDL"); }
        sensor.enableDefault();
    }

    void calibrate() {
        if (manual_calibration) { calibrateToEEPROM(); }
        else { calibrateFromEEPROM(); }
    }

    // read the sensor values and place them into the given sensor packet
    void read(SensorPacket& sensor_packet) { // because sensor_packet is passed by reference(&) it avoids having to make excessive copies
        sensor.read();

        sensor_packet.mag_x = (sensor.m.x - x_offset) * x_scaling;
        sensor_packet.mag_y = (sensor.m.y - y_offset) * y_scaling;
        sensor_packet.mag_z = (sensor.m.z - z_offset) * z_scaling;
    }

    // calculate heading purely based on the compass data. this doesn't fuse the gyro or accelerometer sensors
    float calculateHeading(const SensorPacket& sensor_packet) {
        float mag_x_filtered = x_filter.filterIn(sensor_packet.mag_x);
        float mag_y_filtered = y_filter.filterIn(sensor_packet.mag_y);

        float heading_deg = atan2f(mag_y_filtered, mag_x_filtered) * RAD_TO_DEG;
        return heading_deg;
    }
};

Magnetometer106 compass_main; // creates an instance of the compass

// rtos task to update AHRS, executed at 0.5 kHz to prevent sensor packets from being used more than once
void updateAHRS(void *param) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(4);

    while(1) {
        compass_main.read(sensor_packet_main); // this avoids having to make excessive copies of each sensor packet
        ahrs_packet_main.yaw = compass_main.calculateHeading(sensor_packet_main);

        ahrs_packet_main.timestamp = static_cast<long>(micros());
        
        xTaskDelayUntil(&last_wake, period); // again, xTaskDelayUntil() gives better accuracy than a simple vTaskDelay()
    }
}

#endif