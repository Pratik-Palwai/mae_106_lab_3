#ifndef ROBOT_VARS_HPP
#define ROBOT_VARS_HPP

#include <Arduino.h>

struct SensorPacket { float mag_x = 0.0, mag_y = 0.0, mag_z = 0.0; };

struct AHRSPacket { long timestamp = 0; float yaw = 0.0; };

double pid_input = 0.0, target_heading = 0.0, pid_output = 0.0; // pid input and output variables, used in actuation.hpp

AHRSPacket ahrs_packet_main; // because we are passing packets by reference we only ever need one instance of the packet
SensorPacket sensor_packet_main; // instead of creating copies the functions just modify the packets in place

void serialOutput(void *param) {
    while(1)
    {
        Serial.print(">timestamp:" + String(ahrs_packet_main.timestamp));
        Serial.print(",mag_x:" + String(sensor_packet_main.mag_x));
        Serial.print(",mag_y:" + String(sensor_packet_main.mag_y));
        Serial.print(",mag_z:" + String(sensor_packet_main.mag_z));
        Serial.print(",yaw:" + String(ahrs_packet_main.yaw));
        Serial.print("\r\n");

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#endif