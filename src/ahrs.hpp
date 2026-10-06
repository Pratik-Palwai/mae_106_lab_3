#ifndef AHRS_HPP
#define AHRS_HPP

#include "magnetometer.hpp"

Magnetometer106 compass_main; // creates an instance of the compass, defined in "magnetometer.hpp"

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