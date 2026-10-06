#ifndef ACTUATION_HPP
#define ACTUATION_HPP

#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

#include "packets_vars_functions.hpp"

Servo steering_servo;
const int SERVO_PIN = D9;

const float K_P = 0.50, K_I = 0.0, K_D = 0.0;
PID steering_correction(&pid_input, &pid_output, &target_heading, K_P, K_I, K_D, REVERSE);  // pid mode can be DIRECT or REVERSE depending on how the servo and magnetometer are mounted

void steerRobot(void *param) {
    float initial_heading = 180.0;
    float final_heading = 270.0;

    while(1) {
        pid_input = ahrs_packet_main.yaw;

        steering_correction.Compute();
        steering_servo.write(90.0 + pid_output); // convert the +/- heading from the PID to a 0-180 command for the servo

        vTaskDelay(pdMS_TO_TICKS(25)); // steer the servo at 40 Hz
    }
}

#endif