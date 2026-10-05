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

        if (heading_state == 0) { target_heading = initial_heading; }
        else { target_heading = final_heading; }
        if ((heading_state == 0) && (clicks_on_straight > CLICKS_BEFORE_TURN)) { heading_state = 1; }
        else if (heading_state == 1) {
            float angle_error = angleDiff(pid_input, target_heading);
            if (abs(angle_error) < 5.0) { heading_state = 2; } // transition from turning state to corridor state
                                                               // even though the heading state changes the target remains the same
            
            clicks_on_straight = 0; // don't increment this value while turning
        }

        // transition between corridor state to stopped state
        else if ((heading_state == 2) && (clicks_on_straight > CLICKS_AFTER_TURN)) { heading_state = 3; }

        steering_correction.Compute();
        steering_servo.write(90.0 + pid_output); // convert the +/- heading from the PID to a 0-180 command for the servo

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#endif