// Copyright (c) 2026 Apiluck Noppkun. All rights reserved.
// This source code is provided for personal use only.
// Redistribution, resale, or commercial use without written
// permission from the author is strictly prohibited.
// Contact: apiluck.banh@gmail.com

#ifndef LUNA_ROBOT_H
#define LUNA_ROBOT_H
#define LUNA_ROBOT
#define LED_PIN 48
#define LUNA_BASE MECANUM
#define LUNA_MOTOR_DRIVER
#define IMU_MPU6050
#define SDA_PIN 2
#define SCL_PIN 1

#define K_P1 0.01
#define K_I1 0.8
#define K_D1 0.2

#define K_P2 0.01
#define K_I2 0.9
#define K_D2 0.2

#define K_P3 0.01
#define K_I3 0.9
#define K_D3 0.2

#define K_P4 0.01
#define K_I4 0.9
#define K_D4 0.2

#define MOTOR_MAX_RPM 60
#define MAX_RPM_RATIO 0.8
#define MOTOR_OPERATING_VOLTAGE 6
#define MOTOR_POWER_MAX_VOLTAGE 12
#define MOTOR_POWER_MEASURED_VOLTAGE 12
#define COUNTS_PER_REV1 2400
#define COUNTS_PER_REV2 2400
#define COUNTS_PER_REV3 2400
#define COUNTS_PER_REV4 2400
#define WHEEL_DIAMETER 0.065
#define LR_WHEELS_DISTANCE 0.1975
#define PWM_BITS 8
#define PWM_FREQUENCY 8000

/// ENCODER PINS
#define MOTOR1_ENCODER_A 13
#define MOTOR1_ENCODER_B 12
#define MOTOR1_ENCODER_INV true

#define MOTOR2_ENCODER_A 11
#define MOTOR2_ENCODER_B 10
#define MOTOR2_ENCODER_INV false

#define MOTOR3_ENCODER_A 9
#define MOTOR3_ENCODER_B 8
#define MOTOR3_ENCODER_INV true

#define MOTOR4_ENCODER_A 14
#define MOTOR4_ENCODER_B 21
#define MOTOR4_ENCODER_INV false

// Motor Pins
#define MOTOR1_PWM -1
#define MOTOR1_IN_A 18
#define MOTOR1_IN_B 17
#define MOTOR1_INV false

#define MOTOR2_PWM -1
#define MOTOR2_IN_A 16
#define MOTOR2_IN_B 15
#define MOTOR2_INV true

#define MOTOR3_PWM -1
#define MOTOR3_IN_A 7
#define MOTOR3_IN_B 6
#define MOTOR3_INV true

#define MOTOR4_PWM -1
#define MOTOR4_IN_A 4
#define MOTOR4_IN_B 5
#define MOTOR4_INV true

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -(pow(2, PWM_BITS) - 1)

#define NODE_NAME "luna_robot_base_node"
#endif
