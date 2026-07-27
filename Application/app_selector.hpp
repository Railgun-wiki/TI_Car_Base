#pragma once

#define APP_LINE_FOLLOW 0
#define APP_H_QUESTION 1
#define APP_IMU_OLED_TEST 2
#define APP_MOTOR_CENTER_TEST 3

#ifndef APP_ACTIVE
#define APP_ACTIVE APP_MOTOR_CENTER_TEST
#endif

#if APP_ACTIVE != APP_LINE_FOLLOW && APP_ACTIVE != APP_H_QUESTION &&           \
    APP_ACTIVE != APP_IMU_OLED_TEST && APP_ACTIVE != APP_MOTOR_CENTER_TEST
#error "APP_ACTIVE must select a supported application"
#endif
