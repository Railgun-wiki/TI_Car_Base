#ifndef CAR_BASE_BUILD_CONFIG_H
#define CAR_BASE_BUILD_CONFIG_H

// Application selection. Override APP_ACTIVE with a compiler -D option when
// building another image; the identifiers remain stable across CCS configs.
#define APP_LINE_FOLLOW 0
#define APP_H_QUESTION 1
#define APP_IMU_OLED_TEST 2
#define APP_MOTOR_CENTER_TEST 3

#ifndef APP_ACTIVE
#define APP_ACTIVE APP_H_QUESTION
#endif

#if APP_ACTIVE != APP_LINE_FOLLOW && APP_ACTIVE != APP_H_QUESTION &&           \
    APP_ACTIVE != APP_IMU_OLED_TEST && APP_ACTIVE != APP_MOTOR_CENTER_TEST
#error "APP_ACTIVE must select a supported application"
#endif

// Attitude backend selection.
#define ATTITUDE_BACKEND_DMP 0
#define ATTITUDE_BACKEND_COMPLEMENTARY 1
#define ATTITUDE_BACKEND_KALMAN 2
#define ATTITUDE_BACKEND_MAHONY 3
#define ATTITUDE_BACKEND_BMI270 4

#ifndef ATTITUDE_CONFIG_BACKEND
#define ATTITUDE_CONFIG_BACKEND ATTITUDE_BACKEND_MAHONY
#endif

#if ATTITUDE_CONFIG_BACKEND != ATTITUDE_BACKEND_DMP &&                         \
    ATTITUDE_CONFIG_BACKEND != ATTITUDE_BACKEND_COMPLEMENTARY &&               \
    ATTITUDE_CONFIG_BACKEND != ATTITUDE_BACKEND_KALMAN &&                      \
    ATTITUDE_CONFIG_BACKEND != ATTITUDE_BACKEND_MAHONY &&                      \
    ATTITUDE_CONFIG_BACKEND != ATTITUDE_BACKEND_BMI270
#error "ATTITUDE_CONFIG_BACKEND must select a supported backend"
#endif

#ifndef BMI270_ONBOARD_FUSION
#define BMI270_ONBOARD_FUSION 0
#endif

#if BMI270_ONBOARD_FUSION != 0 && BMI270_ONBOARD_FUSION != 1
#error "BMI270_ONBOARD_FUSION must be 0 or 1"
#endif

#endif
