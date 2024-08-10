#pragma once
#include <Arduino.h>

namespace CubliMini {
namespace Config {

// WIFI
#define WIFI_DEFAULT_SSID            "the_princess_castle"
#define WIFI_DEFAULT_PASSWORD        "13538305834"
#define WIFI_DEFAULT_TCP_SERVER_PORT 1312

// PARAM CONFIG
#define P_BALANCE_X_P   2     // 2
#define P_BALANCE_X_V   0.25f  // 0.25 
#define P_BALANCE_X_S   0.02f  // 0.02
#define P_BALANCE_X_A   6.0f

#define P_BALANCE_Y_P   -3.5f
#define P_BALANCE_Y_V   -0.35f
#define P_BALANCE_Y_S   0.03f
#define P_BALANCE_Y_A   -2.6

#define P_BALANCE_Z_P   0
#define P_BALANCE_Z_V   0.1f // 0.1
#define P_BALANCE_Z_S   -0.02f // 0.02
#define P_BALANCE_Z_A   0

#define U_BALANCE_CH3_P 3     //3
#define U_BALANCE_CH3_V 0.35f
#define U_BALANCE_CH3_S 0.02f
#define U_BALANCE_CH3_A -0.28f

// PIN CONFIG
#define MOTOR_LED_NUM  30
#define MOTOR1_LED_PIN 23
#define MOTOR2_LED_PIN 19
#define MOTOR3_LED_PIN 22

#define SYSTEM_LED_PIN 10
#define SYSTEM_LED_NUM 12

#define SYSTEM_KEY1_PIN (int)34
#define SYSTEM_KEY2_PIN (int)35

#define SERVO_MOTOR_1_PIN (int)27
#define SERVO_MOTOR_2_PIN (int)26
#define SERVO_MOTOR_3_PIN (int)25

#define CAN_TXD_PIN (int)15
#define CAN_RXD_PIN (int)13

#define LED_IN_PIN (int)10

#define IMU_SCL_PIN (int)18
#define IMU_SDA_PIN (int)5

#define IMU_INTERRUPT_1_PIN (int)9
#define IMU_INTERRUPT_2_PIN (int)4

#define IMU_INIT_GYRO_THRESHOLD     150
#define IMU_STATIC_GYRO_THRESHOLD   8

#define BAT_VOLTAGE_PIN (int)36

} // namespace CubliMini
} // namespace Config