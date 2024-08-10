#pragma once
#include <Arduino.h>

#define MOTOR_PP 7
#define MOTOR_PHASE_RESISTANCE 0.208f
#define MOTOR_KV 1800

#define MOTOR1_CS1      (int)2
#define MOTOR1_PWM1_PIN (int)21
#define MOTOR1_PWM2_PIN (int)18
#define MOTOR1_PWM3_PIN (int)14  // 14
#define MOTOR1_LIN_PINA (int)37  // CAPP
#define MOTOR1_LIN_PINB (int)35

#define MOTOR2_CS2      (int)5
#define MOTOR2_PWM1_PIN (int)25
#define MOTOR2_PWM2_PIN (int)26
#define MOTOR2_PWM3_PIN (int)27
#define MOTOR2_LIN_PINA (int)39  // VN
#define MOTOR2_LIN_PINB (int)38  // CAPN

#define MOTOR3_CS3      (int)9
#define MOTOR3_PWM1_PIN (int)22
#define MOTOR3_PWM2_PIN (int)19
#define MOTOR3_PWM3_PIN (int)23
#define MOTOR3_LIN_PINA (int)36  // VP
#define MOTOR3_LIN_PINB (int)34

#define MOTOR_SSI_SCK   (int)10
#define MOTOR_SSI_DO    (int)4

#define CAN_TXD_PIN (int)15
#define CAN_RXD_PIN (int)13

#define MOS_POWER_PIN (int)33