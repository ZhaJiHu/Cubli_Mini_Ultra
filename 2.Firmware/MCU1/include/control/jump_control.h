#pragma once

#include "bsp/servo_motor.h"
#include "config/config.h"
#include "control/p_balance_control.h"
#include "control/u_balance_control.h"
#include "control/pid.h"

namespace CubliMini {
namespace Control {
using namespace CubliMini::Bsp;

typedef enum JumpState_e
{
    IDLE,
    U_ACCELERATE,
    U_JUMPING,
    U_BALABCE,
    P_ACCELERATE,
    P_JUMPING,
    P_BALABCE
};

class JumpControl
{
   public:
    JumpControl(
        PBalanceControl &p_balance_control,
        UBalanceControl &u_balance_control,
        ServoInitialPosition_t &servo_initial_position)
        : p_balance_control_(p_balance_control),
          u_balance_control_(u_balance_control),
          servo_initial_position_(servo_initial_position),
          state_(IDLE)
    {
        servo_motor1.re_init();
        servo_motor2.re_init();
        servo_motor3.re_init();

        servo_motor1.setDegree(servo_initial_position_.ch1);
        servo_motor2.setDegree(servo_initial_position_.ch2);
        servo_motor3.setDegree(servo_initial_position_.ch3);
    }

    void Loop(
        PBalanceControl::PSensor &sensor,
        const PBalanceControl::MotorSpeed &motor_speed,
        bool is_static)
    {
        switch (state_)
        {
        case IDLE:
            /* code */
            break;

        default:
            break;
        }
    }

    PBalanceControl &p_balance_control_;
    UBalanceControl &u_balance_control_;
    ServoInitialPosition_t &servo_initial_position_;

    JumpState_e state_;
    ServoMotor servo_motor1 {SERVO_MOTOR_1_PIN, 13};
    ServoMotor servo_motor2 {SERVO_MOTOR_2_PIN, 14};
    ServoMotor servo_motor3 {SERVO_MOTOR_3_PIN, 15};
};

}  // namespace Control
}  // namespace CubliMini