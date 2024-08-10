#include <SimpleFOC.h>

#include "SPI.h"
#include "bsp/can.h"
#include "comm/comm.h"
#include "comm/time.h"
#include "config/config.h"
#include "mt6701/MagneticSensorMT6701SSI.h"

using namespace Cubli::Comm;
using namespace Cubli::Bsp;

#define MOTOR_MAX_TORQUE 15.0f  // rad/s
#define MOTOR_MAX_SPEED  650   // rad/s

BLDCMotor motor1       = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE, MOTOR_KV);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOTOR1_PWM1_PIN, MOTOR1_PWM2_PIN, MOTOR1_PWM3_PIN);

BLDCMotor motor2       = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE, MOTOR_KV);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOTOR2_PWM1_PIN, MOTOR2_PWM2_PIN, MOTOR2_PWM3_PIN);

BLDCMotor motor3       = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE, MOTOR_KV);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(MOTOR3_PWM1_PIN, MOTOR3_PWM2_PIN, MOTOR3_PWM3_PIN);

 float set_ch1_motor_speed = 0;
 float set_ch2_motor_speed = 0;
 float set_ch3_motor_speed = 0;

float motor_ch1_speed;
float motor_ch2_speed;
float motor_ch3_speed;

Cubli::Bsp::MotorTransMode motor_trans_mode;

Commander command = Commander(Serial);

void CommTask(void *parameter);
void CanReceiveTask(void *parameter);
void CanSendTask(void *parameter);

float mvPa = 200 * 5.0f * 2.0f / 3.0f / 2.0f;
#define mvPa (float)200

InlineCurrentSense current_sense1 =
    InlineCurrentSense(0.0009f, mvPa, MOTOR1_LIN_PINA, MOTOR1_LIN_PINB);
InlineCurrentSense current_sense2 =
    InlineCurrentSense(0.0009f, mvPa, MOTOR2_LIN_PINA, MOTOR2_LIN_PINB);
InlineCurrentSense current_sense3 =
    InlineCurrentSense(0.0009f, mvPa, MOTOR3_LIN_PINA, MOTOR3_LIN_PINB);

float mode_control = 1;
void doMotor1(char *cmd) { command.motor(&motor3, cmd); }
void doCh1Speed(char *cmd) { command.scalar(&set_ch1_motor_speed, cmd); }
void doCh2Speed(char *cmd) { command.scalar(&set_ch2_motor_speed, cmd); }
void doCh3Speed(char *cmd) { command.scalar(&set_ch3_motor_speed, cmd); }
void doModeControl(char *cmd) { command.scalar(&mode_control, cmd); }

MagneticSensorMT6701SSI sensor1(MOTOR1_CS1);
MagneticSensorMT6701SSI sensor2(MOTOR2_CS2);
MagneticSensorMT6701SSI sensor3(MOTOR3_CS3);

SPIClass motor_spi(VSPI);
Cubli::Bsp::CanDriver can;

SemaphoreHandle_t motor_speed_semaphore;
SemaphoreHandle_t motor_set_speed_mutex;

// void setup()
// {
//     Serial.begin(115200);
//     delay(300);
//     pinMode(MOS_POWER_PIN, OUTPUT);
//     digitalWrite(MOS_POWER_PIN, LOW);

//     sensor1.init(&motor_spi);
//     sensor2.init(&motor_spi, true);
//     sensor3.init(&motor_spi, true);

//     current_sense1.init();
//     current_sense1.gain_a *= -1;

//     current_sense2.init();
//     current_sense2.gain_a *= -1;

//     current_sense3.init();
//     current_sense3.gain_a *= -1;
// }

// void loop()
// {
//     sensor1.update();
//     sensor2.update();
//     sensor3.update();
//     printf("motor1 angle: %0.3f vel: %0.3f motor2 angle: %0.3f vel: %0.3f motor3 angle: %0.3f"
//     "vel: %0.3f\r\n",
//         sensor1.getAngle(),
//         sensor1.getVelocity(),
//         sensor2.getAngle(),
//         sensor2.getVelocity(),
//         sensor3.getAngle(),
//         sensor3.getVelocity());

//     PhaseCurrent_s currents1 = current_sense1.getPhaseCurrents();
//     float current_magnitude1 = current_sense1.getDCCurrent();
//     PhaseCurrent_s currents2 = current_sense2.getPhaseCurrents();
//     float current_magnitude2 = current_sense2.getDCCurrent();
//     PhaseCurrent_s currents3 = current_sense3.getPhaseCurrents();
//     float current_magnitude3 = current_sense3.getDCCurrent();

//     // printf("motor1 a[%0.3f] mA b[%0.3f] mA c[%0.3f] mA cur[%0.3f] mA\n",
//     //     (currents1.a*-1000),
//     //     (currents1.b*1000),
//     //     (currents1.c*1000),
//     //     (current_magnitude1*1000));

//     //  printf("motor2 a[%0.3f] mA b[%0.3f] mA c[%0.3f] mA cur[%0.3f] mA\n",
//     //     (currents2.a*-1000),
//     //     (currents2.b*1000),
//     //     (currents2.c*1000),
//     //     (current_magnitude2*1000));

//     //  printf("motor3 a[%0.3f] mA b[%0.3f] mA c[%0.3f] mA cur[%0.3f] mA\n",
//     //     (currents3.a*-1000),
//     //     (currents3.b*1000),
//     //     (currents3.c*1000),
//     //     (current_magnitude3*1000));
//     delay(10);
// }

void FocInit(
    MagneticSensorMT6701SSI *sensor,
    BLDCDriver3PWM *driver,
    BLDCMotor *motor,
    InlineCurrentSense *current_sense)
{
    motor->linkSensor(sensor);
    // 驱动器设置
    driver->voltage_power_supply = 11.1;
    driver->init();
    motor->linkDriver(driver);

    current_sense->linkDriver(driver);
    current_sense->init();

    current_sense->gain_a *= -1;
    motor->linkCurrentSense(current_sense);

    motor->torque_controller = TorqueControlType::foc_current;
    motor->controller        = MotionControlType::torque;
    motor->motion_downsample = 0.0;

    // velocity loop PID
    motor->PID_velocity.P           = 1.0;
    motor->PID_velocity.I           = 20.0;
    motor->PID_velocity.D           = 0.0;
    motor->PID_velocity.output_ramp = 1000.0;
    motor->PID_velocity.limit       = 2.5;
    // Low pass filtering time constant
    motor->LPF_velocity.Tf = 0.005;

    // angle loop PID
    motor->P_angle.P           = 20.0;
    motor->P_angle.I           = 0.0;
    motor->P_angle.D           = 0.0;
    motor->P_angle.output_ramp = 0.0;
    motor->P_angle.limit       = 20.0;
    // Low pass filtering time constant
    motor->LPF_angle.Tf = 0.0;
    // current q loop PID
    motor->PID_current_q.P           = 0.5;
    motor->PID_current_q.I           = 50.0;  // 20
    motor->PID_current_q.D           = 0.0;
    motor->PID_current_q.output_ramp = 0.0;
    motor->PID_current_q.limit       = 0.85f;
    // Low pass filtering time constant
    motor->LPF_current_q.Tf = 0.002;
    // current d loop PID
    motor->PID_current_d.P           = 0.5;
    motor->PID_current_d.I           = 50.0;  // 50
    motor->PID_current_d.D           = 0.0;
    motor->PID_current_d.output_ramp = 0.0;
    motor->PID_current_d.limit       = 0.85;
    // Low pass filtering time constant
    motor->LPF_current_d.Tf = 0.005;
    // Limits
    motor->velocity_limit = 20.0;
    motor->voltage_limit  = 3.5f;  // 数字电源：3.0 电池：3.5
    motor->current_limit  = 3.5f;

    motor->useMonitoring(Serial);

    motor->monitor_downsample = 0;
    motor->monitor_variables  = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;

    // 电机初始化
    motor->init();
    motor->initFOC();
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    pinMode(MOS_POWER_PIN, OUTPUT);
    digitalWrite(MOS_POWER_PIN, HIGH); // LOW HIGH

    sensor1.init(&motor_spi);
    sensor2.init(&motor_spi, true);
    sensor3.init(&motor_spi, true);

    FocInit(&sensor1, &driver1, &motor1, &current_sense1);
    FocInit(&sensor2, &driver2, &motor2, &current_sense2);
    FocInit(&sensor3, &driver3, &motor3, &current_sense3);

    motor1.target = 0;
    motor2.target = 0;
    motor3.target = 0;

    motor_speed_semaphore = xSemaphoreCreateBinary();
    motor_set_speed_mutex = xSemaphoreCreateMutex();
    can.Init(CAN_TXD_PIN, CAN_RXD_PIN, CAN_SPEED_1000KBPS);


    xTaskCreatePinnedToCore(CommTask, "common task", 10000, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(CanReceiveTask, "can receive task", 10000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(CanSendTask, "can send task", 10000, NULL, 3, NULL, 1);

    command.add('M', doMotor1, "motor 1");
    command.add('T', doCh1Speed, "motor 1 speed");
    command.add('O', doCh2Speed, "motor 2 speed");
    command.add('P', doCh3Speed, "motor 3 speed");
    command.add('L', doModeControl, "mode control");
    _delay(1000);
}
uint8_t control_mode_ = 0;
void CanReceiveTask(void *parameter)
{
    float get_set_motor1_speed = 0.0f;
    float get_set_motor2_speed = 0.0f;
    float get_set_motor3_speed = 0.0f;
    uint8_t get_motor_trans_mode;
    uint8_t control_temp;
    for (;;)
    {
        if (can.CanGetMotorSpeed(get_set_motor1_speed, get_set_motor2_speed, get_set_motor3_speed, get_motor_trans_mode, control_temp))
        {
            if (xSemaphoreTake(motor_set_speed_mutex, portMAX_DELAY) == pdTRUE)
            {
                set_ch1_motor_speed = get_set_motor1_speed;
                set_ch2_motor_speed = get_set_motor2_speed;
                set_ch3_motor_speed = get_set_motor3_speed;
                motor_trans_mode    = MotorTransMode::Get(get_motor_trans_mode);
                control_mode_       = control_temp;
                xSemaphoreGive(motor_set_speed_mutex);
            }
        }
    }
}

void CanSendTask(void *parameter)
{
    for (;;)
    {
        if (xSemaphoreTake(motor_speed_semaphore, portMAX_DELAY) == pdTRUE)
        {
            can.CanSendMotorSpeed(motor_ch1_speed, motor_ch2_speed, motor_ch3_speed);
        }
    }
}

void CommTask(void *parameter)
{
    uint32_t count = 0;
    for (;;)
    {
        ++count;
        command.run();
        motor3.monitor();

        if (count % 20 == 0)
        {
            //  printf(
            //     "motor speed (1): %0.4f (2): %0.4f  (3): %0.4f "
            //     "motor_set_speed: (1): %0.4f (2): %0.4f  (3): %0.4f "
            //     "trans mode (1): %0.4f (2): %0.4f  (3): %0.4f \n",
            //     motor_ch1_speed,
            //     motor_ch2_speed,
            //     motor_ch3_speed,
            //     set_ch1_motor_speed,
            //     set_ch2_motor_speed,
            //     set_ch3_motor_speed,
            //     motor_trans_mode.motor1_mode,
            //     motor_trans_mode.motor2_mode,
            //     motor_trans_mode.motor3_mode);
        }
        if (can.CanIsOnline() == eOFF_LINE)
        {
            set_ch1_motor_speed = 0;
            set_ch2_motor_speed = 0;
            set_ch3_motor_speed = 0;
        }
        _delay(10);
    }
}

float SetSpeedLimit(float set_motor_speed, float motor_speed)
{
    float temp_set_spped                   = set_motor_speed;
    bool temp_set_spped_is_positive_number = true;
    if (temp_set_spped < 0)
    {
        temp_set_spped_is_positive_number = false;
    }

    if (fabs(motor_speed) > 650 && fabs(motor_speed) < 700)
    {
        temp_set_spped = ((700 - fabs(motor_speed)) * (700 - fabs(motor_speed))) /
                         (100.0f * 100.0f) * temp_set_spped;
        if (temp_set_spped_is_positive_number == false && temp_set_spped > 0)
        {
            temp_set_spped = -temp_set_spped;
        }
    }
    else if (fabs(motor_speed) <= 650)
    {
    }
    else if (fabs(motor_speed) > 700)
    {
        temp_set_spped = 0;
    }
    return temp_set_spped;
}

float set_ch3_motor_speed_temp = 0;
float set_ch2_motor_speed_temp = 0;
float set_ch1_motor_speed_temp = 0;
Cubli::Bsp::MotorTransMode motor_trans_mode_temp; 


void ModeControl(BLDCMotor &motor, uint8_t get_motor_mode, float &control_value)
{
    switch (get_motor_mode)
    {
    case TOUCH_e:
         motor.controller = MotionControlType::torque;
         control_value = Limit(control_value, MOTOR_MAX_TORQUE);
        break;
    case VELOCITY_e:
         motor.controller = MotionControlType::velocity;
         control_value = Limit(control_value, MOTOR_MAX_SPEED);
        break;
    default:
        break;
    }
}

uint8_t control_mode_loop = 0;
void loop()
{
    if (xSemaphoreTake(motor_set_speed_mutex, 3 / portTICK_PERIOD_MS) == pdTRUE)
    {
        set_ch3_motor_speed_temp = set_ch3_motor_speed;
        set_ch2_motor_speed_temp = set_ch2_motor_speed;
        set_ch1_motor_speed_temp = set_ch1_motor_speed;
        motor_trans_mode_temp    = motor_trans_mode;
        // motor_trans_mode_temp.motor1_mode = VELOCITY_e;
        // motor_trans_mode_temp.motor2_mode = VELOCITY_e;
        // motor_trans_mode_temp.motor3_mode = VELOCITY_e;
        control_mode_loop        = control_mode_;
        xSemaphoreGive(motor_set_speed_mutex);
    }

    if(control_mode_loop == 0)
    {
        motor3.voltage_limit  = 3.5f;  // 数字电源：3.0 电池：3.5
        motor3.current_limit  = 3.5f;
    }
    else if(control_mode_loop == 1)
    {
        motor3.voltage_limit  = 4.0f;
        motor3.current_limit  = 4.0f;
    }

    ModeControl(motor1, motor_trans_mode_temp.motor1_mode, set_ch1_motor_speed_temp);
    ModeControl(motor2, motor_trans_mode_temp.motor2_mode, set_ch2_motor_speed_temp);
    ModeControl(motor3, motor_trans_mode_temp.motor3_mode, set_ch3_motor_speed_temp);

    if(motor1.controller == MotionControlType::torque)
    {
        set_ch1_motor_speed_temp = SetSpeedLimit(set_ch1_motor_speed_temp, motor1.shaft_velocity);
    }

    if(motor2.controller == MotionControlType::torque)
    {
        set_ch2_motor_speed_temp = SetSpeedLimit(set_ch2_motor_speed_temp, motor2.shaft_velocity);
    }

    if(motor3.controller == MotionControlType::torque)
    {
        set_ch3_motor_speed_temp = SetSpeedLimit(set_ch3_motor_speed_temp, motor3.shaft_velocity);
    }

    if(motor_trans_mode_temp.motor1_mode == BRAKE_e)
    {
        driver1.setPwm(0, 0, 0);
    }
    else
    {
        motor1.loopFOC();
    }

    motor1.move(set_ch1_motor_speed_temp);
    motor_ch1_speed = motor1.shaft_velocity;

    if(motor_trans_mode_temp.motor2_mode == BRAKE_e)
    {
        driver2.setPwm(0, 0, 0);
    }
    else
    {
        motor2.loopFOC();
    }
    motor2.move(set_ch2_motor_speed_temp);
    motor_ch2_speed = motor2.shaft_velocity;

    if(motor_trans_mode_temp.motor3_mode == BRAKE_e)
    {
        driver3.setPwm(0, 0, 0);
    }
    else
    {
        motor3.loopFOC();
    }
    motor3.move(set_ch3_motor_speed_temp);
    motor_ch3_speed = motor3.shaft_velocity;
    xSemaphoreGive(motor_speed_semaphore);
}
