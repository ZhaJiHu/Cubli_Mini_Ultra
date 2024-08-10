#include "bsp/can.h"

CAN_device_t CAN_cfg;

namespace Cubli {
namespace Bsp {

void CanDriver::Init(int _can_txd_pin, int _can_rxd_pin, CAN_speed_t _can_speed)
{
    CAN_cfg.tx_pin_id = (gpio_num_t)_can_txd_pin;
    CAN_cfg.rx_pin_id = (gpio_num_t)_can_rxd_pin;
    CAN_cfg.speed     = _can_speed;
    CAN_cfg.rx_queue  = xQueueCreate(100, sizeof(CAN_frame_t));
    ESP32Can.CANInit();
}

void CanDriver::SendMessage(uint32_t msg_id, uint8_t message[8])
{
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF  = CAN_frame_std;
    tx_frame.FIR.B.RTR = CAN_no_RTR;
    tx_frame.MsgID     = msg_id;
    tx_frame.FIR.B.DLC = 8;
    memcpy(tx_frame.data.u8, message, sizeof(tx_frame.data.u8));
    ESP32Can.CANWriteFrame(&tx_frame);
}

void CanDriver::CanSendMotorSpeed(
    float _send_ch1_speed, float _send_ch2_speed, float _send_ch3_speed)
{
    uint8_t message[8];
    message[0] = (int16_t)(_send_ch1_speed * 40.0f) & 0xff;
    message[1] = (int16_t)(_send_ch1_speed * 40.0f) >> 8;
    message[2] = (int16_t)(_send_ch2_speed * 40.0f) & 0xff;
    message[3] = (int16_t)(_send_ch2_speed * 40.0f) >> 8;
    message[4] = (int16_t)(_send_ch3_speed * 40.0f) & 0xff;
    message[5] = (int16_t)(_send_ch3_speed * 40.0f) >> 8;
    message[6] = 0;
    message[7] = 0;
    SendMessage(eCAN_SEND_MOTOR_SPEED_FRAME, message);
}

bool CanDriver::CanGetMotorSpeed(
    float &_set_ch1_speed, float &_set_ch2_speed, float &_set_ch3_speed, uint8_t& _motor_mode, uint8_t &control)
{
    CAN_frame_t rx_frame;
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, portMAX_DELAY) == pdTRUE)
    {
        // 标准帧和数据帧
        if (rx_frame.FIR.B.FF == CAN_frame_std && rx_frame.FIR.B.RTR == CAN_no_RTR)
        {
            if (rx_frame.MsgID == eCAN_GET_MOTOR_SPEED_FRAME)
            {
                _set_ch1_speed  = (int16_t)(rx_frame.data.u8[0] | rx_frame.data.u8[1] << 8) / 40.0f;
                _set_ch2_speed  = (int16_t)(rx_frame.data.u8[2] | rx_frame.data.u8[3] << 8) / 40.0f;
                _set_ch3_speed  = (int16_t)(rx_frame.data.u8[4] | rx_frame.data.u8[5] << 8) / 40.0f;
                _motor_mode = rx_frame.data.u8[6];
                control = rx_frame.data.u8[7];
                // rx_frame_count_ = 0;
                can_is_online_  = eON_LINE;
                lost_can_time_.reset();
                return true;
            }
        }
    }
    return false;
}

CanStatus_e CanDriver::CanIsOnline()
{
    // rx_frame_count_++;
    // if (rx_frame_count_ > CAN_OFFLINE_COUNT)
    // {
    //     can_is_online_ = eOFF_LINE;
    // }
    if(lost_can_time_.GetTimeS() > 0.5f)
    {
        can_is_online_ = eOFF_LINE;
    }
    return can_is_online_;
}

}  // namespace Bsp
}  // namespace Cubli