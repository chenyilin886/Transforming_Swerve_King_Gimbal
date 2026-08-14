/**
 * @file VisionComm.cpp
 * @brief RCIA vision protocol over USB CDC virtual COM port.
 */

#include "VisionComm.hpp"
#include "VisionCommC.h"
#include "../Variable.hpp"
#include "BoardComm.hpp"

extern "C" {
#include "usbd_cdc_if.h"
}

namespace VisionComm
{

void Manager::Init()
{
    rx_stream_len_ = 0;
    tx_busy_ = false;
    rx_ = RxData{};
    std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

void Manager::Parse(const uint8_t *data, uint32_t size)
{
    if (data == nullptr || size == 0U)
    {
        return;
    }

    for (uint32_t i = 0; i < size; ++i)
    {
        ConsumeByte(data[i]);
    }
}

void Manager::ConsumeByte(uint8_t byte)
{
    if (rx_stream_len_ == 0U)
    {
        if (byte == RCIA_HEADER)
        {
            rx_buffer_[rx_stream_len_++] = byte;
        }
        return;
    }

    if (rx_stream_len_ == 1U)
    {
        if (byte == RCIA_HEADER)
        {
            rx_buffer_[rx_stream_len_++] = byte;
        }
        else
        {
            rx_stream_len_ = 0;
        }
        return;
    }

    rx_buffer_[rx_stream_len_++] = byte;

    if (rx_stream_len_ >= RCIA_RX_FRAME_SIZE)
    {
        if (ParseFrame())
        {
            rx_stream_len_ = 0;
        }
        else
        {
            ResyncRxStream();
        }
    }
}

bool Manager::ParseFrame()
{
    if (rx_buffer_[0] != RCIA_HEADER || rx_buffer_[1] != RCIA_HEADER)
    {
        return false;
    }

    if (rx_buffer_[12] != RCIA_TAIL)
    {
        return false;
    }

    const int32_t pitch_raw = decode_i32_be(&rx_buffer_[2]);
    const int32_t yaw_raw   = decode_i32_be(&rx_buffer_[6]);

    rx_.pitch_angle  = static_cast<float>(pitch_raw) / ANGLE_SCALE;
    rx_.yaw_angle    = static_cast<float>(yaw_raw) / ANGLE_SCALE;
    rx_.vision_ready = rx_buffer_[10];
    rx_.fire         = rx_buffer_[11];
    rx_.timestamp    = decode_u32_be(&rx_buffer_[13]);
    rx_.aim_x        = rx_buffer_[17];
    rx_.aim_y        = rx_buffer_[18];
    rx_.online       = 1;

    rx_watch_.UpdateLastTime();

    VisionComm_Data.pitch_angle  = rx_.pitch_angle;
    VisionComm_Data.yaw_angle    = rx_.yaw_angle;
    VisionComm_Data.pitch_raw    = pitch_raw;
    VisionComm_Data.yaw_raw      = yaw_raw;
    VisionComm_Data.vision_ready = rx_.vision_ready;
    VisionComm_Data.fire         = rx_.fire;
    VisionComm_Data.timestamp    = rx_.timestamp;
    VisionComm_Data.aim_x        = rx_.aim_x;
    VisionComm_Data.aim_y        = rx_.aim_y;
    VisionComm_Data.online       = rx_.online;
    VisionComm_Data.rx_head0     = rx_buffer_[0];
    VisionComm_Data.rx_head1     = rx_buffer_[1];
    VisionComm_Data.rx_tail      = rx_buffer_[12];

    return true;
}

void Manager::ResyncRxStream()
{
    for (uint8_t i = 1; i + 1 < rx_stream_len_; ++i)
    {
        if (rx_buffer_[i] == RCIA_HEADER && rx_buffer_[i + 1] == RCIA_HEADER)
        {
            const uint8_t remaining = static_cast<uint8_t>(rx_stream_len_ - i);
            std::memmove(rx_buffer_, &rx_buffer_[i], remaining);
            rx_stream_len_ = remaining;
            return;
        }
    }

    if (rx_stream_len_ > 0U && rx_buffer_[rx_stream_len_ - 1U] == RCIA_HEADER)
    {
        rx_buffer_[0] = RCIA_HEADER;
        rx_stream_len_ = 1;
    }
    else
    {
        rx_stream_len_ = 0;
    }
}

void Manager::Send()
{
    if (tx_busy_)
    {
        tx_skip_count_++;
        VisionComm_Data.tx_skip_count = tx_skip_count_;  // 同步到全局变量
        return;
    }

    tx_buffer_[0] = RCIA_HEADER;
    tx_buffer_[1] = RCIA_HEADER;

    encode_f32_be(&tx_buffer_[2],  IMU_Data.quat_w);
    encode_f32_be(&tx_buffer_[6],  IMU_Data.quat_x);
    encode_f32_be(&tx_buffer_[10], IMU_Data.quat_y);
    encode_f32_be(&tx_buffer_[14], IMU_Data.quat_z);

    const float bullet_speed = BoardComm::Gimbal_to_Chassis::Instance().getLaunchSpeed();
    encode_f32_be(&tx_buffer_[18], bullet_speed);

    tx_buffer_[22] = ENEMY_COLOR_BLUE;
    tx_buffer_[23] = VISION_MODE_AUTO_AIM;
    tx_buffer_[24] = RCIA_TAIL;
    encode_u32_be(&tx_buffer_[25], HAL_GetTick());

    tx_busy_ = true;
    VisionComm_Data.tx_busy = 1;  // 同步到全局变量
    
    const uint8_t result = CDC_Transmit_FS(tx_buffer_, RCIA_TX_FRAME_SIZE);
    if (result == USBD_OK)
    {
        tx_count_++;
        VisionComm_Data.tx_count = tx_count_;  // 同步到全局变量
    }
    else
    {
        tx_busy_ = false;
        VisionComm_Data.tx_busy = 0;  // 同步到全局变量
        tx_skip_count_++;
        VisionComm_Data.tx_skip_count = tx_skip_count_;  // 同步到全局变量
    }
}

bool Manager::IsConnected()
{
    rx_watch_.UpdateTime();
    rx_watch_.CheckStatus();
    const bool online = (rx_watch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE);

    if (!online)
    {
        rx_.online = 0;
        VisionComm_Data.online = 0;
    }

    return online;
}

void Manager::TxComplete()
{
    tx_busy_ = false;
    VisionComm_Data.tx_busy = 0;  // 同步到全局变量
}

} // namespace VisionComm

extern "C" void VisionComm_USBReceive(const uint8_t *data, uint32_t len)
{
    VisionComm::Manager::Instance().Parse(data, len);
}

extern "C" void VisionComm_USBTxComplete(void)
{
    VisionComm::Manager::Instance().TxComplete();
}
