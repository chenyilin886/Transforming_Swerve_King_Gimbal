/**
 * @file VisionComm.cpp
 * @brief SP + CRC16 vision protocol over USB CDC virtual COM port.
 */

#include "VisionComm.hpp"
#include "VisionCommC.h"
#include "../Variable.hpp"
#include "BoardComm.hpp"
#include <cmath>

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
        if (byte == SP_HEADER0)
        {
            rx_buffer_[rx_stream_len_++] = byte;
        }
        return;
    }

    if (rx_stream_len_ == 1U)
    {
        if (byte == SP_HEADER1)
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

    if (rx_stream_len_ >= SP_RX_FRAME_SIZE)
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
    if (rx_buffer_[0] != SP_HEADER0 || rx_buffer_[1] != SP_HEADER1)
    {
        return false;
    }

    const uint16_t received_crc = decode_u16_le(&rx_buffer_[SP_RX_CRC_OFFSET]);
    const uint16_t calculated_crc = crc16_mcrf4xx(rx_buffer_, SP_RX_CRC_OFFSET);
    if (received_crc != calculated_crc)
    {
        return false;
    }

    const uint8_t mode = rx_buffer_[2];
    if (mode > VISION_CONTROL_AIM_AND_FIRE)
    {
        return false;
    }

    const float yaw = decode_f32_le(&rx_buffer_[3]);
    const float yaw_vel = decode_f32_le(&rx_buffer_[7]);
    const float yaw_acc = decode_f32_le(&rx_buffer_[11]);
    const float pitch = decode_f32_le(&rx_buffer_[15]);
    const float pitch_vel = decode_f32_le(&rx_buffer_[19]);
    const float pitch_acc = decode_f32_le(&rx_buffer_[23]);
    if (!std::isfinite(yaw) || !std::isfinite(yaw_vel) ||
        !std::isfinite(yaw_acc) || !std::isfinite(pitch) ||
        !std::isfinite(pitch_vel) || !std::isfinite(pitch_acc))
    {
        return false;
    }

    rx_.mode = mode;
    rx_.yaw = yaw;
    rx_.yaw_vel = yaw_vel;
    rx_.yaw_acc = yaw_acc;
    rx_.pitch = pitch;
    rx_.pitch_vel = pitch_vel;
    rx_.pitch_acc = pitch_acc;

    // Keep the existing control interface in degrees while the wire protocol
    // uses radians.
    rx_.pitch_angle = pitch * RAD_TO_DEG;
    rx_.yaw_angle = yaw * RAD_TO_DEG;
    rx_.vision_ready = (mode != VISION_CONTROL_IDLE) ? 1U : 0U;
    rx_.fire = (mode == VISION_CONTROL_AIM_AND_FIRE) ? 1U : 0U;
    rx_.timestamp = HAL_GetTick();
    rx_.aim_x = 0;
    rx_.aim_y = 0;
    rx_.online       = 1;

    rx_watch_.UpdateLastTime();

    VisionComm_Data.pitch_angle  = rx_.pitch_angle;
    VisionComm_Data.yaw_angle    = rx_.yaw_angle;
    VisionComm_Data.yaw_velocity = rx_.yaw_vel;
    VisionComm_Data.yaw_acceleration = rx_.yaw_acc;
    VisionComm_Data.pitch_velocity = rx_.pitch_vel;
    VisionComm_Data.pitch_acceleration = rx_.pitch_acc;
    VisionComm_Data.pitch_raw    = static_cast<int32_t>(rx_.pitch_angle * 100.0f);
    VisionComm_Data.yaw_raw      = static_cast<int32_t>(rx_.yaw_angle * 100.0f);
    VisionComm_Data.vision_ready = rx_.vision_ready;
    VisionComm_Data.fire         = rx_.fire;
    VisionComm_Data.timestamp    = rx_.timestamp;
    VisionComm_Data.aim_x        = rx_.aim_x;
    VisionComm_Data.aim_y        = rx_.aim_y;
    VisionComm_Data.online       = rx_.online;
    VisionComm_Data.rx_head0     = rx_buffer_[0];
    VisionComm_Data.rx_head1     = rx_buffer_[1];
    VisionComm_Data.rx_tail      = 0;

    return true;
}

void Manager::ResyncRxStream()
{
    for (uint8_t i = 1; i + 1 < rx_stream_len_; ++i)
    {
        if (rx_buffer_[i] == SP_HEADER0 && rx_buffer_[i + 1] == SP_HEADER1)
        {
            const uint8_t remaining = static_cast<uint8_t>(rx_stream_len_ - i);
            std::memmove(rx_buffer_, &rx_buffer_[i], remaining);
            rx_stream_len_ = remaining;
            return;
        }
    }

    if (rx_stream_len_ > 0U && rx_buffer_[rx_stream_len_ - 1U] == SP_HEADER0)
    {
        rx_buffer_[0] = SP_HEADER0;
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

    tx_buffer_[0] = SP_HEADER0;
    tx_buffer_[1] = SP_HEADER1;

    // Work mode is kept at AUTO_AIM during the first protocol migration.
    tx_buffer_[2] = VISION_MODE_AUTO_AIM;

    // The vision side waits for a valid quaternion during startup. Use the
    // identity quaternion until IMU data is online instead of sending zeros.
    float quat_w = IMU_Data.quat_w;
    float quat_x = IMU_Data.quat_x;
    float quat_y = IMU_Data.quat_y;
    float quat_z = IMU_Data.quat_z;
    const float quat_norm_sq = quat_w * quat_w + quat_x * quat_x +
                               quat_y * quat_y + quat_z * quat_z;
    if (IMU_Data.online == 0U || !std::isfinite(quat_w) ||
        !std::isfinite(quat_x) || !std::isfinite(quat_y) ||
        !std::isfinite(quat_z) || quat_norm_sq < 0.5f || quat_norm_sq > 1.5f)
    {
        quat_w = 1.0f;
        quat_x = 0.0f;
        quat_y = 0.0f;
        quat_z = 0.0f;
    }

    encode_f32_le(&tx_buffer_[3],  quat_w);
    encode_f32_le(&tx_buffer_[7],  quat_x);
    encode_f32_le(&tx_buffer_[11], quat_y);
    encode_f32_le(&tx_buffer_[15], quat_z);

    encode_f32_le(&tx_buffer_[19], Joint_Data.yaw.real_angle);
    encode_f32_le(&tx_buffer_[23], Joint_Data.yaw.velocity);
    encode_f32_le(&tx_buffer_[27], Joint_Data.pitch.real_angle);
    encode_f32_le(&tx_buffer_[31], Joint_Data.pitch.velocity);

    const float bullet_speed = BoardComm::Gimbal_to_Chassis::Instance().getLaunchSpeed();
    encode_f32_le(&tx_buffer_[35], bullet_speed);

    // The current counter is a planned shot-command counter, not a referee
    // confirmed projectile count. Send zero until that semantic is finalized.
    encode_u16_le(&tx_buffer_[39], 0U);

    const uint16_t crc = crc16_mcrf4xx(tx_buffer_, SP_TX_CRC_OFFSET);
    encode_u16_le(&tx_buffer_[SP_TX_CRC_OFFSET], crc);

    tx_busy_ = true;
    VisionComm_Data.tx_busy = 1;  // 同步到全局变量
    
    const uint8_t result = CDC_Transmit_FS(tx_buffer_, SP_TX_FRAME_SIZE);
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
    tx_complete_count_++;
    VisionComm_Data.tx_complete_count = tx_complete_count_;
}

void Manager::TxAbort()
{
    if (tx_busy_)
    {
        tx_abort_count_++;
        VisionComm_Data.tx_abort_count = tx_abort_count_;
    }

    // USB reset/disconnect cancels the in-flight transfer without a TxCplt callback.
    tx_busy_ = false;
    VisionComm_Data.tx_busy = 0;
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

extern "C" void VisionComm_USBTxAbort(void)
{
    VisionComm::Manager::Instance().TxAbort();
}
