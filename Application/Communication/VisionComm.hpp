/**
 * @file VisionComm.hpp
 * @brief SP + CRC16 vision protocol over USB CDC virtual COM port.
 */

#pragma once

#include "main.h"
#include "state_watch.hpp"
#include <cstdint>
#include <cstring>

namespace VisionComm
{

constexpr uint8_t  SP_HEADER0                   = 0x53; // 'S'
constexpr uint8_t  SP_HEADER1                   = 0x50; // 'P'
constexpr uint8_t  SP_RX_FRAME_SIZE             = 29;   // vision -> gimbal
constexpr uint8_t  SP_TX_FRAME_SIZE             = 43;   // gimbal -> vision
constexpr uint8_t  SP_TX_CRC_OFFSET             = 41;
constexpr uint8_t  SP_RX_CRC_OFFSET             = 27;
constexpr uint8_t  VISION_MODE_IDLE             = 0;
constexpr uint8_t  VISION_MODE_AUTO_AIM        = 1;
constexpr uint8_t  VISION_MODE_SMALL            = 2;
constexpr uint8_t  VISION_MODE_BIG              = 3;
constexpr uint8_t  VISION_CONTROL_IDLE          = 0;
constexpr uint8_t  VISION_CONTROL_AIM           = 1;
constexpr uint8_t  VISION_CONTROL_AIM_AND_FIRE  = 2;
constexpr uint32_t VISION_TIMEOUT_MS            = 500;
constexpr float    RAD_TO_DEG                   = 57.29577951308232f;

inline uint16_t decode_u16_le(const uint8_t *data)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) |
                                 (static_cast<uint16_t>(data[1]) << 8));
}

inline uint32_t decode_u32_le(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

inline float decode_f32_le(const uint8_t *data)
{
    const uint32_t raw = decode_u32_le(data);
    float value;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

inline void encode_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

inline void encode_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
    data[2] = static_cast<uint8_t>((value >> 16) & 0xFFU);
    data[3] = static_cast<uint8_t>((value >> 24) & 0xFFU);
}

inline void encode_f32_le(uint8_t *data, float value)
{
    uint32_t raw;
    std::memcpy(&raw, &value, sizeof(raw));
    encode_u32_le(data, raw);
}

// CRC-16/MCRF4XX: poly 0x1021, reflected implementation 0x8408,
// init 0xFFFF, xorout 0x0000. The returned value is sent low byte first.
inline uint16_t crc16_mcrf4xx(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint32_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit)
        {
            crc = (crc & 1U) != 0U
                ? static_cast<uint16_t>((crc >> 1) ^ 0x8408U)
                : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

struct RxData
{
    uint8_t  mode;
    float    yaw;
    float    yaw_vel;
    float    yaw_acc;
    float    pitch;
    float    pitch_vel;
    float    pitch_acc;

    // Existing control code consumes these compatibility values in degrees.
    float    pitch_angle;
    float    yaw_angle;
    uint8_t  vision_ready;
    uint8_t  fire;
    uint32_t timestamp;
    uint8_t  aim_x;
    uint8_t  aim_y;
    uint8_t  online;
};

class Manager
{
public:
    Manager(const Manager &) = delete;
    Manager &operator=(const Manager &) = delete;

    static Manager &Instance()
    {
        static Manager instance;
        return instance;
    }

    void Init();
    void Parse(const uint8_t *data, uint32_t size);
    void Send();
    bool IsConnected();

    const RxData &GetRxData() const { return rx_; }
    bool IsTxBusy() const { return tx_busy_; }
    uint32_t GetTxCount() const { return tx_count_; }
    uint32_t GetTxSkipCount() const { return tx_skip_count_; }
    void TxComplete();
    void TxAbort();

private:
    Manager()
        : rx_watch_(VISION_TIMEOUT_MS)
        , tx_busy_(false)
        , rx_stream_len_(0)
    {
        std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
        std::memset(tx_buffer_, 0, sizeof(tx_buffer_));
        rx_ = RxData{};
    }

    void ConsumeByte(uint8_t byte);
    bool ParseFrame();
    void ResyncRxStream();

    uint8_t rx_buffer_[SP_RX_FRAME_SIZE];
    uint8_t tx_buffer_[SP_TX_FRAME_SIZE];
    RxData rx_;

    BSP::WATCH_STATE::StateWatch rx_watch_;
    volatile bool tx_busy_;
    uint32_t tx_count_ = 0;
    uint32_t tx_skip_count_ = 0;
    uint32_t tx_complete_count_ = 0;
    uint32_t tx_abort_count_ = 0;
    uint8_t rx_stream_len_;
};

} // namespace VisionComm
