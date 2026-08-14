/**
 * @file VisionComm.hpp
 * @brief RCIA vision protocol over USB CDC virtual COM port.
 */

#pragma once

#include "main.h"
#include "state_watch.hpp"
#include <cstdint>
#include <cstring>

namespace VisionComm
{

constexpr uint8_t  RCIA_HEADER        = 0x39;
constexpr uint8_t  RCIA_TAIL          = 0xFF;
constexpr uint8_t  RCIA_RX_DATA_SIZE  = 17;
constexpr uint8_t  RCIA_TX_DATA_SIZE  = 27;
constexpr uint8_t  RCIA_RX_FRAME_SIZE = 2 + RCIA_RX_DATA_SIZE;
constexpr uint8_t  RCIA_TX_FRAME_SIZE = 2 + RCIA_TX_DATA_SIZE;
constexpr uint8_t  ENEMY_COLOR_BLUE   = 0x52;
constexpr uint8_t  VISION_MODE_IDLE   = 0;
constexpr uint8_t  VISION_MODE_AUTO_AIM = 1;
constexpr uint8_t  VISION_MODE_SMALL  = 2;
constexpr uint8_t  VISION_MODE_BIG    = 3;
constexpr float    ANGLE_SCALE        = 100.0f;
constexpr uint32_t VISION_TIMEOUT_MS  = 500;

inline int32_t decode_i32_be(const uint8_t *data)
{
    const uint32_t value = (static_cast<uint32_t>(data[0]) << 24) |
                           (static_cast<uint32_t>(data[1]) << 16) |
                           (static_cast<uint32_t>(data[2]) << 8) |
                           static_cast<uint32_t>(data[3]);
    return static_cast<int32_t>(value);
}

inline uint32_t decode_u32_be(const uint8_t *data)
{
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

inline void encode_u32_be(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(value & 0xFF);
}

inline void encode_f32_be(uint8_t *data, float value)
{
    uint32_t raw;
    std::memcpy(&raw, &value, sizeof(raw));
    encode_u32_be(data, raw);
}

struct RxData
{
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

    uint8_t rx_buffer_[RCIA_RX_FRAME_SIZE];
    uint8_t tx_buffer_[RCIA_TX_FRAME_SIZE];
    RxData rx_;

    BSP::WATCH_STATE::StateWatch rx_watch_;
    volatile bool tx_busy_;
    uint32_t tx_count_ = 0;
    uint32_t tx_skip_count_ = 0;
    uint8_t rx_stream_len_;
};

} // namespace VisionComm
