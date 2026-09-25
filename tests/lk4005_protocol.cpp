// Real LK driver, real MotorBase/StateWatch and CAN interface, fake bus/time.
#include "../BSP/Motor/Lk/LkMotor.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

static uint32_t tick = 1000;
extern "C" uint32_t HAL_GetTick(void) { return tick; }
class Bus : public HAL::CAN::ICanDevice {
public:
    HAL::CAN::Frame sent{};
    void init() override {}
    void start() override {}
    bool send(const HAL::CAN::Frame &f) override { sent = f; return true; }
    bool receive(HAL::CAN::Frame &) override { return false; }
    CAN_HandleTypeDef *get_handle() const override { return nullptr; }
    void register_rx_callback(HAL::CAN::RxCallback) override {}
    void trigger_rx_callbacks(const HAL::CAN::Frame &) override {}
};
static HAL::CAN::Frame circle_frame(uint32_t raw) {
    HAL::CAN::Frame f{};
    f.id = 0x141; f.dlc = 8; f.data[0] = 0x94;
    for (int i=0; i<4; ++i) f.data[4+i] = uint8_t(raw >> (8*i));
    return f;
}
static HAL::CAN::Frame multi_frame(int64_t raw) {
    HAL::CAN::Frame f{};
    f.id = 0x141; f.dlc = 8; f.data[0] = 0x92;
    const uint64_t bits = uint64_t(raw);
    for (int i=0; i<7; ++i) f.data[1+i] = uint8_t(bits >> (8*i));
    return f;
}
static void multi_turn_diagnostics() {
    Bus bus;
    BSP::MOTOR::LK::LK4005 motor(&bus);
    assert(!motor.getMultiTurnPosition(1).valid);
    assert(!motor.ReadMultiTurnPosition(0) && !motor.ReadMultiTurnPosition(2));
    assert(motor.ReadMultiTurnPosition(1));
    assert(bus.sent.id == 0x141 && bus.sent.dlc == 8 && bus.sent.data[0] == 0x92);
    for (int i=1; i<8; ++i) assert(bus.sent.data[i] == 0);
    const int64_t values[] = {0, 1, -1, 36000, -36000, 123456789, -123456789,
        (int64_t(1) << 55) - 1, -(int64_t(1) << 55)};
    for (int64_t value : values) {
        motor.Parse(multi_frame(value));
        const auto sample = motor.getMultiTurnPosition(1);
        assert(sample.valid && sample.raw == value && sample.received_ms == tick);
        assert(!sample.accumulated_valid);
        assert(sample.output_deg == double(value) / 1000.0);
        assert(!motor.isConnected(1)); // A query is never motion feedback.
    }
    motor.Parse(multi_frame(1234));
    auto malformed = multi_frame(9999);
    malformed.dlc = 7; motor.Parse(malformed);
    assert(motor.getMultiTurnPosition(1).raw == 1234);
    malformed.dlc = 8; malformed.id = 0x142; motor.Parse(malformed);
    assert(motor.getMultiTurnPosition(1).raw == 1234);
    malformed.id = 0x141; malformed.is_extended_id = true; motor.Parse(malformed);
    assert(motor.getMultiTurnPosition(1).raw == 1234);
    malformed.is_extended_id = false; malformed.is_remote_frame = true; motor.Parse(malformed);
    assert(motor.getMultiTurnPosition(1).raw == 1234);
    test_irq_mask = 1;
    motor.getMultiTurnPosition(1);
    assert(test_irq_mask == 1);
    test_irq_mask = 0;
    assert(!motor.getMultiTurnPosition(0).valid && !motor.getMultiTurnPosition(2).valid);
}
int main() {
    multi_turn_diagnostics();
    Bus bus;
    BSP::MOTOR::LK::LK4005 motor(&bus);
    assert(!motor.isConnected(1));
    assert(!motor.getA1Position(1).valid);
    assert(!motor.getA1Position(0).valid && !motor.getA1Position(2).valid);
    assert(motor.ReadSingleCircle(1));
    assert(bus.sent.id == 0x141 && bus.sent.dlc == 8 && bus.sent.data[0] == 0x94);
    for (int i=1;i<8;++i) assert(bus.sent.data[i] == 0);
    assert(motor.getSingleCirclePeriodDeg() == 36.0f);
    motor.Parse(circle_frame(12345));
    auto s = motor.getSingleCircle(1);
    assert(s.valid && !s.accumulated_valid && !motor.isConnected(1));
    assert(std::fabs(s.output_deg-12.345f) < 0.0001f);
    HAL::CAN::Frame a1{};
    a1.id = 0x141; a1.dlc = 8; a1.data[0] = 0xA1;
    a1.data[1] = 35; a1.data[2] = 100; a1.data[4] = 20;
    a1.data[6] = 0xf0; a1.data[7] = 0xff;
    motor.Parse(a1);
    ++tick;
    a1.data[6] = 0x10; a1.data[7] = 0;
    motor.Parse(a1); // Rotor wraps; output accumulation must stay small positive.
    const float accumulated = motor.getAddAngleRad(1);
    assert(accumulated > 0 && accumulated < 0.001f);
    const auto a1_sample = motor.getA1Position(1);
    assert(a1_sample.valid && a1_sample.received_ms == tick);
    assert(a1_sample.accumulated_rad == accumulated);
    assert(std::fabs(a1_sample.output_phase_deg - 16.0f*36.0f/65536.0f) < 0.000001f);
    assert(a1_sample.velocity_rad == motor.getVelocityRad(1));
    test_irq_mask = 1;
    motor.getA1Position(1);
    assert(test_irq_mask == 1);
    test_irq_mask = 0;
    motor.Parse(circle_frame(35999));
    s = motor.getSingleCircle(1);
    assert(s.valid && s.accumulated_valid && s.raw == 35999);
    assert(std::fabs(s.output_deg-35.999f) < 0.0001f);
    assert(s.accumulated_rad == accumulated && s.received_ms == tick);
    assert(motor.getFeedback(1).cmd == 0xA1 && motor.getFeedback(1).angle == 16);
    assert(motor.getTemperature(1) == 35 && motor.getAddAngleRad(1) == accumulated);
    ++tick;
    motor.Parse(multi_frame(-36000));
    assert(motor.getA1Position(1).received_ms == a1_sample.received_ms);
    assert(motor.getA1Position(1).output_phase_deg == a1_sample.output_phase_deg);
    assert(motor.getMultiTurnPosition(1).output_deg == -36.0);
    assert(motor.getMultiTurnPosition(1).accumulated_valid);
    assert(motor.getMultiTurnPosition(1).accumulated_rad == accumulated);
    const float paired_speed = motor.getMultiTurnPosition(1).velocity_rad;
    assert(paired_speed == motor.getVelocityRad(1) && paired_speed > 0);
    a1.data[4] = 0;
    motor.Parse(a1);
    assert(motor.getVelocityRad(1) == 0);
    assert(motor.getMultiTurnPosition(1).velocity_rad == paired_speed); // Snapshot, not live speed.
    assert(motor.getFeedback(1).cmd == 0xA1 && motor.getFeedback(1).angle == 16);
    assert(motor.getAddAngleRad(1) == accumulated && motor.getSingleCircle(1).raw == 35999);
    test_irq_mask = 1;
    motor.getSingleCircle(1);
    assert(test_irq_mask == 1); // Preserve caller interrupt state.
    test_irq_mask = 0;
    motor.Parse(circle_frame(36000));
    assert(!motor.getSingleCircle(1).valid);
    motor.Parse(circle_frame(0));
    assert(motor.getSingleCircle(1).valid && motor.getSingleCircle(1).output_deg == 0);
    auto malformed = circle_frame(1000);
    malformed.dlc = 7;
    motor.Parse(malformed);
    assert(motor.getSingleCircle(1).raw == 0);
    malformed.dlc = 8; malformed.id = 0x142;
    motor.Parse(malformed);
    assert(motor.getSingleCircle(1).raw == 0);
    malformed.id = 0x141; malformed.is_remote_frame = true;
    motor.Parse(malformed);
    assert(motor.getSingleCircle(1).raw == 0);
    // Enable acknowledgement must not zero or otherwise corrupt rotor feedback.
    HAL::CAN::Frame ack{}; ack.id=0x141; ack.dlc=8; ack.data[0]=0x88;
    motor.Parse(ack);
    assert(motor.getFeedback(1).angle == 16 && motor.getAddAngleRad(1) == accumulated);
    tick += 201;
    motor.Parse(multi_frame(72000));
    assert(!motor.isConnected(1));
    assert(!motor.getMultiTurnPosition(1).accumulated_valid);
    motor.Parse(circle_frame(1000));
    assert(!motor.isConnected(1)); // A query cannot disguise a dead motion stream.
    assert(!motor.getSingleCircle(1).accumulated_valid);
    assert(motor.getAddAngleRad(1) == accumulated);
    std::puts("PASS: real LK4005 0x92/0x94/A1 decoding, bounds, snapshots and online isolation");
}
