#pragma once
#include <stdint.h>
#include <cmath>
extern "C" uint32_t HAL_GetTick(void);
namespace BSP::MOTOR::LK {
struct LKA1Position {
    float output_phase_deg = 0, accumulated_rad = 10, velocity_rad = 0;
    uint32_t received_ms = 0;
    bool valid = false;
};
class LK4005 {
public:
    float angle = 10, velocity = 0, phase_offset_deg = 0;
    int16_t torque = 0;
    bool online = true, respond = true;
    LKA1Position snapshot{};
    bool isConnected(int) const { return online; }
    float getVelocityRad(int) const { return velocity; }
    float getAddAngleRad(int) const { return angle; }
    void ctrl_Torque(int, int16_t value) { torque = value; }
    LKA1Position getA1Position(int) {
        if (respond) {
            snapshot.output_phase_deg = std::fmod(phase_offset_deg + (angle-10)*180/3.14159265358979323846f,36.0f);
            if (snapshot.output_phase_deg < 0) snapshot.output_phase_deg += 36;
            snapshot.accumulated_rad = angle;
            snapshot.velocity_rad = velocity;
            snapshot.received_ms = HAL_GetTick();
            snapshot.valid = true;
        }
        return snapshot;
    }
    // Intentionally no 0x92/0x94 APIs: production feeder must compile using A1 only.
};
extern LK4005 *lk4005_motor;
}
