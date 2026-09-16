#pragma once
#include <stdint.h>
namespace BSP::MOTOR::LK {
class LK4005 {
public:
    float angle = 10.0f, velocity = 0.0f;
    int16_t torque = 0;
    bool isConnected(int) const { return true; }
    float getVelocityRad(int) const { return velocity; }
    float getAddAngleRad(int) const { return angle; }
    void ctrl_Torque(int, int16_t value) { torque = value; }
};
extern LK4005 *lk4005_motor;
}
