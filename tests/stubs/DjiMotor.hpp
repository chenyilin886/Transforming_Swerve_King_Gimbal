#pragma once
#include <stdint.h>
namespace BSP::MOTOR::DJI {
class TestFriction {
public:
    int16_t left_current = 0, right_current = 0;
    float getVelocityRpm(int) const { return 0; }
    void ctrl_Current(int id, int16_t value) {
        (id == 1 ? left_current : right_current) = value;
    }
    void sendCAN() {}
};
extern TestFriction *motor_3508;
}
