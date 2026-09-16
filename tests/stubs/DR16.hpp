#pragma once
#include <stdint.h>
namespace BSP::Remote {
class DR16 {
public:
    enum class Switch { UNKNOWN, UP, DOWN, MIDDLE };
    struct Mouse { bool left = false; };
    struct Keyboard { bool r = false; };
    Switch s1 = Switch::UP, s2 = Switch::UP;
    bool offline = false;
    float wheel = 0;
    Mouse mouse;
    Keyboard keyboard;
    static DR16 &Instance() { static DR16 remote; return remote; }
    Switch GetS1() const { return s1; }
    Switch GetS2() const { return s2; }
    bool IsOffline() const { return offline; }
    float GetWheel() const { return wheel; }
    Mouse GetMouse() const { return mouse; }
    Keyboard GetKeyboard() const { return keyboard; }
};
}
