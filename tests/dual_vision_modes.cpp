// Host regression tests execute the production ShootFSM, DialController and PID.
// Only DR16, HAL time and motor IO are replaced with deterministic doubles.
#include "ShootFSM.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

Dial_Config_t Dial_Config{}, Dial_Config_UpUp{}, Dial_Config_UpDown{};
Dial_Status_t Dial_Status{};
Shoot_Config_t Shoot_Config{};
Shoot_Status_t Shoot_Status{};
Friction_Data_t Friction_Data{};
VisionComm_Data_t VisionComm_Data{};
Remote_State_t Remote_State{};
KeyboardMouse_Control_t KeyboardMouse_Control{};

static uint32_t tick = 100;
extern "C" uint32_t HAL_GetTick(void) { return tick; }
static BSP::MOTOR::LK::LK4005 dial_motor;
static BSP::MOTOR::DJI::TestFriction friction_motor;
namespace BSP::MOTOR::LK { LK4005 *lk4005_motor = &dial_motor; }
namespace BSP::MOTOR::DJI { TestFriction *motor_3508 = &friction_motor; }
using Switch = BSP::Remote::DR16::Switch;
using DialState = BSP::CTRL::DialState;

static bool near(float a, float b) { return std::fabs(a-b) < 0.0001f; }
static Dial_Config_t config() {
    Dial_Config_t c{};
    c.enabled = c.feature_enable = 1;
    c.slots_per_rotation = 9;
    c.angle_per_shot_deg = 40;
    c.wheel_start_threshold = 0.5f;
    c.long_press_ms = 20;
    c.auto_fire_hz = 5;
    c.pos_kp = c.vel_kp = 1;
    c.pos_vel_limit = 80;
    c.raw_output_limit = 2048;
    return c;
}
struct Fixture {
    BSP::FSM::Class_ShootFSM fsm;
    BSP::Remote::DR16 &rc = BSP::Remote::DR16::Instance();
    Fixture() {
        rc = BSP::Remote::DR16{};
        dial_motor = BSP::MOTOR::LK::LK4005{};
        friction_motor = BSP::MOTOR::DJI::TestFriction{};
        Dial_Config = Dial_Config_UpUp = Dial_Config_UpDown = config();
        Dial_Status = {};
        Shoot_Status = {};
        Shoot_Config = {};
        Shoot_Config.feature_enable = Shoot_Config.shoot_enabled = 1;
        Shoot_Config.friction_target_rpm = 6000;
        Shoot_Config.friction_kp = 1;
        VisionComm_Data = {};
        Friction_Data = {};
        Friction_Data.left.online = Friction_Data.right.online = 1;
        KeyboardMouse_Control = {};
        Remote_State = {};
    }
    void step(uint32_t ms = 4) { tick += ms; fsm.Control(); }
};

static void vision_priority_and_friction(Switch s2) {
    Fixture f;
    f.rc.s2 = s2;
    VisionComm_Data.online = 1;
    VisionComm_Data.vision_ready = 0; // Does NOT gate feeder ownership.
    f.rc.wheel = 1;
    f.step();
    assert(Shoot_Status.dial_config_mode == (s2 == Switch::UP ? 1 : 2));
    assert(Shoot_Status.friction_enable && friction_motor.left_current > 0 && friction_motor.right_current < 0);
    assert(Shoot_Status.vision_control && !Shoot_Status.trigger_source);
    assert(Dial_Status.shot_count == 0); // fire=0 blocks the held wheel.
    VisionComm_Data.fire = 1;
    f.step();
    assert(Shoot_Status.trigger_source == 2 && Dial_Status.shot_count == 1);
    VisionComm_Data.online = 0; // Held wheel must not fire on takeover.
    f.step();
    assert(!Shoot_Status.vision_control && Shoot_Status.waiting_release);
    assert(Shoot_Status.trigger_source == 0 && Dial_Status.shot_count == 1);
    f.rc.wheel = 0;
    f.step();
    f.rc.wheel = 1;
    f.step();
    assert(Shoot_Status.trigger_source == 1 && Dial_Status.shot_count == 2);
    VisionComm_Data.online = 1; // fire still high; release required again.
    f.step();
    assert(Shoot_Status.waiting_release && Dial_Status.shot_count == 2);
    VisionComm_Data.fire = 0;
    f.step();
    VisionComm_Data.fire = 1;
    f.step();
    assert(Dial_Status.shot_count == 3);
    // Online is the criterion; loss of either friction feedback releases ownership.
    Friction_Data.left.online = 0;
    f.step();
    assert(!Shoot_Status.vision_control && Shoot_Status.waiting_release);
    Friction_Data.left.online = 1;
    Friction_Data.right.online = 0;
    f.step();
    assert(!Shoot_Status.vision_control);
}

static void config_switch_and_single_completion() {
    Fixture f;
    f.step();
    f.rc.wheel = 1;
    f.step();
    float committed = Dial_Status.target_angle;
    assert(committed > dial_motor.angle);
    f.rc.s2 = Switch::DOWN;
    f.step();
    assert(Shoot_Status.dial_config_mode == 2 && Shoot_Status.waiting_release);
    assert(Dial_Status.shot_count == 1 && near(Dial_Status.target_angle, committed));
    f.step(100);
    assert(Dial_Status.shot_count == 1 && f.fsm.dial_ctrl.state == DialState::STOP);
    f.rc.wheel = 0;
    f.step();
    f.rc.wheel = 1;
    f.step();
    assert(Dial_Status.shot_count == 2);
    // Only the selected config consumes clear_pid and receives FSM enable.
    Dial_Config_UpUp.clear_pid = Dial_Config_UpDown.clear_pid = 1;
    Shoot_Config.shoot_enabled = 0;
    f.step();
    assert(!Dial_Config_UpDown.enabled && Dial_Config_UpUp.enabled);
    assert(!Dial_Config_UpDown.clear_pid && Dial_Config_UpUp.clear_pid);
}

static void independent_vision_timing_and_rate() {
    Fixture f;
    Dial_Config_UpUp.long_press_ms = 8;
    Dial_Config_UpDown.long_press_ms = 40;
    Dial_Config_UpDown.auto_fire_hz = 10;
    Dial_Config_UpDown.wheel_to_hz = 99; // Ignored by vision.
    VisionComm_Data.online = 1;
    f.step();
    VisionComm_Data.fire = 1;
    f.step();
    f.step(8);
    assert(f.fsm.dial_ctrl.state == DialState::AUTO);
    float before = Dial_Status.target_angle;
    f.step(20);
    assert(near(Dial_Status.target_angle-before, 5*40*(PI/180)*0.020f));
    f.rc.s2 = Switch::DOWN;
    f.step();
    assert(near(Dial_Status.target_angle, dial_motor.angle)); // Stop AUTO backlog.
    assert(Shoot_Status.waiting_release && Dial_Status.shot_count == 1);
    VisionComm_Data.fire = 0;
    f.step();
    VisionComm_Data.fire = 1;
    f.step();
    f.step(20);
    assert(f.fsm.dial_ctrl.state == DialState::SINGLE);
    f.step(20);
    assert(f.fsm.dial_ctrl.state == DialState::AUTO);
    before = Dial_Status.target_angle;
    f.step(20);
    assert(near(Dial_Status.target_angle-before, 10*40*(PI/180)*0.020f));
}

static void manual_threshold_rate_and_middle() {
    Fixture f;
    f.rc.s2 = Switch::DOWN;
    Dial_Config_UpDown.wheel_start_threshold = 0.8f;
    Dial_Config_UpDown.wheel_to_hz = 12;
    f.step();
    f.rc.wheel = 0.7f;
    f.step();
    assert(Dial_Status.shot_count == 0);
    f.rc.wheel = 1;
    f.step();
    f.step(20);
    float before = Dial_Status.target_angle;
    f.step(20);
    assert(near(Dial_Status.target_angle-before, 12*40*(PI/180)*0.020f));
    f.rc.s2 = Switch::MIDDLE;
    f.step();
    assert(Shoot_Status.dial_config_mode == 0 && !Shoot_Status.friction_enable);
    assert(Shoot_Status.waiting_release && near(Dial_Status.target_angle, dial_motor.angle));
    f.rc.wheel = 0;
    f.step();
    f.rc.mouse.left = true;
    f.step();
    assert(Shoot_Status.trigger_source == 3);
}

static void disable_and_estop_override_fire() {
    Fixture f;
    VisionComm_Data.online = 1;
    f.step();
    VisionComm_Data.fire = 1;
    f.step();
    Shoot_Config.shoot_enabled = 0;
    f.step();
    assert(f.fsm.dial_ctrl.state == DialState::DISABLE && dial_motor.torque == 0);
    Shoot_Config.shoot_enabled = 1;
    f.step();
    assert(Shoot_Status.waiting_release && Dial_Status.shot_count == 1);
    f.rc.offline = true; // Even if switch values are stale and fire is high.
    f.step();
    assert(dial_motor.torque == 0 && !Shoot_Status.friction_enable);
    assert(friction_motor.left_current == 0 && friction_motor.right_current == 0);
    f.rc.offline = false;
    f.rc.s1 = f.rc.s2 = Switch::DOWN;
    f.step();
    assert(dial_motor.torque == 0 && !Shoot_Status.friction_enable);
    f.rc.s1 = f.rc.s2 = Switch::UP;
    Dial_Config_UpUp.feature_enable = 0;
    f.step();
    assert(f.fsm.dial_ctrl.state == DialState::DISABLE && dial_motor.torque == 0);
}

int main() {
    vision_priority_and_friction(Switch::UP);
    vision_priority_and_friction(Switch::DOWN);
    config_switch_and_single_completion();
    independent_vision_timing_and_rate();
    manual_threshold_rate_and_middle();
    disable_and_estop_override_fire();
    std::puts("PASS: 6 dual-vision feeder/friction scenarios");
}
