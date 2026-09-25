// Host regression tests execute the production ShootFSM, DialController and PID.
// Only DR16, HAL time and motor IO are replaced with deterministic doubles.
#include "ShootFSM.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>

Dial_Config_t Dial_Config{}, Dial_Config_UpUp{}, Dial_Config_UpDown{};
Dial_Status_t Dial_Status{};
Dial_PID_Config_t Dial_PID_Config{};
Dial_Calibration_t Dial_Calibration{};
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
    return c;
}
struct Fixture {
    BSP::FSM::Class_ShootFSM fsm;
    BSP::Remote::DR16 &rc = BSP::Remote::DR16::Instance();
    Fixture(bool ready = true) {
        rc = BSP::Remote::DR16{};
        dial_motor = BSP::MOTOR::LK::LK4005{};
        friction_motor = BSP::MOTOR::DJI::TestFriction{};
        Dial_Config = Dial_Config_UpUp = Dial_Config_UpDown = config();
        Dial_Config_UpDown.fire_mode = DialFireMode::DIRECT_AUTO;
        Dial_PID_Config = {};
        Dial_PID_Config.pos_kp = Dial_PID_Config.vel_kp = 1;
        Dial_PID_Config.pos_vel_limit = 80;
        Dial_PID_Config.raw_output_limit = 2048;
        Dial_Status = {};
        Dial_Calibration = {0, 1, 2, 300, 1000, 100};
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
        if (ready) {
            step();
            step(100);
            assert(Dial_Status.home_state == 3);
        }
    }
    void step(uint32_t ms = 4) { tick += ms; fsm.Control(); }
};

static void vision_priority_and_friction(Switch s2) {
    Fixture f;
    const uint32_t single_count = s2 == Switch::UP ? 1U : 0U;
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
    assert(Shoot_Status.trigger_source == 2 && Dial_Status.shot_count == 1 * single_count);
    VisionComm_Data.online = 0; // Held wheel must not fire on takeover.
    f.step();
    assert(!Shoot_Status.vision_control && Shoot_Status.waiting_release);
    assert(Shoot_Status.trigger_source == 0 && Dial_Status.shot_count == 1 * single_count);
    f.rc.wheel = 0;
    f.step();
    f.rc.wheel = 1;
    f.step();
    assert(Shoot_Status.trigger_source == 1 && Dial_Status.shot_count == 2 * single_count);
    VisionComm_Data.online = 1; // fire still high; release required again.
    f.step();
    assert(Shoot_Status.waiting_release && Dial_Status.shot_count == 2 * single_count);
    VisionComm_Data.fire = 0;
    f.step();
    VisionComm_Data.fire = 1;
    f.step();
    assert(Dial_Status.shot_count == 3 * single_count);
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
    assert(Dial_Status.shot_count == 1 && f.fsm.dial_ctrl.state == DialState::AUTO);
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
    Dial_Config_UpDown.long_press_ms = 10000; // Ignored in direct AUTO.
    Dial_Config_UpDown.auto_fire_hz = 10;
    Dial_Config_UpDown.wheel_to_hz = 99; // Ignored by vision.
    VisionComm_Data.online = 1;
    f.step();
    VisionComm_Data.fire = 1;
    f.step();
    f.step(8);
    assert(f.fsm.dial_ctrl.state == DialState::AUTO);
    float before = Dial_Status.target_angle;
    for (int i = 0; i < 10; ++i) f.step(20);
    assert(near(Dial_Status.target_angle-before, 40*(PI/180)));
    before = Dial_Status.target_angle;
    f.rc.s2 = Switch::DOWN;
    f.step();
    assert(near(Dial_Status.target_angle, dial_motor.angle)); // Cancel AUTO lead.
    assert(Shoot_Status.waiting_release && Dial_Status.shot_count == 2);
    VisionComm_Data.fire = 0;
    f.step();
    VisionComm_Data.fire = 1;
    before = Dial_Status.target_angle;
    f.step();
    assert(f.fsm.dial_ctrl.state == DialState::AUTO);
    assert(near(Dial_Status.target_angle-before, 1.6f*PI/180));
    assert(Dial_Status.shot_count == 2); // No initial single in down mode.
    before = Dial_Status.target_angle;
    for (int i = 0; i < 5; ++i) f.step(20);
    assert(near(Dial_Status.target_angle-before, 40*(PI/180)));
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
    for (int i = 0; i < 5; ++i) f.step(20);
    assert(near(Dial_Status.target_angle-before, 48*(PI/180)));
    before = Dial_Status.target_angle;
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

static void direct_auto_sources_stop_and_hold() {
    for (int source : {1,2,3}) {
        Fixture f;
        f.rc.s2 = Switch::DOWN;
        VisionComm_Data.online = source == 2;
        Dial_Config_UpDown.long_press_ms = 10000; // Never delays direct AUTO.
        Dial_PID_Config.vel_kp = 40;
        f.step(); // Establish ownership with trigger released.
        const float start = Dial_Status.target_angle;
        if (source == 1) f.rc.wheel = 1;
        if (source == 2) VisionComm_Data.fire = 1;
        if (source == 3) f.rc.mouse.left = true;
        for (int i=1; i<=25; ++i) {
            f.step();
            assert(f.fsm.dial_ctrl.state == DialState::AUTO);
            assert(Shoot_Status.trigger_source == source);
            assert(near(Dial_Status.target_angle-start, i*0.8f*PI/180));
            assert(Dial_Status.shot_count == 0);
        }
        // A 100 ms pulse plans only 20 degrees. Release never adds the missing 20.
        dial_motor.angle = start + 13*PI/180;
        dial_motor.velocity = 2;
        f.rc.wheel = 0;
        VisionComm_Data.fire = 0;
        f.rc.mouse.left = false;
        f.step();
        const float stopped = Dial_Status.target_angle;
        assert(near(stopped,dial_motor.angle) && dial_motor.torque < 0);
        assert(Dial_Status.shot_count == 0 && f.fsm.dial_ctrl.state == DialState::STOP);
        dial_motor.velocity = 0;
        for (int i=0; i<500; ++i) f.step();
        assert(near(Dial_Status.target_angle,stopped));
        assert(near(Dial_Status.home_delta_deg,0));
        // Restart directly from the held position without a correction step.
        if (source == 1) f.rc.wheel = 1;
        if (source == 2) VisionComm_Data.fire = 1;
        if (source == 3) f.rc.mouse.left = true;
        f.step();
        assert(f.fsm.dial_ctrl.state == DialState::AUTO && !Dial_Status.waiting_release);
        assert(near(Dial_Status.target_angle, dial_motor.angle+0.8f*PI/180));
    }
}

static void direct_auto_rate_changes_and_counting() {
    Fixture f;
    f.rc.s2 = Switch::DOWN;
    f.step();
    const float start = Dial_Status.target_angle;
    f.rc.wheel = 1;
    for (int i=0; i<50; ++i) f.step();
    assert(near(Dial_Status.target_angle-start,40*PI/180));
    assert(Dial_Status.shot_count == 1); // No synthetic single counted at trigger edge.
    Dial_Config_UpDown.auto_fire_hz = 10;
    float before = Dial_Status.target_angle;
    f.step();
    assert(near(Dial_Status.target_angle-before,1.6f*PI/180));
    assert(near(Dial_Config_UpUp.auto_fire_hz,5));
    for (float rate : {0.0f,-1.0f,std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<float>::infinity(),100.0f}) {
        Dial_Config_UpDown.auto_fire_hz = rate;
        before = Dial_Status.target_angle;
        f.step();
        assert(near(Dial_Status.target_angle-before,rate == 100 ? 8*PI/180 : 0));
    }
}

static void shared_pid_live_gains_and_limits() {
    Fixture f;
    // Identical holding error under both switch positions must use the same live gains.
    dial_motor.angle -= 0.1f;
    Dial_PID_Config.pos_kp = 4;
    Dial_PID_Config.vel_kp = 10;
    for (Switch s2 : {Switch::UP,Switch::DOWN}) {
        f.rc.s2 = s2;
        f.step();
        assert(near(Dial_Status.target_velocity,0.4f) && dial_motor.torque == 4);
    }
    Dial_PID_Config.vel_kp = 20;
    for (Switch s2 : {Switch::UP,Switch::DOWN}) {
        f.rc.s2 = s2;
        f.step();
        assert(dial_motor.torque == 8);
    }
    Dial_PID_Config.pos_vel_limit = 0.2f;
    Dial_PID_Config.raw_output_limit = 3;
    for (Switch s2 : {Switch::UP,Switch::DOWN}) {
        f.rc.s2 = s2;
        f.step();
        assert(near(Dial_Status.target_velocity,0.2f) && dial_motor.torque == 3);
    }
}

static void live_fire_mode_change_requires_release() {
    Fixture f;
    f.rc.s2 = Switch::DOWN;
    f.step();
    f.rc.wheel = 1; f.step();
    Dial_Config_UpDown.fire_mode = DialFireMode::SINGLE_THEN_AUTO;
    f.step();
    assert(Dial_Status.waiting_release && near(Dial_Status.target_angle,dial_motor.angle));
    assert(Dial_Status.shot_count == 0);
    f.rc.wheel = 0; f.step();
    f.rc.wheel = 1; f.step();
    assert(Dial_Status.shot_count == 1 && f.fsm.dial_ctrl.state == DialState::SINGLE);
    Dial_Config_UpDown.fire_mode = static_cast<DialFireMode>(2);
    f.step();
    assert(dial_motor.torque == 0 && Dial_Status.home_state == 0);
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

static void continuous_linear_target_and_release() {
    Fixture f;
    Dial_PID_Config.vel_kp = 40;
    f.rc.wheel = 1;
    f.step();
    const float first = Dial_Status.target_angle;
    f.step(20); // First AUTO cycle immediately advances at full slope.
    assert(near(Dial_Status.target_angle-first, 4*PI/180));
    float previous = Dial_Status.target_angle;
    for (int i=0; i<1045; ++i) {
        f.step();
        assert(near(Dial_Status.target_angle-previous, 0.8f*PI/180));
        previous = Dial_Status.target_angle;
    }
    assert(Dial_Status.shot_count == 22); // 4.2 seconds of AUTO + initial single.
    assert(near(Dial_Status.target_angle, 10 + 22*40*PI/180));
    dial_motor.angle = 10 + 13*PI/180;
    dial_motor.velocity = 2;
    f.rc.wheel = 0;
    f.step();
    const float held = Dial_Status.target_angle;
    assert(f.fsm.dial_ctrl.state == DialState::STOP);
    assert(near(held, dial_motor.angle) && near(Dial_Status.target_velocity, 0));
    assert(dial_motor.torque < 0); // Brake against positive velocity in release cycle.
    dial_motor.velocity = 0;
    dial_motor.angle += PI/180;
    for (int i=0; i<249; ++i) f.step();
    assert(near(Dial_Status.target_angle, held)); // Not a feedback-following target.
    assert(Dial_Status.home_state == 2); // Simulated feedback has drifted from the held target.
    for (int i=0; i<50; ++i) f.step();
    assert(near(Dial_Status.home_delta_deg, 0));
    assert(near(Dial_Status.target_angle,held));
    assert(Dial_Status.shot_count == 22);
    // A new single shot advances from the held relative target, not a slot grid.
    f.rc.wheel = 1;
    f.step();
    assert(Dial_Status.shot_count == 23 && !Dial_Status.waiting_release);
    assert(near(Dial_Status.target_angle, held+40*PI/180));
}

static void single_completion_without_idle_correction() {
    Fixture f;
    Dial_Calibration.stop_align_delay_ms = 1; // Legacy field must have no effect.
    f.rc.wheel = 1; f.step();
    const float committed = Dial_Status.target_angle;
    f.rc.wheel = 0; f.step();
    for (int i=0; i<100; ++i) f.step();
    assert(near(Dial_Status.target_angle, committed)); // Never cancel unfinished single.
    dial_motor.angle = committed;
    f.step(100); f.step(100); f.step(100);
    assert(near(Dial_Status.target_angle, committed) && Dial_Status.home_state == 3);
    // Drift later must not create any delayed correction target.
    dial_motor.angle += 5*PI/180;
    for (int i=0; i<100; ++i) f.step();
    assert(near(Dial_Status.target_angle, committed));
}

static void continuous_rate_guards() {
    for (float rate : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<float>::infinity(), 100.0f}) {
        Fixture f;
        Dial_Config_UpUp.auto_fire_hz = rate;
        f.rc.wheel = 1; f.step(); f.step(20);
        const float before = Dial_Status.target_angle;
        f.step();
        const float expected = rate == 100 ? 8*PI/180 : 0;
        assert(near(Dial_Status.target_angle-before,expected));
        f.rc.wheel = 0; f.step();
        assert(near(Dial_Status.target_angle,dial_motor.angle));
    }
}

static void jam_keeps_target_and_override_holds_current() {
    Fixture f;
    auto &cfg = Dial_Config_UpUp;
    cfg.jam_detect_enable = 1;
    cfg.jam_torque_threshold = 0.9f;
    cfg.jam_err_threshold = 0.01f;
    cfg.jam_duration_ms = 4;
    cfg.jam_reverse_ms = 8;
    cfg.jam_reverse_torque = -100;
    Dial_PID_Config.raw_output_limit = 1;
    Dial_PID_Config.pos_kp = Dial_PID_Config.vel_kp = 100;
    f.rc.wheel = 1; f.step();
    const float committed = Dial_Status.target_angle;
    assert(Dial_Status.jam_detected && dial_motor.torque == -100);
    f.step(); f.step();
    assert(!Dial_Status.jam_detected && near(Dial_Status.target_angle,committed));
    assert(Dial_Status.shot_count == 1 && Dial_Status.waiting_release);
    cfg.jam_detect_enable = 0;
    cfg.raw_override_enable = 1;
    cfg.raw_override_cmd = 123;
    f.step(); f.step();
    assert(dial_motor.torque == 123 && Dial_Status.shot_count == 1);
    dial_motor.angle += 3 * PI/180;
    const float override_exit_angle = dial_motor.angle;
    cfg.raw_override_enable = 0;
    f.step(100);
    assert(Dial_Status.home_state == 3 && Dial_Status.waiting_release);
    assert(near(Dial_Status.target_angle,override_exit_angle));
    assert(near(Dial_Status.home_delta_deg,0));
}


static void auto_release_cancels_unjam() {
    Fixture f;
    f.rc.wheel = 1; f.step(); f.step(20);
    auto &cfg = Dial_Config_UpUp;
    cfg.jam_detect_enable = 1;
    cfg.jam_torque_threshold = 0.9f;
    cfg.jam_err_threshold = 0.01f;
    cfg.jam_duration_ms = 4;
    cfg.jam_reverse_ms = 100;
    cfg.jam_reverse_torque = -100;
    Dial_PID_Config.raw_output_limit = 1;
    Dial_PID_Config.pos_kp = Dial_PID_Config.vel_kp = 100;
    f.step();
    assert(Dial_Status.jam_detected && dial_motor.torque == -100);
    f.step(); // Jam gate exits AUTO; release must still cancel reverse motion.
    dial_motor.velocity = -1;
    f.rc.wheel = 0; f.step();
    assert(!Dial_Status.jam_detected && dial_motor.torque > 0);
    assert(near(Dial_Status.target_angle,dial_motor.angle));
}

static void a1_startup_uses_current_position_without_correction() {
    struct Case { double legacy_zero; float phase; };
    const Case cases[] = {{32.704,28.677f},{0,35},{0,20},{0,21},{20,0},{32.704,32.704f}};
    for (const auto &c : cases) {
        Fixture f(false);
        Dial_Calibration.zero_output_deg = c.legacy_zero;
        dial_motor.phase_offset_deg = c.phase;
        f.step();
        assert(Dial_Status.home_state == 3 && Dial_Status.home_feedback_valid);
        assert(near(Dial_Status.a1_output_deg,c.phase));
        assert(near(Dial_Status.home_delta_deg,0));
        assert(near(Dial_Status.target_angle,10));
        assert(near(Dial_Status.target_velocity,0) && dial_motor.torque == 0);
        f.rc.wheel = 1; f.step();
        assert(Dial_Status.shot_count == 1 && Dial_Status.home_state == 2);
        assert(near(Dial_Status.home_error_deg,40));
        assert(near(Dial_Status.target_angle,10+40*PI/180));
        dial_motor.angle = Dial_Status.target_angle;
        f.rc.wheel = 0; f.step();
        assert(Dial_Status.home_state == 3);
    }
}

static void startup_reference_accepts_wheel_without_correction() {
    {
        Fixture f(false);
        dial_motor.phase_offset_deg = 25;
        f.step();
        assert(Dial_Status.home_state == 3 && near(Dial_Status.home_delta_deg,0));
        const float startup_target = Dial_Status.target_angle;
        assert(near(startup_target,dial_motor.angle));
        f.rc.wheel = 1;
        f.step();
        assert(!Dial_Status.waiting_release && Dial_Status.trigger_source == 1);
        assert(Dial_Status.shot_count == 1);
        assert(near(Dial_Status.target_angle,startup_target+40*PI/180));
    }
    {
        Fixture f(false);
        dial_motor.velocity = 1; // A moving but fresh A1 sample may establish reference.
        f.rc.wheel = 1;           // A held wheel is accepted on the first valid sample.
        f.step();
        assert(Dial_Status.slot_reference_valid && !Dial_Status.waiting_release);
        assert(Dial_Status.trigger_source == 1 && Dial_Status.shot_count == 1);
        assert(Dial_Status.control_source == 1 && Dial_Status.target_angle > dial_motor.angle);
    }
}

static void a1_feedback_loss_and_new_reference() {
    Fixture f;
    f.rc.wheel = 1; f.step();
    dial_motor.respond = false;
    f.step(101);
    assert(!Dial_Status.home_feedback_valid && Dial_Status.home_state == 1);
    assert(dial_motor.torque == 0 && !Dial_Status.slot_reference_valid);
    dial_motor.angle = 20;
    dial_motor.phase_offset_deg = 7 - (20-10)*180/PI;
    dial_motor.respond = true;
    f.step();
    assert(near(Dial_Status.home_delta_deg,0) && near(Dial_Status.target_angle,20));
    assert(Dial_Status.waiting_release && Dial_Status.trigger_source == 0);
    assert(Dial_Status.shot_count == 1);
    f.rc.wheel = 0;
    dial_motor.angle = Dial_Status.target_angle;
    f.step(); f.step(100);
    assert(Dial_Status.home_state == 3 && !Dial_Status.waiting_release && Dial_Status.shot_count == 1);
    dial_motor.online = false; f.step();
    assert(Dial_Status.home_state == 0 && dial_motor.torque == 0);
    dial_motor.online = true; f.step();
    assert(Dial_Status.home_state == 3 && !Dial_Status.waiting_release);
}

static void legacy_alignment_config_is_ignored() {
    for (int legacy_case=0; legacy_case<6; ++legacy_case) {
        Fixture f(false);
        if (legacy_case == 0) Dial_Calibration.zero_output_deg = std::numeric_limits<double>::quiet_NaN();
        if (legacy_case == 1) Dial_Calibration.home_tolerance_deg = 6;
        if (legacy_case == 2) Dial_Calibration.home_settle_ms = 0;
        if (legacy_case == 3) Dial_Calibration.home_raw_limit = -1;
        if (legacy_case == 4) Dial_Calibration.home_velocity_limit = -1;
        if (legacy_case == 5) dial_motor.phase_offset_deg = std::numeric_limits<float>::quiet_NaN();
        f.rc.wheel = 1;
        f.step();
        assert(Dial_Status.slot_reference_valid && Dial_Status.shot_count == 1);
        assert(near(Dial_Status.target_angle,10+40*PI/180));
        assert(near(Dial_Status.home_delta_deg,0));
    }
    for (int fault=0; fault<3; ++fault) {
        Fixture f(false);
        if (fault == 0) dial_motor.respond = false;
        if (fault == 1) dial_motor.angle = std::numeric_limits<float>::quiet_NaN();
        if (fault == 2) dial_motor.online = false;
        f.rc.wheel = 1;
        f.step(); f.step(101);
        assert(dial_motor.torque == 0 && Dial_Status.shot_count == 0);
        assert(!Dial_Status.slot_reference_valid && Dial_Status.home_state < 2);
    }
    Fixture f;
    const float held = Dial_Status.target_angle;
    Dial_Calibration.zero_output_deg = 5;
    f.step();
    assert(Dial_Status.home_state == 3 && near(Dial_Status.target_angle,held));
    assert(near(Dial_Status.home_delta_deg,0));
}

static void idle_never_realigns_to_slot_grid() {
    for (float degrees : {85.0f,527.0f,-45.0f}) {
        Fixture f;
        f.rc.s2 = Switch::DOWN;
        f.step();
        f.rc.wheel = 1; f.step();
        dial_motor.angle = 10+degrees*PI/180;
        f.rc.wheel = 0; f.step();
        const float held = Dial_Status.target_angle;
        for (int i=0; i<500; ++i) f.step();
        assert(near(Dial_Status.target_angle,held));
        assert(near(Dial_Status.target_angle,dial_motor.angle));
        assert(near(Dial_Status.home_delta_deg,0));
        assert(Dial_Status.shot_count == 0);
        assert(Dial_Status.home_state == 3);
    }
}

static void a1_cached_sample_still_expires() {
    Fixture f(false);
    f.step();
    assert(Dial_Status.home_state == 3);
    dial_motor.respond = false;
    f.step(100);
    assert(Dial_Status.home_state == 3);
    f.step(1);
    assert(Dial_Status.home_state == 1 && dial_motor.torque == 0);
    dial_motor.respond = true;
    f.step();
    assert(Dial_Status.home_state == 3);
}

int main() {
    startup_reference_accepts_wheel_without_correction();
    a1_startup_uses_current_position_without_correction();
    a1_feedback_loss_and_new_reference();
    legacy_alignment_config_is_ignored();
    idle_never_realigns_to_slot_grid();
    a1_cached_sample_still_expires();
    vision_priority_and_friction(Switch::UP);
    vision_priority_and_friction(Switch::DOWN);
    config_switch_and_single_completion();
    independent_vision_timing_and_rate();
    manual_threshold_rate_and_middle();
    direct_auto_sources_stop_and_hold();
    direct_auto_rate_changes_and_counting();
    shared_pid_live_gains_and_limits();
    live_fire_mode_change_requires_release();
    disable_and_estop_override_fire();
    continuous_linear_target_and_release();
    single_completion_without_idle_correction();
    continuous_rate_guards();
    jam_keeps_target_and_override_holds_current();
    auto_release_cancels_unjam();
    std::puts("PASS: A1-relative feeder without correction, direct AUTO, shared PID, dual-vision and jam scenarios");
}
