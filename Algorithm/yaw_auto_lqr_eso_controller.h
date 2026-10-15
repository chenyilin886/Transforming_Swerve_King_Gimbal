/**
 ******************************************************************************
 * @file    yaw_auto_lqr_eso_controller.h
 * @school  南京理工大学（NJUST）
 * @author  戴子轩
 * @email   daizeming24@gmail.com
 * @date    2026/5/16
 * @brief   Standalone yaw auto-aim LQR-ESO controller matching the math summary.
 *
 * This pair is a documentation companion for yaw-auto-lqreso-math-summary.docx.
 * It keeps only the continuous-time reference tracking, feedforward, LQR,
 * third-order ESO, disturbance compensation, torque limit, and slew limit chain.
 ******************************************************************************
 */

#ifndef YAW_AUTO_LQR_ESO_CONTROLLER_H
#define YAW_AUTO_LQR_ESO_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float j_kg_m2;                    /* 转动惯量 J (kg*m^2) */
    float b_nms_rad;                  /* 粘性阻尼 B (N*m*s/rad) */
    float k_theta;                    /* 角度误差反馈增益 */
    float k_omega;                    /* 角速度误差反馈增益 */
    float k_i;                        /* 角度误差积分增益；0=关闭 */
    float theta_integral_limit_rad_s; /* 积分状态限幅 (rad*s；字段名沿用原接口) */
    float tau_coulomb_nm;             /* 库仑摩擦前馈力矩 (N*m) */
    float coulomb_smooth_rad_s;       /* 库仑摩擦平滑速度 (rad/s) */
    float eso_bandwidth_rad_s;        /* ESO 带宽 w0 (rad/s) */
    float eso_comp_gain;              /* ESO 扰动补偿比例；0=关闭比例补偿 */
    float eso_comp_limit_nm;          /* ESO 补偿力矩绝对值限幅 (N*m) */
    float eso_omega_gate_rad_s;       /* 补偿反馈角速度门槛；<=0=不限制 */
    float eso_alpha_gate_rad_s2;      /* 补偿参考角加速度门槛；<=0=不限制 */
    float tau_bias_ki;                /* 力矩偏置积分增益；0=关闭 */
    float tau_bias_limit_nm;          /* 力矩偏置补偿限幅 (N*m) */
    float tau_meas_lpf_alpha;         /* 实测力矩低通滤波系数 alpha (0~1) */
    float theta_deadband_rad;         /* 角度误差死区；0=关闭 */
    float torque_soft_limit_nm;       /* 对称软限幅 +/-N*m；<=0=关闭 */
    float torque_min_nm;              /* 输出硬限幅下界 (N*m) */
    float torque_max_nm;              /* 输出硬限幅上界 (N*m) */
    float torque_slew_rate_nm_s;      /* 输出力矩变化率上限 (N*m/s) */
    uint8_t eso_enable;               /* ESO 观测器开关 (1=运行) */
    uint8_t eso_comp_enable;          /* ESO 扰动补偿开关 (1=加入输出) */
    uint8_t torque_slew_enable;       /* 力矩变化率限制开关 (1=启用) */
} YawAutoLqrEsoConfig_t;

typedef struct
{
    float theta_rad;
    float omega_rad_s;
} YawAutoLqrEsoState_t;

typedef struct
{
    float theta_rad;
    float omega_rad_s;
    float alpha_rad_s2;
} YawAutoLqrEsoReference_t;

typedef struct
{
    float theta_rad;
    float omega_rad_s;
    float tau_meas_nm;
    uint8_t feedback_ok;
} YawAutoLqrEsoFeedback_t;

typedef struct
{
    float z1;
    float z2;
    float z3;
    float b0;
    float w0;
    float beta1;
    float beta2;
    float beta3;
} YawAutoLqrEsoObserver_t;

typedef struct
{
    YawAutoLqrEsoObserver_t eso;
    float theta_integral_rad_s;
    float tau_bias_nm;
    float tau_meas_filt_nm;
    float tau_cmd_last_nm;
    uint8_t feedback_ready;
} YawAutoLqrEso_t;

typedef struct
{
    float a[2][2];
    float b[2][2];   /* input vector is [tau, d] */
    float c[2];
} YawAutoLqrEsoStateSpace_t;

typedef struct
{
    float theta_ref_rad;
    float omega_ref_rad_s;
    float alpha_ref_rad_s2;
    float e_theta_rad;
    float e_omega_rad_s;
    float tau_ff_alpha_nm;
    float tau_ff_viscous_nm;
    float tau_ff_coulomb_nm;
    float tau_ff_nm;
    float tau_i_nm;
    float tau_lqr_nm;
    float tau_eso_raw_nm;
    float tau_eso_active_nm;
    float tau_bias_nm;
    float tau_pre_limit_nm;
    float tau_cmd_before_slew_nm;
    float tau_cmd_nm;
    uint8_t eso_comp_active;
    uint8_t soft_limit_active;
    uint8_t hard_limit_active;
    uint8_t slew_limit_active;
} YawAutoLqrEsoOutput_t;

void YawAutoLqrEso_Init(YawAutoLqrEso_t *ctrl);
void YawAutoLqrEso_Reset(YawAutoLqrEso_t *ctrl, float theta_rad, float omega_rad_s);
uint8_t YawAutoLqrEso_GetStateSpace(const YawAutoLqrEsoConfig_t *cfg,
                                    YawAutoLqrEsoStateSpace_t *model);
void YawAutoLqrEso_Calc(YawAutoLqrEso_t *ctrl,
                        const YawAutoLqrEsoConfig_t *cfg,
                        const YawAutoLqrEsoFeedback_t *feedback,
                        const YawAutoLqrEsoReference_t *ref,
                        float dt_s,
                        YawAutoLqrEsoOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif
