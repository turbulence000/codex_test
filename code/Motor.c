/*
 * Motor.c
 *
 *  Created on: 2025锟斤拷10锟斤拷3锟斤拷
 *      Author: MR
 */

#include "zf_common_headfile.h"
#include <math.h>   // fabsf

#define encoder_ware             1024
#define INTERRUPT_INTERVAL_MS    10    // 锟叫断硷拷锟�(ms)

/* ===================== 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷倩锟斤拷锟� ===================== */
static float motor_speed_prev = 0.0f;   // 锟斤拷一锟斤拷锟剿诧拷锟斤拷锟阶拷锟�
static float motor_speed      = 0.0f;   // 锟斤拷前锟剿诧拷锟斤拷锟阶拷锟�(rpm)

/* 锟斤拷锟斤拷锟绞癸拷玫谋锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟� TIM2锟斤拷 */
static encoder_index_enum motor_encoder = TIM2_ENCODER;
PID_Controller speed_pid;

/* ===================== 锟斤拷锟斤拷锟斤拷锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟� ===================== */
void Encoder_Init(void)
{
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟絋IM2
    encoder_quad_init(TIM2_ENCODER,
                      TIM2_ENCODER_CH1_P33_7,
                      TIM2_ENCODER_CH2_P33_6);

    // 锟斤拷锟斤拷锟斤拷锟揭拷锟斤拷锟� TIM4锟斤拷锟斤拷 motor_encoder 锟斤拷锟斤拷锟斤拷锟芥即锟斤拷
    // encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
}


/* ===================== PID 锟斤拷始锟斤拷锟斤拷锟斤拷锟皆拷锟斤拷锟斤拷锟斤拷锟斤拷锟� ===================== */
void pid_motor_speed_init(PID_Controller *pid, float kp, float ki, float kd,
                          float output_min, float output_max,
                          float integ_min, float integ_max)
{
    if (pid == NULL) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output_min = output_min;
    pid->output_max = output_max;

    pid->target = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;

    pid->integ_min = integ_min;
    pid->integ_max = integ_max;
}


/* 锟斤拷原锟斤拷锟侥硷拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷呒锟斤拷锟斤拷锟� */
int Encoder_count(encoder_index_enum encoder_n)
{
    if (encoder_n == 0)
        return -encoder_get_count(encoder_n);
    else
        return encoder_get_count(encoder_n);
}

int diff_navigation = 0;
/* ===================== 锟斤拷锟斤拷锟斤拷锟斤拷伲锟揭伙拷椎锟酵拷锟� ===================== */
void motor_speed_calculate(void)
{
    diff_navigation = Encoder_count(motor_encoder);
    encoder_clear_count(motor_encoder);

    // rpm = 锟斤拷锟斤拷锟斤拷 * 60000 / (每转锟斤拷锟斤拷 * 锟斤拷锟斤拷锟斤拷锟斤拷ms)
    float new_speed = (float)diff_navigation * 60000.0f /
                      (encoder_ware * INTERRUPT_INTERVAL_MS);

    // 一锟阶碉拷通锟剿诧拷
    motor_speed = 0.88f * motor_speed_prev + 0.12f * new_speed;
    motor_speed_prev = motor_speed;
}


/* 锟斤拷取锟斤拷锟阶拷伲锟斤拷锟斤拷锟斤拷锟芥本锟斤拷锟斤拷 motor_id锟斤拷锟斤拷锟斤拷锟斤拷锟接口诧拷锟狡伙拷锟解部锟斤拷锟斤拷 */
float motor_get_speed(void)
{

    return motor_speed;
}


/* ===================== 锟斤拷锟斤拷锟皆拷锟斤拷锟斤拷锟� ===================== */
#define SERVO_PWM_FREQ_HZ      (300)
#define SERVO_MIN_PULSE_MS     (0.5f)
#define SERVO_MAX_PULSE_MS     (2.5f)

static float servo_angle_to_duty(float angle)
{
    angle = clampf(angle, 0.0f, 180.0f);

    float pulse_ms = SERVO_MIN_PULSE_MS + (angle / 180.0f) * (SERVO_MAX_PULSE_MS - SERVO_MIN_PULSE_MS);
    float period_ms = 1000.0f / (float)SERVO_PWM_FREQ_HZ;
    return (pulse_ms / period_ms) * 10000.0f;
}

void steering_engine_Init(void)
{
    int duty = (int)servo_angle_to_duty(92.5f);
    pwm_init(ATOM1_CH1_P33_9, SERVO_PWM_FREQ_HZ, duty);
}

void steering_engine_set_angle(float angle)
{
    float duty = servo_angle_to_duty(angle);
    pwm_set_duty(ATOM1_CH1_P33_9, duty);
}

void motor_speed_display(void)
{
   printf("%f\n",motor_get_speed());
    //printf("%f\n",del);
}


/* 锟剿诧拷锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷锟� */
void motor_filter_init(void)
{
    motor_speed_prev = 0.0f;
    motor_speed = 0.0f;
}


/* ===================== 锟劫度伙拷 PID + 前锟斤拷 ===================== */
#define SPEED_FF_K          (0.6f)  // duty per rpm
#define SPEED_FF_B          (80.f)    // duty offset
#define SPEED_FF_RPM_EPS    (1.0f)     // rpm锟斤拷小锟节革拷值锟斤拷为目锟斤拷=0锟斤拷锟斤拷锟斤拷偏锟斤拷
#define MOTOR_SPEED_TARGET_MAX_RPM  (4000.0f)


/* 锟借定目锟斤拷转锟劫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟� motor_id */
void motor_set_speed( float target_rpm)
{
    target_rpm = clampf(target_rpm,
                        -MOTOR_SPEED_TARGET_MAX_RPM,
                         MOTOR_SPEED_TARGET_MAX_RPM);

    speed_pid.target = target_rpm;
}


/* 停转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟� motor_id */
void motor_stop(void)
{
    speed_pid.target = 0.0f;
    speed_pid.error = 0.0f;
    speed_pid.last_error = 0.0f;
    speed_pid.integral = 0.0f;
    speed_pid.output = 0.0f;

    small_driver_set_duty(0, 0);
}


/* PID + 前锟斤拷锟斤拷锟� duty */
float pid_speed_ff_update(PID_Controller *pid, float feedback_rpm)
{
    if (pid == NULL) return 0.0f;

    float dt = (float)INTERRUPT_INTERVAL_MS * 0.001f;
    if (dt <= 0.0f) dt = 1e-3f;

    // ---------- (A) 前锟斤拷 ----------
    float duty_ff = 0.0f;
    float target = pid->target;

    if (fabsf(target) > SPEED_FF_RPM_EPS)
    {
        float abs_rpm = fabsf(target);
        duty_ff = SPEED_FF_K * abs_rpm + SPEED_FF_B;
        // 锟斤拷锟斤拷支锟街凤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤按 target 锟斤拷锟脚革拷 duty_ff 锟接凤拷锟斤拷
        // duty_ff = (target > 0 ? duty_ff : -duty_ff);
    }

    // ---------- (B) PID ----------
    pid->error = pid->target - feedback_rpm;

    pid->integral += pid->error * dt;
    pid->integral = clampf(pid->integral, pid->integ_min, pid->integ_max);

    float derivative = (pid->error - pid->last_error) / dt;

    float duty_pid = pid->kp * pid->error
                   + pid->ki * pid->integral
                   + pid->kd * derivative;

    // ---------- (C) 锟较筹拷锟睫凤拷 ----------
    float out = duty_ff + duty_pid;
    out = clampf(out, pid->output_min, pid->output_max);

    pid->output = out;
    pid->last_error = pid->error;

    return out;
}

//float del = pid_speed_ff_update();
/* 每锟斤拷锟节碉拷锟矫ｏ拷锟斤拷 motor_speed_calculate() 锟斤拷 motor_speed_control_step() */
void motor_speed_control_step(void)
{
    float fb_rpm = motor_get_speed();
    float u = pid_speed_ff_update(&speed_pid, fb_rpm);
    u = u/voltage*12.2;
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷一通锟斤拷为0
    //printf("%f\n",u);
    small_driver_set_duty(u, 0);
}
