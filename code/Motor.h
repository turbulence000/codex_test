/*
 * Motor.h
 *
 *  Created on: 2025年10月3日
 *      Author: MR
 */

#include "zf_common_headfile.h"
#include "Control.h"
#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_



/* ===================== 编码器初始化（单电机） ===================== */
void Encoder_Init(void);

extern PID_Controller speed_pid;
extern int diff_navigation ;
/* ===================== PID 初始化（你的原函数保留） ===================== */
void pid_motor_speed_init(PID_Controller *pid, float kp, float ki, float kd,
                          float output_min, float output_max,
                          float integ_min, float integ_max);



/* 你原来的计数方向兼容逻辑保留 */
int Encoder_count(encoder_index_enum encoder_n);


/* ===================== 单电机测速（一阶低通） ===================== */
void motor_speed_calculate(void);



/* 获取电机转速：单电机版本忽略 motor_id，但保留接口不破坏外部代码 */
float motor_get_speed();


/* ===================== 舵机：原样保留 ===================== */
void steering_engine_Init(void);

void steering_engine_set_angle(float angle);


void motor_speed_display(void);

/* 滤波初始化：单电机 */
void motor_filter_init(void);


/* 设定目标转速：单电机忽略 motor_id */
void motor_set_speed(float target_rpm);


/* 停转：单电机忽略 motor_id */
void motor_stop();



/* PID + 前馈输出 duty */
float pid_speed_ff_update(PID_Controller *pid, float feedback_rpm);

/* 每周期调用：先 motor_speed_calculate() 再 motor_speed_control_step() */
void motor_speed_control_step(void);



#endif /* CODE_MOTOR_H_ */

