/*
 * Control.h
 *
 *  Created on: 2026锟斤拷1锟斤拷20锟斤拷
 *      Author: MR
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

typedef struct {
    float kp;               // 锟斤拷锟斤拷系锟斤拷
    float ki;               // 锟斤拷锟斤拷系锟斤拷
    float kd;               // 微锟斤拷系锟斤拷锟斤拷锟斤拷前未使锟矫ｏ拷锟斤拷锟斤拷锟斤拷展锟斤拷
    float target;           // 目锟斤拷转锟劫ｏ拷rpm锟斤拷
    float error;            // 锟斤拷前锟斤拷锟�
    float last_error;       // 锟斤拷一锟斤拷锟斤拷锟�
    float integral;         // 锟斤拷锟斤拷锟斤拷
    float output;           // PID锟斤拷锟斤拷锟秸硷拷毡龋锟�
    float output_min;       // 锟斤拷锟斤拷锟叫≈碉拷锟�-6000锟斤拷
    float output_max;       // 锟斤拷锟斤拷锟斤拷值锟斤拷6000锟斤拷
    float integ_min;        //锟斤拷锟斤拷锟斤拷min
    float integ_max;        //锟斤拷锟斤拷锟斤拷max

    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟皆诧拷锟斤拷微锟斤拷 + 锟斤拷通锟斤拷锟斤拷
    float last_measure;
    float d_filt;
    float d_tau;        // 微锟街碉拷通时锟戒常锟斤拷(s)锟斤拷锟斤拷 0.02f
} PID_Controller;

typedef enum
{
    BALANCE_MODE_RATE_ANGLE_STEERING = 0, // 锟斤拷锟劫度伙拷锟斤拷锟角度伙拷锟斤拷转锟斤拷
    BALANCE_MODE_ANGLE_STEERING      = 1  // 锟角度伙拷锟斤拷转锟斤拷
} BalanceControlMode;


void pid_init_2(PID_Controller *pid, float kp, float ki, float kd,
             float output_min, float output_max,float integ_min,float integ_max);
float clampf(float a,float b,float c);
float pid_pos_update_angular_velocity(PID_Controller *pid, float measure, float dt);
extern PID_Controller Rate_Servo;
extern PID_Controller RollAnglePID;
extern PID_Controller AngleServoPID;
void servo_position_pid_control(PID_Controller *pid, float measure, float dt);
extern uint8 stop_flag;  //锟斤拷锟街碉拷锟酵Ｖ癸拷锟街疚�
void Protect_roll(void);
void display_pid(void);
void speed_control(void);
void balance_control_update(float dt);
float balance_angle_loop_update(float roll_deg, float dt);
float balance_rate_loop_update(float roll_rate_dps, float roll_rate_ref, float dt);
void balance_angle_only_control_update(float dt);
void set_balance_control_mode(BalanceControlMode mode);
BalanceControlMode get_balance_control_mode(void);
void set_roll_ref_deg(float ref_deg);
extern float u;
extern float roll_rate_ref_dps;

// ===== 锟斤拷锟斤拷转锟津环ｏ拷锟斤拷锟解环锟斤拷 =====
extern PID_Controller YawPID;
extern float desired_yaw;        // 锟斤拷锟斤拷锟斤拷锟斤拷牵锟絛eg锟斤拷
void steering_control_update(float dt);  // 锟斤拷 Mahony 锟斤拷锟铰猴拷锟斤拷锟�
void set_desired_yaw(float yaw_deg);     //
void update_pid_gains_by_speed(float speed);  //

extern float Ctrl_lastSpeed;

#endif /* CODE_CONTROL_H_ */
