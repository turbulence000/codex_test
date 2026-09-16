/*
 * Control.c
 *
 *  Created on: 2026锟斤拷1锟斤拷20锟斤拷
 *      Author: MR
 */
/*********************************************************************************************************************
 * @brief
 * @param none
 * @example
 * @date 2026.1.20
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "route.h"
/*********************************************************************************************************************
 * @brief PID 锟斤拷始锟斤拷
 * @param
 * @example
 * @date 2026.1.20
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/

// ======================== PID 锟斤拷始锟斤拷 ========================
void pid_init_2(PID_Controller *pid, float kp, float ki, float kd,
			 float output_min, float output_max,float integ_min,float integ_max)

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

	// 微锟街碉拷通锟剿诧拷锟斤拷锟斤拷
	pid->last_measure = 0.0f;
	pid->d_filt = 0.0f;
	pid->d_tau = 0.02f;   // 默锟斤拷时锟戒常锟斤拷锟斤拷锟缴帮拷锟斤拷要锟睫革拷


}
/*********************************************************************************************************************
 * @brief 锟斤拷锟睫凤拷锟斤拷锟斤拷
 * @param none
 * @example
 * @date 2026.1.20
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/
float clampf(float a,float b,float c)
{
	if(a<b)
	{
		a=b;
	}
	else if(a>c)
	{
		a=c;
	}
	return a;

}

// ======================== 速度相关 PID 增益调度 ========================
// 线性函数: kp = kp_0 + slope * speed
// RollAnglePID: 速度越高增益越小（陀螺效应）
#define  ANGLE_KP_0      11.5f       // speed=0 时的 Kp
#define  ANGLE_KP_SLOPE  -0.00107f   // 每 1000 RPM 降约 1.07

// YawPID: 速度越高转向越灵敏（|Kp| 更大）
#define  YAW_KP_0        -0.32f      // speed=0 时的 Kp
#define  YAW_KP_SLOPE    -0.000043f  // 每 1000 RPM 增约 0.043

void update_pid_gains_by_speed(float speed)
{
	RollAnglePID.kp = ANGLE_KP_0  + ANGLE_KP_SLOPE * speed;
	YawPID.kp       = YAW_KP_0    + YAW_KP_SLOPE   * speed;
}

/*********************************************************************************************************************
 * @brief 位锟斤拷式 PID 锟斤拷锟铰ｏ拷锟斤拷锟截匡拷锟斤拷锟斤拷锟斤拷锟斤拷位 deg锟斤拷
 * @param  PID_Controller *pid PID 锟结构锟斤拷指锟诫；float measure 锟斤拷前锟斤拷锟斤拷值锟斤拷float dt PID 锟斤拷锟斤拷锟斤拷锟斤拷
 * @example
 * @date 2026.1.20
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/
float pid_pos_update_angular_velocity(PID_Controller *pid, float measure, float dt)
{

	pid->error = pid->target - measure;

	if (pid->ki != 0.0f)
	{
		pid->integral += pid->error * dt;
		pid->integral = clampf(pid->integral, pid->integ_min, pid->integ_max);
	}
	else
	{
		pid->integral = 0.0f;
	}

	// D 锟斤拷使锟矫诧拷锟斤拷值微锟斤拷 dy/dt锟斤拷锟斤拷锟斤拷目锟斤拷突锟戒导锟斤拷微锟街筹拷锟�
	float d_raw = (measure - pid->last_measure) / dt;  // dy/dt
	pid->last_measure = measure;

	// 一锟阶碉拷通锟斤拷d_filt = alpha*d_filt + (1-alpha)*d_raw
	float tau = (pid->d_tau > 1e-6f) ? pid->d_tau : 0.02f;
	float alpha = tau / (tau + dt);
	pid->d_filt = alpha * pid->d_filt + (1.0f - alpha) * d_raw;

	// 注锟斤拷锟斤拷牛锟紻 on measurement => u += -kd * dy/dt
	float out = pid->kp * pid->error + pid->ki * pid->integral - pid->kd * pid->d_filt;

	out = clampf(out,pid->output_min,pid->output_max);

	pid->output = out;
	return out;
}

//////////////////////// PID 锟斤拷锟斤拷锟斤拷实锟斤拷 ///////////////////////////////
PID_Controller Rate_Servo;
// 双锟斤拷模式锟斤拷锟角度伙拷 -> 锟斤拷锟侥匡拷锟斤拷锟劫讹拷
PID_Controller RollAnglePID;
// 锟斤拷锟斤拷模式锟斤拷锟角度伙拷 -> 直锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
PID_Controller AngleServoPID;
// 转锟斤拷锟斤拷锟解环锟斤拷锟斤拷锟斤拷 -> 锟斤拷锟侥匡拷锟斤拷锟斤拷锟� roll_ref_deg
PID_Controller YawPID;

static BalanceControlMode g_balance_control_mode = BALANCE_MODE_RATE_ANGLE_STEERING;

// ====== 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 ======
#define SERVO_CENTER_DEG  92.5f
#define SERVO_MIN_DEG     0.0f
#define SERVO_MAX_DEG     180.0f

// ====== 平锟斤拷锟斤拷转锟斤拷锟斤拷夭锟斤拷锟斤拷锟绞凳憋拷锟叫э拷锟� ======
#define ROLL_MECHANICAL_ZERO    (1.5f)      // 锟斤拷械锟斤拷位偏锟矫ｏ拷直锟斤拷时锟斤拷目锟斤拷锟斤拷锟角ｏ拷deg锟斤拷
#define ROLL_REF_MAX_DEG        (20.0f)     // 转锟斤拷锟斤拷锟斤拷锟侥匡拷锟斤拷锟斤拷锟斤拷薹锟斤拷锟絛eg锟斤拷
#define ROLL_RATE_REF_MAX_DPS   (800.0f)    // 锟角度伙拷锟斤拷锟斤拷锟侥匡拷锟斤拷锟劫讹拷锟睫凤拷锟斤拷deg/s锟斤拷
#define SERVO_U_MAX_DEG         (60.0f)     // 锟斤拷锟斤拷锟斤拷锟斤拷薹锟斤拷锟絛eg锟斤拷
#define ANGLE_OUT_LPF_ALPHA     (1.0f)     // 锟角度伙拷锟斤拷锟斤拷锟酵ㄏ碉拷锟斤拷锟�0~1锟斤拷越小越平锟斤拷锟斤拷
// 目锟斤拷锟叫憋拷锟斤拷俣龋锟統aw_setpoint 每锟斤拷锟斤拷锟戒化锟角度ｏ拷越小越平锟斤拷
#define YAW_SLEW_RATE_DPS       (150.0f)

float roll_ref_deg = 1.5f;   // 目锟斤拷锟斤拷锟角ｏ拷锟斤拷转锟斤拷锟斤拷锟竭革拷锟铰ｏ拷
float desired_yaw   = 0.0f;  // 锟斤拷锟斤拷锟斤拷锟斤拷牵锟絛eg锟斤拷锟斤拷锟斤拷 set_desired_yaw() 锟斤拷锟斤拷
static float yaw_setpoint = 0.0f; // 锟斤拷前斜锟斤拷目锟疥，锟金步逼斤拷锟斤拷锟斤拷锟斤拷锟斤拷

// 转锟斤拷锟津不对可革拷为 +1
#define SERVO_SIGN        (-1.0f)
/*********************************************************************************************************************
 * @brief 锟斤拷锟轿伙拷锟绞� PID 锟斤拷锟铰ｏ拷锟斤拷锟截匡拷锟斤拷锟斤拷锟斤拷锟斤拷位 deg锟斤拷
 * @param  PID_Controller *pid PID 锟结构锟斤拷指锟诫；float measure 锟斤拷前锟斤拷锟斤拷值锟斤拷float dt PID 锟斤拷锟斤拷锟斤拷锟斤拷
 * @example
 * @date 2026.1.20
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/
float u_deg =0;
float cmd = 0;

void servo_position_pid_control(PID_Controller *pid, float measure, float dt)
{
	u_deg = pid_pos_update_angular_velocity(pid, measure, dt);   // PID 锟斤拷锟斤拷锟斤拷锟斤拷锟絛eg锟斤拷
	u_deg *= SERVO_SIGN;
	cmd = SERVO_CENTER_DEG + u_deg; // 锟斤拷锟斤拷锟斤拷越锟斤拷锟斤拷锟�
	cmd = clampf(cmd, SERVO_MIN_DEG, SERVO_MAX_DEG);
	steering_engine_set_angle(cmd);


}

float u;
float roll_rate_ref_dps = 0.0f;
static float roll_rate_ref_dps_lpf = 0.0f;
static int roll_rate_ref_lpf_inited = 0;

float balance_angle_loop_update(float roll_deg, float dt)
{
	if (dt <= 1e-6f) dt = 1e-3f;

	roll_ref_deg = clampf(roll_ref_deg, -ROLL_REF_MAX_DEG, ROLL_REF_MAX_DEG);
	RollAnglePID.target = roll_ref_deg;

	float roll_rate_ref_raw = pid_pos_update_angular_velocity(&RollAnglePID, roll_deg, dt);

	// 锟角度伙拷锟斤拷锟揭伙拷椎锟酵拷锟斤拷锟斤拷平锟斤拷俣锟侥匡拷甓讹拷锟�
	if (!roll_rate_ref_lpf_inited)
	{
		roll_rate_ref_dps_lpf = roll_rate_ref_raw;
		roll_rate_ref_lpf_inited = 1;
	}
	else
	{
		roll_rate_ref_dps_lpf += ANGLE_OUT_LPF_ALPHA * (roll_rate_ref_raw - roll_rate_ref_dps_lpf);
	}

	roll_rate_ref_dps = clampf(roll_rate_ref_dps_lpf, -ROLL_RATE_REF_MAX_DPS, ROLL_RATE_REF_MAX_DPS);
	return roll_rate_ref_dps;
}

float balance_rate_loop_update(float roll_rate_dps, float roll_rate_ref, float dt)
{
	if (dt <= 1e-6f) dt = 1e-3f;

	Rate_Servo.target = roll_rate_ref;
	return pid_pos_update_angular_velocity(&Rate_Servo, roll_rate_dps, dt);
}

void set_balance_control_mode(BalanceControlMode mode)
{
	g_balance_control_mode = mode;
}

BalanceControlMode get_balance_control_mode(void)
{
	return g_balance_control_mode;
}

void balance_angle_only_control_update(float dt)
{

	float roll_deg = icm42688_roll;

	roll_ref_deg = clampf(roll_ref_deg, -ROLL_REF_MAX_DEG, ROLL_REF_MAX_DEG);
	AngleServoPID.target = roll_ref_deg;

	u = pid_pos_update_angular_velocity(&AngleServoPID, roll_deg, dt);
	u *= SERVO_SIGN;

	float cmd_local = SERVO_CENTER_DEG + u;
	cmd_local = clampf(cmd_local, SERVO_MIN_DEG, SERVO_MAX_DEG);
	steering_engine_set_angle(cmd_local);
}

void balance_control_update(float dt)
{
	// 锟斤拷锟斤拷锟斤拷dt 锟斤拷锟斤拷太小
	if (dt <= 1e-6f) dt = 1e-3f;

	update_pid_gains_by_speed(Ctrl_lastSpeed);

	if (g_balance_control_mode == BALANCE_MODE_ANGLE_STEERING)
	{
		balance_angle_only_control_update(dt);
		return;
	}

	// 1) 锟斤拷取锟斤拷态锟斤拷锟斤拷俣龋锟斤拷锟斤拷锟� IMU锟斤拷
	float roll_deg = Roll_a;          // deg
	// 锟斤拷锟斤拷锟斤拷俣龋锟侥拷锟绞癸拷锟� gyro_x锟斤拷deg/s锟斤拷
	float roll_rate_dps = Roll_g_F;   // deg/s

	// 2) 锟解环锟角度伙拷锟斤拷roll_ref -> roll_rate_ref
	float roll_rate_ref = balance_angle_loop_update(roll_deg, dt);

	// 3) 锟节伙拷锟斤拷锟劫度伙拷锟斤拷roll_rate_ref -> servo_u
	u = balance_rate_loop_update(roll_rate_dps, roll_rate_ref, dt);

	// 4) 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟侥斤拷 + 锟斤拷锟斤拷
	u *= SERVO_SIGN;
	float cmd_local = SERVO_CENTER_DEG + u;
	cmd_local = clampf(cmd_local, SERVO_MIN_DEG, SERVO_MAX_DEG);
	steering_engine_set_angle(cmd_local);


}

//// ======================== 锟解部锟接口ｏ拷锟斤拷锟矫固讹拷 roll_ref锟斤拷锟斤拷锟斤拷转锟斤拷时使锟矫ｏ拷 ========================
void set_roll_ref_deg(float ref_deg)
{
	roll_ref_deg = ref_deg;
}
/*********************************************************************************************************************
 * @brief 锟斤拷锟斤拷潜锟斤拷锟斤拷锟斤拷锟�
 * @param none
 * @example Protect_roll()
 * @date 2026.1.20
 * @remark 锟斤拷锟斤拷蔷锟斤拷锟街碉拷锟斤拷锟� 35 时锟斤拷停锟斤拷锟斤拷志
 * @remark
 * @remark
 ********************************************************************************************************************/
uint8 stop_flag = 1;  // 锟斤拷锟街碉拷锟酵Ｖ癸拷锟街疚�
void Protect_roll(void)
{
	if(abs((int)icm42688_roll)>=45)
	{
		stop_flag =1;
	}
//	if(key_get_short_press()==3)
//	{
//		stop_flag=0;
//	}

}

/*********************************************************************************************************************
 * @brief 锟斤拷锟角度诧拷娣讹拷锟斤拷锟� [-180, 180]
 ********************************************************************************************************************/
static float normalize_angle_diff(float diff)
{
	while (diff >  180.0f) diff -= 360.0f;
	while (diff < -180.0f) diff += 360.0f;
	return diff;
}

/*********************************************************************************************************************
 * @brief 转锟斤拷锟斤拷锟解环锟斤拷锟斤拷锟斤拷 yaw -> 目锟斤拷锟斤拷锟斤拷 roll_ref_deg
 * @param dt  锟斤拷锟斤拷锟斤拷锟节ｏ拷s锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 Mahony 同锟斤拷锟节ｏ拷0.005s锟斤拷
 * @remark 锟斤拷 Mahony 锟斤拷锟铰猴拷balance_control_update 之前锟斤拷锟斤拷
 *         锟斤拷锟津不讹拷时锟缴碉拷锟斤拷 YawPID 锟斤拷 kp 锟斤拷锟斤拷
 ********************************************************************************************************************/
void steering_control_update(float dt)
{
	if (dt <= 1e-6f) dt = 1e-3f;

	update_pid_gains_by_speed(Ctrl_lastSpeed);

	// ---- 手推录制回放 (navigation.c) ----
	if (N.Nag_SystemRun_Index == 2 && !N.Nag_Stop_f)
	{
		YawPID.target = 0.0f;

		float out = pid_pos_update_angular_velocity(&YawPID, -N.Final_Out, dt);
		roll_ref_deg = ROLL_MECHANICAL_ZERO + clampf(out, -ROLL_REF_MAX_DEG, ROLL_REF_MAX_DEG);
		return;
	}

	// ---- XY路径回放 (route.c) ----
	if (route_is_running())
	{
		route_step(diff_navigation);
		float target = g_route.target_yaw;
		float yaw    = icm42688_yaw;

		YawPID.target = target;
		float yaw_err = normalize_angle_diff(target - yaw);
		float yaw_meas_equiv = target - yaw_err;
		float out = pid_pos_update_angular_velocity(&YawPID, yaw_meas_equiv, dt);
		roll_ref_deg = ROLL_MECHANICAL_ZERO + clampf(out, -ROLL_REF_MAX_DEG, ROLL_REF_MAX_DEG);
		return;
	}

	// ---- 普通惯导跟随 ----
	float yaw = icm42688_yaw;

	// 目标航向平滑, 防止 desired_yaw 突变
	float rem = normalize_angle_diff(desired_yaw - yaw_setpoint);
	float max_step = YAW_SLEW_RATE_DPS * dt;
	yaw_setpoint += clampf(rem, -max_step, max_step);
	yaw_setpoint = normalize_angle_diff(yaw_setpoint);

	YawPID.target = yaw_setpoint;

	float yaw_err = normalize_angle_diff(yaw_setpoint - yaw);
	float yaw_meas_equiv = yaw_setpoint - yaw_err;
	float out = pid_pos_update_angular_velocity(&YawPID, yaw_meas_equiv, dt);
	roll_ref_deg = ROLL_MECHANICAL_ZERO + clampf(out, -ROLL_REF_MAX_DEG, ROLL_REF_MAX_DEG);
}

/*********************************************************************************************************************
 * @brief 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
 * @param yaw_deg  目锟斤拷 yaw 锟角ｏ拷deg锟斤拷
 ********************************************************************************************************************/
void set_desired_yaw(float yaw_deg)
{
	desired_yaw  = normalize_angle_diff(yaw_deg);
	yaw_setpoint = icm42688_yaw;  // 锟接碉拷前锟斤拷实锟斤拷锟斤拷始平锟斤拷锟斤拷锟斤拷
}


/*********************************************************************************************************************
 * @brief 锟斤拷锟斤拷锟斤拷息锟斤拷锟�
 * @param none
 * @example
 * @date 2026.1.22
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/
void display_pid(void)
{
	printf("%f\n",u);
}

#define SPEED_BASE_DEFAULT_RPM      (1800.0f)
#define SPEED_CMD_MIN_RPM           (0.0f)
#define SPEED_CMD_MAX_RPM           (4200.0f)

static float speed_tilt_comp_piecewise(float tilt_abs_deg)
{
	if (tilt_abs_deg < 4.0f)
	{
		return 0.0f;
	}
	if (tilt_abs_deg < 8.0f)
	{
		return (tilt_abs_deg - 4.0f) * 2.f;
	}
	if (tilt_abs_deg < 15.0f)
	{
		return 16.0f + (tilt_abs_deg - 8.0f) * 4.0f;
	}

	tilt_abs_deg = clampf(tilt_abs_deg, 15.0f, 27.0f);
	return 36.0f + (tilt_abs_deg - 15.0f) * 6.0f;
}

/*********************************************************************************************************************
 * @brief 锟劫度匡拷锟狡猴拷锟斤拷
 * @param none
 * @example
 * @date 2026.1.22
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/
float now_speed=0.f;
void speed_control(void)
{
	if(!stop_flag)
	{
//	    if( fabs(icm42688_roll)>15 && fabs(icm42688_roll)<45)
//	    {
//
//	        motor_set_speed(Ctrl_lastSpeed*0.9+ + fabs(icm42688_roll)*+60);
//
//	    }else{

	        motor_set_speed(Ctrl_lastSpeed + fabs(icm42688_roll)*40);//60

	//7    }

	}
	else
	{
		motor_set_speed(0);
	}
}
