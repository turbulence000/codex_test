/*
 * flash_1.c
 *
 *  Created on: 2026楠烇拷1閺堬拷16閺冿拷
 *      Author: MR
 */
#include "zf_common_headfile.h"
#include "Control.h"

volatile pid_params_t g_params;

// 姒涙顓婚崣鍌涙殶閿涘牅绗傞悽鍨閼伙拷 Flash 閺冪姵鏅ラ崚娆戞暏濮濄倕锟界》绱�
const pid_params_t DEFAULT_PARAMS = {
    .rate_kp      = 0.11f,
    .rate_ki      = 0.0f,
    .rate_kd      = 0.0f,

    .angle_kp     = 10.4f,
    .angle_ki     = 0.0f,
    .angle_kd     = 0.0f,

    .kp           = 13.5f,
    .kd           = 0.0f,
    .ki           = 0.0f,


    .yp           = -0.15f,

    .speed_kp     = 2.f,
    .speed_ki     = 0.3f,
    .speed_kd     = 0.0f,

    .target_speed = 2500.f,


};

#define PARAM_WORD_LEN  ((sizeof(pid_params_t) + 3) / 4)


/**
 * @brief 閸欏倹鏆熼崚婵嗩潗閸栨牭绱版稉濠勬暩閺冩湹绱崗鍫滅矤 Flash 閸旂姾娴囬敍灞炬￥閺佸牆鍨悽锟� DEFAULT_PARAMS
 */
void Param_Init(void)
{
    g_params = DEFAULT_PARAMS;
    pid_params_t saved;
    flash_read_page(0, PARAM_PAGE_NUM, (uint32*)&saved, PARAM_WORD_LEN);
    if (saved.kp >= 0.1f && saved.kp <= 100.0f
        && saved.rate_kp >= 0.01f && saved.rate_kp <= 10.0f) {
        g_params = saved;
    }
}

/**
 * @brief 缁斿宓嗙亸锟� g_params 閸愭瑥鍙� Flash
 */
void Param_SaveNow(void)
{
    flash_write_page(0, PARAM_PAGE_NUM, (const uint32*)&g_params, PARAM_WORD_LEN);
}

/**
 * @brief 鐏忥拷 g_params 閸氬本顒為崘娆忓弳瑜版挸澧犻悽鐔告櫏閻拷 PID 閹貉冨煑閸ｏ拷
 */
void Param_ApplyToPID(void)
{
    AngleServoPID.kp = g_params.kp;
    AngleServoPID.kd = g_params.kd;
    AngleServoPID.ki = g_params.ki;
    YawPID.kp = g_params.yp;

    // 鍙岀幆妯″紡鍙傛暟锛堜粎 BALANCE_MODE_RATE_ANGLE_STEERING 鏃剁敓鏁堬級
    Rate_Servo.kp   = g_params.rate_kp;
    Rate_Servo.ki   = g_params.rate_ki;
    Rate_Servo.kd   = g_params.rate_kd;
    RollAnglePID.kp = g_params.angle_kp;
    RollAnglePID.ki = g_params.angle_ki;
    RollAnglePID.kd = g_params.angle_kd;

    // 閫熷害 PID 鍙傛暟
    speed_pid.kp = g_params.speed_kp;
    speed_pid.ki = g_params.speed_ki;
    speed_pid.kd = g_params.speed_kd;



    // 鐩爣閫熷害鍚屾鍒版帶鍒跺彉閲�
    Ctrl_lastSpeed = (float)g_params.target_speed;
}
