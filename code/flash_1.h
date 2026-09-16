/*
 * flash_1.h
 *
 *  Created on: 2026年1月16日
 *      Author: MR
 */
#include "zf_common_headfile.h"
#ifndef CODE_FLASH_1_H_
#define CODE_FLASH_1_H_

//存储数据扇页
//#define flash_1_example 0
#define GPS_PAGE_INDEX 1
#define GPS_NUMBER_PAGE 2
#define GPS_MAX_POINTS 256
// PID ���� Flash �洢
#define PARAM_PAGE_NUM  (10)

// PID�������ýṹ��
typedef struct
{
    float rate_kp;       // Rate_Servo    P (ָ��: RKP)
    float rate_ki;       // Rate_Servo    I (ָ��: RKI)
    float rate_kd;       // Rate_Servo    D (ָ��: RKD)

    float angle_kp;      // RollAnglePID  P (ָ��: AKP)
    float angle_ki;      // RollAnglePID  I (ָ��: AKI)
    float angle_kd;      // RollAnglePID  D (ָ��: AKD)

    float kp;            // AngleServoPID P (ָ��: KP)
    float kd;            // AngleServoPID D (ָ��: KD)
    float ki;            // AngleServoPID I (ָ��: KI)
    float yp;            // YawPID        P (ָ��: YP)

    float speed_kp;      // speed_pid     P (ָ��: SKP)
    float speed_ki;      // speed_pid     I (ָ��: SKI)
    float speed_kd;      // speed_pid     D (ָ��: SKD)

    float JJ_POINT;

    int32 target_speed;  // Ŀ���ٶ� (ָ��: SPD)
} pid_params_t;

extern volatile pid_params_t g_params;
extern const pid_params_t DEFAULT_PARAMS;

void Param_Init(void);
void Param_SaveNow(void);
void Param_ApplyToPID(void);

#endif /* CODE_FLASH_1_H_ */
