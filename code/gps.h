/*
 * gps.h
 *
 *  Created on: 2026锟斤拷1锟斤拷16锟斤拷
 *      Author: MR
 */
#include "zf_common_headfile.h"
#ifndef CODE_GPS_H_
#define CODE_GPS_H_


extern volatile uint8_t auto_follow_key_event;
extern double real_time_latitude;  // 实时纬锟斤拷
extern double real_time_longitude; // 实时锟斤拷锟斤拷
extern double target_point[2][128]; //读取经度纬度的数组
extern uint8_t auto_drive_flag;  // 自动驾驶标志位 (0:停车, 1:跑图)
extern volatile uint8_t  InterrupFlag;
extern volatile uint8_t  count;

void gps_get_real_state(void);
void gps_init(void);
void gps_prase(void);
void StoreGpsToFlash(void);
void StoreGPSPointNumberToFlash(void);
void GetGpsFromFlash(void);
void key_flash_gps(void);



void Auto_Follow(void);
void Ips_ShowData_GPS(void);






#endif /* CODE_GPS_H_ */
