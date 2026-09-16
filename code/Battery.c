/*
 * Battery.c
 *
 *  Created on: 2025年10月19日
 *      Author: MR
 */

#include "zf_common_headfile.h"

float voltage = 100;

//蜂鸣器初始化
void Beep_Init(void){

    gpio_init(P33_10,GPO,0,GPO_PUSH_PULL);
}


// 读取电池电压的函数
void read_battery_voltage(adc_channel_enum battery_channel) {

    // 读取 ADC 值，可以选择直接读取或进行均值滤波
    uint16 adc_value = adc_mean_filter_convert(battery_channel, 10); // 采集 10 次

    // 将 ADC 值转换为电压
    voltage = (adc_value*35.14) / 4095.0; // 12 位分辨率最大值为 4095

    //电压检测5
//    if(voltage<11.6){
//        gpio_set_level(P33_10,1);
//    }
//    else{
//        gpio_set_level(P33_10,0);
//    }

//    printf("%.2f\n",voltage);
}
