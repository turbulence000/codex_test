/*
 * Ctrl.c
 *
 *  Created on: 2026年3月11日
 *      Author: Lin
 */
#include "zf_common_headfile.h"

void Send_channel_Data(void){

    if(1 == uart_receiver.finsh_flag)                            // 帧完成标志判断
    {
        if(1 == uart_receiver.state)                             // 遥控器失控状态判断
        {
            printf("CH1-CH6 data: ");
            for(int i = 0; i < 6; i++)
            {
                printf("%d ", uart_receiver.channel[i]);         // 串口输出6个通道数据
            }
            printf("\r\n");
        }
        else
        {
            printf("Remote control has been disconnected.\r\n"); // 串口输出失控提示
        }
        uart_receiver.finsh_flag = 0;                            // 帧完成标志复位

    }


}


static float normalize_angle_180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}



float Ctrl_lastSpeed = 2000;

void Uart_Car_Ctrl(void){



    if(1 == uart_receiver.finsh_flag)                            // 帧完成标志判断
        {
            if(1 == uart_receiver.state)                             // 遥控器失控状态判断
            {


                    // ==========================================
                    // 通道一：方向控制
                    // ==========================================
                    // Ctrl.c 通道一部分替换为：
                    


                    uint16_t dir_ch = uart_receiver.channel[0];

                    // 限幅
                    if(dir_ch < 200)  dir_ch = 200;
                    if(dir_ch > 1800) dir_ch = 1800;

                    // 归一化到 [-1, +1]，中�?1000
                    float dir_norm = ((float)dir_ch - 1000.0f) / 800.0f;

                    // 死区：摇杆中位抖动不累加
                    if(fabsf(dir_norm) < 0.1f) dir_norm = 0.0f;

                    // 累加目标航向（主循环调用，速率约几十Hz，步长可调）
                    desired_yaw += dir_norm * 1.f;   //
                    desired_yaw = normalize_angle_180(desired_yaw);


                    // ==========================================
                    // 通道二：速度控制
                    // ==========================================

                    //如果改变映射也要同步改变
//                    static float last_target_speed = 2800.0f; // 记录上一次的速度，初始值为2800
//
//                    //读取通道1的数据
//                    uint16_t speed_ch_data = uart_receiver.channel[1];
//
//                    //极值限幅保护
//                    if (speed_ch_data < 200) {
//                        speed_ch_data = 200;
//                    } else if (speed_ch_data > 1800) {
//                        speed_ch_data = 1800;
//                    }
//
//                    // 线性映射计算目标速度
//                    last_target_speed = 1.5f * (float)speed_ch_data + 1300.0f;
//
//                    Ctrl_lastSpeed = last_target_speed;


                    // ==========================================
                    // 通道三：速度清除
                    // ==========================================

                    if(uart_receiver.channel[2]==1792){

                        gpio_set_level(P33_10,1);

                        stop_flag = 1;

                        }else{

                        gpio_set_level(P33_10,0);
                    }

                    // ==========================================
                    // 通道四：速度变换 192 992 1792
                    // ==========================================
//                    if(uart_receiver.channel[3]==1792){
//                        Ctrl_lastSpeed = 3200.f;
//
//                    }else if(uart_receiver.channel[3]==992){
//                        Ctrl_lastSpeed = 2400.f;
//
//                    }else if(uart_receiver.channel[3]==192){
//                        Ctrl_lastSpeed = 2000.f;
//
//                    }
//




            }
            else
            {
                //printf("Remote control has been disconnected.\r\n"); // 串口输出失控提示
            }
            uart_receiver.finsh_flag = 0;                            // 帧完成标志复位

        }









}


























