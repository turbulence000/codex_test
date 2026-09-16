#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

// **************************** 主函数 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认的调试串口
    // 此处书写用户代码 初始化外设等
    Init();                         // 你的原有初始化

    pit_ms_init(CCU60_CH0,10);       // 1ms 定时中断 (直立环、速度环)
    pit_ms_init(CCU61_CH0,5);       // 5ms 定时中断 (姿态解算、转向环)
    pit_ms_init(CCU60_CH1,5);
    pit_us_init(CCU61_CH1,5000);

    cpu_wait_event_ready();         // 等待所有核心初始化完毕

    while (TRUE)
    {
        //Uart_Car_Ctrl();
        //Send_channel_Data();
       //Print_ICM42688_Data();
        //route_print_info();
        //ips200_show_int(8*sizeof("g_route.running"),100,g_route.running,4);


    }
}

// **************************** 中断服务函数 ****************************


IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

    key_scanner();
    //key_route();

    motor_speed_calculate();                        // 速度计算
    //route_step(diff_navigation);

    motor_speed_control_step();                     // 速度控制步进

    Protect_roll();                                 // 倾角保护
    speed_control();


}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);

    balance_control_update(0.005);


}

IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{

    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);

    steering_control_update(0.005f);                // 转向环
}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);

    Control();



}




#pragma section all restore
