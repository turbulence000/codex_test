/*
 * Init.c
 *
 *  Created on: 2025闁跨喐鏋婚幏锟�??10闁跨喐鏋婚幏锟�??3闁跨喐鏋婚幏锟�??
 *      Author: MR
 */


#include "zf_common_headfile.h"

// 直走 2 米 → 右转 → 再走 1 米
route_pt_t demo[] = {
    { 0,  0},   // 起点
    { 0,  5},   // 前 100cm（5格 × 20cm）
//    { 0, 10},   // 再前 100cm
//    { 5, 10},   // 右转走 100cm
};

void Init(void)
{

    ips200_init(IPS200_TYPE_SPI);



    gps_init();
    gnss_init(1);


//    clear_navigation();
//    Init_Nag();
    key_init(10);
    Encoder_Init();
    //route_set_path(demo, 2);   // 4 个点，自动存 Flash
//    route_init();


    adc_init(ADC0_CH11_A11, ADC_12BIT);


    Param_Init();
    steering_engine_Init();
    Init_ICM42688();
    Filter_Init();
    Gyro_Calibration();



    set_roll_ref_deg(1.68f);


    Beep_Init();

    small_driver_uart_init();

//    pid_init_2(&Rate_Servo, 0.11, 0.0, 0.005, -45, 45, 0.0, 0.0);
//    pid_init_2(&RollAnglePID, 10.5f, 0.0, 0.15, -800, 800, -50.0, 50.0);//100.2
//
//    pid_init_2(&YawPID, -0.4f, 0.0f, 0.0f, -400.0f, 400.0f, 0.0f, 0.0f);  //-0.45
    pid_init_2(&Rate_Servo, 0.11, 0.0, 0.00, -100, 100, 0.0, 0.0);
    pid_init_2(&RollAnglePID, 10.4f, 0.0, 0.15, -800, 800, -50.0, 50.0);//100.2


    pid_init_2(&AngleServoPID,11.5f, 0.f, 0.0f,-70.0f, 70.0f, -40.0f, 40.0f);// 11.7
    pid_init_2(&YawPID, -0.45f, 0.0f, 0.0f, -400.0f, 400.0f, 0.0f, 0.0f);  //-0.45


    motor_filter_init();

    pid_motor_speed_init(&speed_pid,1.5,0.15,0.00,0,10000,0,6000);//0.895


    set_balance_control_mode(0);

    //uart_receiver_init();

    wireless_uart_init();
    Param_ApplyToPID();


}

