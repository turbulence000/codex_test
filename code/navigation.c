/*
 * nagivation.c
 *
 *  Created on: 2024-10-16
 *      Author: Monst
 */

#include "zf_common_headfile.h"
#include "navigation.h"

Nag N;

static float normalize_angle_180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 功能说明：导航记录状态机
// 处理流程：根据 N.End_f 执行保存、写入 Flash 和结束流程
//-------------------------------------------------------------------------------------------------------------------
void Nag_Read()
{
        switch(N.End_f)
        {
            case 0:Run_Nag_Save();  // 保存当前路程对应的航向点
                break;
            case 1:;
                    flash_Nag_Write();  // 将缓存航向点写入 Flash
                    N.End_f++;
                    break;
            case 2://Buzzer_check(500);   // 可在此处添加结束提示音
                    N.End_f++;  // 导航记录流程结束
                    break;
        }
}

//-------------------------------------------------------------------------------------------------------------------
// 功能说明：导航运行主逻辑
// 处理流程：读取目标角度并计算当前偏差输出到 N.Final_Out
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run()
{
  if(N.Nag_Stop_f){
    N.Final_Out=0;
    return;
  }
    Run_Nag_GPS();  // 获取目标航向角

        // Wrap error into [-180, 180] to avoid sudden large steering at yaw boundary.
        N.Final_Out=normalize_angle_180(N.Angle_Run-Nag_Yaw);// 计算目标角与当前航向角偏差

}

//-------------------------------------------------------------------------------------------------------------------
// 功能说明：按里程间隔保存导航点
// 处理流程：累计里程，达到阈值后将当前航向角存入缓存
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_Save()
{
       // 累计本周期里程增量
    if(N.size > MaxSize)// 缓存写满后写入 Flash 并切换页
    {
        flash_Nag_Write();
        N.size=0;   // 缓存索引清零
        N.Flash_page_index++;   // Flash 页号加一
        zf_assert(N.Flash_page_index < Nag_End_Page);// 防止越界
    }

    while(N.Mileage_All >= Nag_Set_mileage)    // 达到采样里程阈值（while 防止高速丢点）
    {
       int32 Save=(int32)(Nag_Yaw*100); // 航向角放大 100 倍转为整型保存
       flash_union_buffer[N.size++].int32_type = Save;  // 压入缓存

       N.Save_index++;

       if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;// 扣除一个采样步长
       else N.Mileage_All += Nag_Set_mileage;
    }

}


//-------------------------------------------------------------------------------------------------------------------
// 功能说明：按里程间隔读取导航点
// 处理流程：按顺序从 Flash 取出目标航向角到 N.Angle_Run
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_GPS(void)
{

    if(N.Flash_read_f == 0)// 首次进入时先读取一页 Flash 到缓存
    {
        N.size=0;
        flash_Nag_Read();   // 读取 Flash 数据
        N.Flash_read_f=1;

    }

    if(N.size > MaxSize)    // 当前页缓存读完后切到下一页
    {
        N.Flash_read_f=0;   // 触发下次重新读取 Flash
        N.Flash_page_index++;   // 切换页号

        zf_assert(N.Flash_page_index < Nag_End_Page);
    }


    while(N.Mileage_All >= Nag_Set_mileage)
    {
        // 达到里程阈值后取下一个目标点（while 防止高速时索引滞后）
      N.Save_count++;
    if(N.Save_count > N.Save_index-2)
    {
        N.Nag_Stop_f++;

        return;
    }

       N.Angle_Run = (flash_union_buffer[N.size++].int32_type / 100.0f);

         if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;// 扣除一个采样步长
         else N.Mileage_All += Nag_Set_mileage;
    }


}
//-------------------------------------------------------------------------------------------------------------------
// 功能说明：初始化导航状态
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag(void)
{
    memset(&N, 0, sizeof(N));
    N.Flash_page_index=Nag_Start_Page;
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 功能说明：导航系统任务入口
// 参数说明：
// index: N.Nag_SystemRun_Index，1 为记录，2 为回放
//-------------------------------------------------------------------------------------------------------------------
void Nag_System(void){
    // 未启动或已停止时直接返回
    if(!N.Nag_SystemRun_Index || N.Nag_Stop_f )  return;

    switch(N.Nag_SystemRun_Index)
    {
        case 1 : Nag_Read();    // 记录模式
            break;
        case 2 : Nag_Run();     // 回放模式
            break;

    }
}

//-------------------------------------------------------------------------------------------------------------------
// 功能说明：编码器里程测试
//-------------------------------------------------------------------------------------------------------------------
void encoder_test(void)
{
    N.Mileage_All += diff_navigation;
    if(N.Mileage_All > Nag_Set_mileage)
    {
       N.Save_count++;
       if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;// 扣除一个采样步长

    }

}

//-------------------------------------------------------------------------------------------------------------------
// 功能说明：导航调试数据显示
//-------------------------------------------------------------------------------------------------------------------
void Print_Navigation_Data(void)
{
////        // 编码器调试显示
//        ips200_show_string(0,0,"encoder:");
//        ips200_show_int(50, 0,  Encoder_count(TIM2_ENCODER), 5);
//
//        // Yaw 调试显示
//        ips200_show_string(0,20,"Mileage");
//        ips200_show_int(50, 20, N.Mileage_All, 5);   // x=0, y=100
////        // 计数调试显示
//        ips200_show_string(0,40,"count:");
//        ips200_show_int(50, 40, N.Save_count, 4);

        ips200_show_string(0,20*1,"N.Mileage_All"); ips200_show_float(8*sizeof("N.Mileage_All"),20*1,N.Mileage_All,4,4);
        ips200_show_string(0,20*2,"N.Nag_SystemRun_Index");ips200_show_int(8*sizeof("N.Nag_SystemRun_Index"),20*2,N.Nag_SystemRun_Index,4);
        ips200_show_string(0,20*3,"diff_navigation");     ips200_show_int(8*sizeof("diff_navigation"),20*3,diff_navigation ,4);
        ips200_show_string(0,20*4,"N.Angle_Run"); ips200_show_float(8*sizeof("N.Angle_Run"),20*4,N.Angle_Run,4,4);
        ips200_show_string(0,20*5,"N.Final_Out"); ips200_show_float(8*sizeof("N.Final_Out"),20*5,N.Final_Out,4,4);
        ips200_show_string(0,20*6,"N.End_f");   ips200_show_float(8*sizeof("N.End_f"),20*6,N.End_f,4,4);
        ips200_show_string(0,20*7,"Save_index"); ips200_show_int(8*sizeof("Save_index"),20*7,N.Save_index,4);
        ips200_show_string(0,20*8,"yaw");
        ips200_show_float(50, 20*8, icm42688_yaw, 6, 2);   // x=0, y=100
        //ips200_show_string(0,20*8,"N.Angle_Run"); ips200_show_float(8*sizeof("N.Angle_Run"),20*8,N.Angle_Run,4,4);

        //ips200_show_string(0,20*9,"info");ips200_show_int(8*sizeof("info"),20*9, info ,4);

//        ips200_show_string(0,60,"icm42688_gyro_x:");
//        ips200_show_float(140,60,icm42688_gyro_x,2,3);
//        ips200_show_string(0,80,"icm42688_gyro_y:");
//        ips200_show_float(140,80,icm42688_gyro_y,2,3);
//        ips200_show_string(0,100,"icm42688_gyro_z:");
//        ips200_show_float(140,100,icm42688_gyro_z,2,3);
//        ips200_show_string(0,120,"steering_angle:");
//        ips200_show_float(140,120,u,2,3);

        //printf("info:%d\n",info);
         //printf("yaw:%.2f\n",icm42688_roll);
        //printf("icm42688_gyro_x:%.2f\n",icm42688_gyro_x);

}
uint8_t info = 0;
void key_navigation(void)
{

      info = key_get_short_press();
      if(info==4)
      {
          stop_flag = 0;
          //Beep_navigation_mode = Beep_on;
      }
      if(info==3)
      {
          N.End_f=1;
         // Beep_navigation_mode = Beep_on;

      }
      else if(info==1)
      {
          N.Nag_SystemRun_Index=1;
         // Beep_navigation_mode = Beep_on;
      }
      else if(info==2)
      {
          N.Nag_SystemRun_Index=2;
          N.Nag_Stop_f = 0;
          N.Flash_page_index = Nag_Start_Page;
          N.Flash_read_f = 0;
          N.size = 0;
          N.Save_count = 0;
          N.Mileage_All = 0;
          N.Angle_Run = Nag_Yaw;
          //Beep_navigation_mode = Beep_on;
      }
      else if(info==5)    // 按键5：清除 Flash 导航数据
      {
          clear_navigation();       // 擦除 Flash 第3~10页
          Init_Nag();               // 重置 RAM 中的导航状态
           N.Nag_SystemRun_Index = 0;// 退出录/放模式
      }

}

//-------------------------------------------------------------------------------------------------------------------
// 功能说明：清空导航使用的 Flash 页
//-------------------------------------------------------------------------------------------------------------------
void clear_navigation(void)
{
    for(int i=3;i<11;i++)
    {
        flash_erase_page(0,i);
    }
    return;
}
