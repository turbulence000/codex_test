#include "zf_common_headfile.h"
/*********************************************************************************************************************
 * @brief 显示gps信息函数
 * @param none
 * @example gps_prase(1)
 * @date 2026.1.17
 * @remark 显示gps信息到ips200屏幕上 便于观察与调试
 * @remark
 * @remark
 ********************************************************************************************************************/
// 作用: 解析并刷新GNSS数据。
void gps_prase(void)
{

    if(gnss_flag)
        {
            gnss_flag = 0;
            gnss_data_parse();
        }

}


double real_time_latitude = 0;  // 实时纬度
double real_time_longitude = 0; // 实时经度

// 踩点均值滤波窗口，窗口越大越平滑但响应越慢。
#define GPS_POINT_FILTER_WINDOW   (3u)

static double gps_lat_hist[GPS_POINT_FILTER_WINDOW] = {0};
static double gps_lon_hist[GPS_POINT_FILTER_WINDOW] = {0};
static double gps_lat_sum = 0.0;
static double gps_lon_sum = 0.0;
static uint32 gps_filter_idx = 0;
static uint32 gps_filter_count = 0;

static void gps_point_filter_reset(void)
{
    for (uint32 i = 0; i < GPS_POINT_FILTER_WINDOW; i++)
    {
        gps_lat_hist[i] = 0.0;
        gps_lon_hist[i] = 0.0;
    }

    gps_lat_sum = 0.0;
    gps_lon_sum = 0.0;
    gps_filter_idx = 0;
    gps_filter_count = 0;
}

static void gps_point_filter_push(double lat, double lon)
{
    if (gps_filter_count < GPS_POINT_FILTER_WINDOW)
    {
        gps_lat_hist[gps_filter_idx] = lat;
        gps_lon_hist[gps_filter_idx] = lon;
        gps_lat_sum += lat;
        gps_lon_sum += lon;
        gps_filter_count++;
    }
    else
    {
        gps_lat_sum -= gps_lat_hist[gps_filter_idx];
        gps_lon_sum -= gps_lon_hist[gps_filter_idx];

        gps_lat_hist[gps_filter_idx] = lat;
        gps_lon_hist[gps_filter_idx] = lon;

        gps_lat_sum += lat;
        gps_lon_sum += lon;
    }

    gps_filter_idx++;
    if (gps_filter_idx >= GPS_POINT_FILTER_WINDOW)
    {
        gps_filter_idx = 0;
    }
}

static void gps_point_filter_get(double *lat, double *lon)
{
    if (gps_filter_count == 0)
    {
        *lat = real_time_latitude;
        *lon = real_time_longitude;
        return;
    }

    *lat = gps_lat_sum / (double)gps_filter_count;
    *lon = gps_lon_sum / (double)gps_filter_count;
}
/*********************************************************************************************************************
 * @brief
 * @param none
 * @example
 * @date 2026.1.17
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/
// 作用: 将当前GNSS经纬度同步到实时变量。
void gps_get_real_state(void)       //简易实时更新维度经度函数
{
    real_time_latitude = gnss.latitude;
    real_time_longitude = gnss.longitude;

    gps_point_filter_push(real_time_latitude, real_time_longitude);

}

//-------------------------------------------以下为Flash存点的模块-------------------------------------------//
uint32 current_point_number_flash = 0; // 用flash存gps点位，变量表示目前采集点的数量
double flash_read_data1 = 0;           // GetGpsFromFlash函数使用
uint32 *save_pointer1;
double flash_read_data2 = 0; // GetGpsFromFlash函数使用
uint32 *save_pointer2;
/*********************************************************************************************************************
 * @brief 利用 Flash 进行gps存点(指针地址存点，最大程度保证精度)
 * @param none
 * @example
 * @date 2026.1.17
 * @remark 将采集的点位记录到缓冲区并储存至flash的GPS_PAGE_INDEX扇页  在菜单里面使用
 * @remark  uint32_type 和 float_type 都是4字节  逐飞定义的缓冲区存储容量和每一页的存储容量一样大
 * @remark  在逐飞的flash.h文件里面写了每一页最多存储512个uint32，按照本函数的存法，最多存512/4=128个点位
 ********************************************************************************************************************/
// 作用: 将当前实时经纬度按双uint32格式写入Flash点位页。
void StoreGpsToFlash(void)
{
    double filtered_lat = 0.0;
    double filtered_lon = 0.0;

    save_pointer1 = NULL;
    save_pointer2 = NULL; // 调用之前先清空指针

    gps_point_filter_get(&filtered_lat, &filtered_lon);

    flash_buffer_clear();

    flash_read_page_to_buffer(0, GPS_PAGE_INDEX);
    uint32 off = current_point_number_flash * 4;


    save_pointer1 = (uint32 *)&filtered_lat;
    flash_union_buffer[off + 0].uint32_type = *save_pointer1;
    flash_union_buffer[off + 1].uint32_type = *(++save_pointer1);

    // ①获取 real_time_longitude 变量的内存地址。②将这个地址转换为 uint32 类型的指针。
    // ③将转换后的指针赋值给 save_pointer2变量
    save_pointer2 = (uint32 *)&filtered_lon;
    flash_union_buffer[off + 2].uint32_type = *save_pointer2;
    flash_union_buffer[off + 3].uint32_type = *(++save_pointer2);

    current_point_number_flash++;
    flash_write_page_from_buffer(0, GPS_PAGE_INDEX);
    StoreGPSPointNumberToFlash();
    //printf("\r\n第%d个点存入成功\n", current_point_number_flash);

}

/*********************************************************************************************************************
 * @brief  Flash 存储已经存储点位的数量
 * @param none
 * @example StoreGPSPointNumberToFlash();
 * @date 2026.1.17
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/
// 作用: 将当前已存点数量写入Flash计数页。
void StoreGPSPointNumberToFlash(void)
{
    flash_buffer_clear();
    if (flash_check(0, GPS_NUMBER_PAGE) == 1)
    {
        flash_erase_page(0,GPS_NUMBER_PAGE);   // 清除flash扇页之后，确保在新的一组点位存储过程中从头开始
    }
    flash_union_buffer[0].uint32_type = current_point_number_flash;
    flash_write_page_from_buffer(0, GPS_NUMBER_PAGE);
}
/*********************************************************************************************************************
 * @brief 从Flash读取已经存储点位的信息
 * @param none
 * @example GetGpsFromFlash();
 * @date 2026.1.17
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/
double target_point[2][128] = {{0}}; // 用于储存采集的点位，第一维 0：纬度 1：经度，第二维：总共150个数据
// 作用: 从Flash读取全部已存点到target_point数组。
void GetGpsFromFlash(void)
{

    if(flash_check(0,GPS_PAGE_INDEX))
    {
        uint32 j = 0 ;
        flash_buffer_clear();
        flash_read_page_to_buffer(0, GPS_PAGE_INDEX);
        for(j=0;j<current_point_number_flash;j++)
        {
            //取出第i个点位的纬度
            save_pointer1 = (uint32 *)&flash_read_data1;
            *save_pointer1 = flash_union_buffer[j*4+0].uint32_type;
            *++save_pointer1 = flash_union_buffer[j*4+1].uint32_type;

            //取出第i个点位的经度
            save_pointer2 = (uint32 *)&flash_read_data2;
            *save_pointer2 = flash_union_buffer[j*4+2].uint32_type;
            *++save_pointer2 = flash_union_buffer[j*4+3].uint32_type;

            target_point[0][j] = flash_read_data1;
            target_point[1][j] = flash_read_data2;

            //printf("第%d个点的纬度为%.6f，经度为%.6f\r\n",j+1,target_point[0][j],target_point[1][j]);
        }
    }
    else
    {
        //printf("no save information\n");
    }

}

// 作用: 上电初始化已存点数量。
void gps_init(void)
{
    current_point_number_flash = 0;
    gps_point_filter_reset();

    if (flash_check(0, GPS_NUMBER_PAGE))
    {
        flash_buffer_clear();
        flash_read_page_to_buffer(0, GPS_NUMBER_PAGE);

        uint32 n = flash_union_buffer[0].uint32_type;
        if (n <= GPS_MAX_POINTS) current_point_number_flash = n;
        else current_point_number_flash = 0; // 异常保护
    }
}

// 作用: 清空GPS点位及计数。
void gps_clear(void)
{
    current_point_number_flash = 0;
    gps_point_filter_reset();
    for(int i=0;i<4;i++)
    {
        flash_erase_page(0,i);
    }
    //printf("gps info clear\n");
}



// 作用: 简易按键流程(存点/读点/清空)。
void key_flash_gps(void)       // 简易按键存储读取
{
    uint8_t info=key_get_short_press();
    if(info==1)
    {
       // flash_erase_page(0,GPS_PAGE_INDEX);
       // printf("999");
        StoreGpsToFlash();
    }
    else if(info==2)
    {
        //printf("777");
        GetGpsFromFlash();
    }
    else if(info==3)
    {
        gps_clear();
    }
}



// 将任意角度限制在 0~360 度之间  辅助函数
// 作用: 角度归一化到[0, 360)。
float NormalizeAngle(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f)    angle += 360.0f;
    return angle;
}

// 将角度规范到 [-180, 180]
// 作用: 角度归一化到[-180, 180]。
static float NormalizeAngleSigned(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// 动态切点半径参数：r = base + gain * speed(m/s)，并做上下限保护。
#define GPS_SWITCH_BASE_M          (0.8f)
#define GPS_SWITCH_SPEED_GAIN      (0.35f)
#define GPS_SWITCH_MIN_M           (0.8f)
#define GPS_SWITCH_MAX_M           (2.2f)

// desired_yaw 每次循环最大变化量（度），用于抑制目标航向突变。
#define GPS_YAW_STEP_MAX_DEG       (3.0f)

// ------------------------- 调参指引 -------------------------
// GPS_SWITCH_BASE_M:
//   增大 -> 低速更早切点，转弯更柔和，但可能“抄近路”。
//   减小 -> 更贴近目标点再切点，路径更准，但可能出现晚切与摆动。
//
// GPS_SWITCH_SPEED_GAIN:
//   增大 -> 速度上来后切点半径增长更快，适合高速防冲点。
//   减小 -> 高速仍较保守，路径更贴点，但高速场景更易来不及转向。
//
// GPS_SWITCH_MIN_M / GPS_SWITCH_MAX_M:
//   作为总保险，先调 base/gain，再用 min/max 限住极端工况。
//   常见做法：先把 max 放大到 2.5 左右试车，再收回到稳定值。
//
// GPS_YAW_STEP_MAX_DEG:
//   增大 -> 转向响应更快，但舵机会更“冲”，可能抖动加重。
//   减小 -> 转向更平滑，但连续急弯可能跟不上。
//
// 推荐调参顺序：
//   1) 先定 GPS_YAW_STEP_MAX_DEG（先稳住舵机）
//   2) 再调 BASE + GAIN（解决早切/晚切）
//   3) 最后用 MIN/MAX 做边界收敛
// -----------------------------------------------------------

static uint8_t desired_yaw_slew_inited = 0;
static float desired_yaw_slew_last = 0.0f;

static float GetDynamicSwitchRadius(void)
{
    // GNSS速度单位是 km/h，这里先换算成 m/s。
    float speed_mps = gnss.speed / 3.6f;
    float r = GPS_SWITCH_BASE_M + GPS_SWITCH_SPEED_GAIN * speed_mps;
    return clampf(r, GPS_SWITCH_MIN_M, GPS_SWITCH_MAX_M);
}

static float SlewDesiredYaw(float target_yaw)
{
    if (!desired_yaw_slew_inited)
    {
        // 首次进入直接对齐目标，避免上电/启动瞬间出现人为滞后。
        desired_yaw_slew_last = target_yaw;
        desired_yaw_slew_inited = 1;
        return desired_yaw_slew_last;
    }

    // 使用[-180, 180]最短角差，再做每周期限速。
    float yaw_diff = NormalizeAngleSigned(target_yaw - desired_yaw_slew_last);
    yaw_diff = clampf(yaw_diff, -GPS_YAW_STEP_MAX_DEG, GPS_YAW_STEP_MAX_DEG);
    desired_yaw_slew_last = NormalizeAngleSigned(desired_yaw_slew_last + yaw_diff);
    return desired_yaw_slew_last;
}



// 自动循迹核心控制逻辑
uint8_t auto_drive_flag = 0;  // 自动驾驶标志位 (0:停车, 1:跑图)
double new_azimuth = 0;       // 目标方位角
double new_distance = 0;      // 距离目标点的距离
uint8_t next_point = 0;       // 当前正在前往的目标点序号
float target_yaw_imu = 0.0f;     // 转换到IMU坐标系的目标航向

volatile uint8_t auto_follow_key_event = 0; // 中断置位，主循环消费

static uint8_t auto_store_enable = 0;        // 0.3秒自动存点使能


// 运行时点集：第一个点作为发车基准，后续点叠加静态漂移补偿
#define GPS_RUNTIME_POINT_CAPACITY     128u

static double target_point_runtime[2][128] = {{0}};
static uint32 runtime_point_count = 0;
static double gps_static_drift_lat = 0.0;
static double gps_static_drift_lon = 0.0;


// 作用: 将静态存点转换为运行时点并叠加起点漂移补偿。
static void Build_RuntimeERR(void)
{
    uint32 j = 0;

    runtime_point_count = 0;
    gps_static_drift_lat = 0.0;
    gps_static_drift_lon = 0.0;

    if (current_point_number_flash == 0)
    {
        return;
    }

    // 静态漂移 = 当前实时位置 - 第一个存点（发车点）
    gps_static_drift_lat = real_time_latitude - target_point[0][0];
    gps_static_drift_lon = real_time_longitude - target_point[1][0];

    // 第一个运行点固定为当前发车位置，后续点做漂移补偿
    target_point_runtime[0][0] = real_time_latitude;
    target_point_runtime[1][0] = real_time_longitude;

    for (j = 1; j < current_point_number_flash; j++)
    {
        target_point_runtime[0][j] = target_point[0][j] + gps_static_drift_lat;
        target_point_runtime[1][j] = target_point[1][j] + gps_static_drift_lon;
    }

    runtime_point_count = current_point_number_flash;
}







// 作用: 自动寻点主流程。
void Auto_Follow(void)
{
    gps_prase();
    gps_get_real_state();

    // ---- 按键处理 ----
    uint8_t key = key_get_short_press();

    if (key != 0)                          // 有按键才处理，没按键跳过
    {
        switch (key)
        {
            case 1:
                StoreGpsToFlash();
                break;
            case 5:
                gps_clear();
                desired_yaw = 0;
                auto_drive_flag = 0;
                next_point = 0;
                runtime_point_count = 0;
                auto_store_enable = 0;
                // 清空流程后重置限速状态，确保下次启动从当前目标重新对齐。
                desired_yaw_slew_inited = 0;
                break;
            case 3:
                stop_flag = 0;
                auto_store_enable = 1;
                break;
            case 4:
                if (auto_drive_flag == 0)
                {
                    stop_flag = 0;
                    GetGpsFromFlash();
                    // 至少要有“发车点 + 1个目标点”才有巡点意义
                    if (current_point_number_flash > 1)
                    {
                        Build_RuntimeERR();
                        // 第一个点为发车点基准，从第二个点开始巡点
                        next_point = 1;
                        set_desired_yaw(icm42688_yaw);
                        // 每次新启动巡点都重置限速内部状态，避免继承上次残留。
                        desired_yaw_slew_inited = 0;
                        auto_drive_flag = 1;
                    }
                    else
                    {
                        auto_drive_flag = 0;
                        next_point = 0;
                        runtime_point_count = 0;
                        // 启动失败（点数不足）也做状态清理。
                        desired_yaw_slew_inited = 0;
                    }
                }
                // 按键4只用于启动巡点；
                break;
        }
    }

    // 开启自动存点后，每0.3秒存一次；循迹过程中不存点
    uint8_t auto_store_active = (auto_store_enable == 1) && (auto_drive_flag == 0) && (stop_flag == 0);
    if (auto_store_active)
    {
            StoreGpsToFlash();
    }

    if (auto_drive_flag == 1)
    {
        // 1. 计算距离
        new_distance = get_two_points_distance(
            real_time_latitude, real_time_longitude,
            target_point_runtime[0][next_point], target_point_runtime[1][next_point]
        );
        // 2. 计算 GPS 绝对目标方位角
        new_azimuth = get_two_points_azimuth(
            real_time_latitude, real_time_longitude,
            target_point_runtime[0][next_point], target_point_runtime[1][next_point]
        );
        // GNSS方位角(顺时针正) -> 车辆yaw符号系(左正右负)
        target_yaw_imu = NormalizeAngleSigned(-(float)new_azimuth);

        // 目标航向进入控制器前先限速，降低急转和抖舵。
        desired_yaw = SlewDesiredYaw(target_yaw_imu);

        // 3. 距离判断与切点逻辑
        // 速度越高，切点半径越大，减少高速冲点/晚切点。
        if (new_distance < GetDynamicSwitchRadius())
        {
            next_point++;
            if (next_point >= runtime_point_count)
            {
                auto_drive_flag = 0; // 跑完所有点，停车
                stop_flag = 1;
                // 结束后重置，便于下一轮巡点无历史状态干扰。
                desired_yaw_slew_inited = 0;

            }

        }
    }
}

























// 作用: 在IPS屏幕显示GPS与循迹调试数据。
void Ips_ShowData_GPS(void)
{
    uint32 show_idx = next_point;
    if (show_idx >= current_point_number_flash)
    {
        show_idx = (current_point_number_flash > 0) ? (current_point_number_flash - 1) : 0;
    }

    ips200_show_string(  0, 16*2, "state");
    ips200_show_uint(   100, 16*2, gnss.state, 5);

    ips200_show_string(  0, 16*4, "satellite_used");
    ips200_show_uint(   150, 16*4, gnss.satellite_used, 5);
//    ips200_show_float(0,16*6,real_time_latitude,4,6);
//    ips200_show_float(0,16*7,real_time_longitude,4,6);

//    ips200_show_string(  0, 16*8,  "targetYaw");
//    ips200_show_float( 100, 16*8,  (float)target_yaw_imu,   6, 2);
//
//    ips200_show_string(  0, 16*9,  "distance");
//    ips200_show_float( 100, 16*9,  (float)new_distance,  6, 2);、、
    ips200_show_string(0,16*9,"roll");
    ips200_show_float(100, 16*9, Roll_a, 6, 2);    // x=0, y=0, 閿熸枻鎷烽敓鏂ゆ嫹6浣�, 灏忛敓鏂ゆ嫹2浣�

    ips200_show_string(  0, 16*10, "yaw");
    ips200_show_float( 100, 16*10, (float)icm42688_yaw,  6, 2);

    // ===== 新增：显示已存点数 =====
    ips200_show_string(  0, 16*11, "saved:");
    ips200_show_uint(  100, 16*11, current_point_number_flash, 3);

    // 显示当前正在追踪第几个点
    ips200_show_string(  0, 16*12, "going:");
    ips200_show_uint(  100, 16*12, next_point, 3);

    
}
























