/****************************************************************************************
 *     COPYRIGHT NOTICE
 *     Copyright (C) 2024,AS DAIMXA
 *     copyright Copyright (C) 锟斤拷锟斤拷锟斤拷DAIMXA,2024
 *     All rights reserved.
 *     锟斤拷锟斤拷锟斤拷锟斤拷QQ群锟斤拷710026750
 *
 *     锟斤拷注锟斤拷锟斤拷锟斤拷锟解，锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷帮拷权锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟杰科硷拷锟斤拷锟叫ｏ拷未锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷业锟斤拷途锟斤拷
 *     锟睫革拷锟斤拷锟斤拷时锟斤拷锟诫保锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟狡硷拷锟侥帮拷权锟斤拷锟斤拷锟斤拷
 *      ____    _    ___ __  ____  __    _
 *     |  _ \  / \  |_ _|  \/  \ \/ /   / \
 *     | | | |/ _ \  | || |\/| |\  /   / _ \
 *     | |_| / ___ \ | || |  | |/  \  / ___ \
 *     |____/_/   \_\___|_|  |_/_/\_\/_/   \_\
 *
 * @file       dmx_icm42688.c
 * @brief      锟斤拷锟斤拷锟斤拷CH32V307VCT6锟斤拷源锟斤拷
 * @company    锟较肥达拷锟斤拷锟斤拷锟斤拷锟杰科硷拷锟斤拷锟睫癸拷司
 * @author     锟斤拷锟斤拷锟斤拷锟狡硷拷锟斤拷QQ锟斤拷2453520483锟斤拷
 * @MCUcore    CH32V307VCT6
 * @Software   MounRicer Stdio V191
 * @version    锟介看说锟斤拷锟侥碉拷锟斤拷version锟芥本说锟斤拷
 * @Taobao     https://daimxa.taobao.com/
 * @Openlib    https://gitee.com/daimxa
 * @date       2024-01-04
****************************************************************************************/

#include "dmx_icm42688.h"
#include "zf_common_headfile.h"

// ICM42688锟斤拷锟劫度硷拷锟斤拷锟斤拷
float icm42688_acc_x  = 0, icm42688_acc_y  = 0, icm42688_acc_z  = 0;
// ICM42688锟角硷拷锟劫讹拷锟斤拷锟斤拷
float icm42688_gyro_x = 0, icm42688_gyro_y = 0, icm42688_gyro_z = 0;
short int icm42688_gyro_x_r = 0, icm42688_gyro_y_r = 0, icm42688_gyro_z_r = 0;

// SPI协锟斤拷锟叫达拷锟斤拷锟斤拷甓拷锟�
#define ICM42688_Write_Reg(reg, data)       spi_write_8bit_register(ICM42688_SPI, reg, data);
#define ICM42688_Read_Regs(reg, data,num)   spi_read_8bit_registers(ICM42688_SPI, reg | 0x80, data, num);

// 锟斤拷态锟斤拷锟斤拷锟斤拷锟斤拷,锟斤拷锟铰猴拷锟斤拷锟斤拷为锟斤拷.c锟侥硷拷锟节诧拷锟斤拷锟斤拷
static void Write_Data_ICM42688(unsigned char reg, unsigned char data);
static void Read_Datas_ICM42688(unsigned char reg, unsigned char *data, unsigned int num);
// 锟斤拷锟斤拷转锟斤拷为实锟斤拷锟斤拷锟斤拷锟斤拷锟捷碉拷转锟斤拷系锟斤拷
float icm42688_acc_inv = 1, icm42688_gyro_inv = 1;



/**
*
* @brief    ICM42688锟斤拷锟斤拷锟角筹拷始锟斤拷
* @param
* @return   void
* @notes    锟矫伙拷锟斤拷锟斤拷
* Example:  Init_ICM42688();
*
**/
void Init_ICM42688(void)
{
    // SPI锟斤拷始锟斤拷
    spi_init(ICM42688_SPI, SPI_MODE0, ICM42688_SPI_SPEED, ICM42688_SPC_PIN, ICM42688_SDI_PIN, ICM42688_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM42688_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    // 锟斤拷始锟斤拷锟斤拷时
    char time = 50;
    // 锟斤拷取锟斤拷锟斤拷锟斤拷锟酵猴拷锟斤拷锟斤拷锟斤拷锟皆硷拷
    unsigned char model = 0xff;
    while(1)
    {
        // 锟斤拷芯片ID
        Read_Datas_ICM42688(ICM42688_WHO_AM_I, &model, 1);
        if(model == 0x47)
        {
            // ICM42688,71
            break;
        }
        else
        {
            ICM42688_DELAY_MS(10);
            time--;
            if(time < 0)
            {
                zf_log(0, "ICM42688 Init Error!");
                while(1);
                // 锟斤拷锟斤拷锟斤拷锟斤拷原锟斤拷锟斤拷锟斤拷锟铰硷拷锟斤拷
                // ICM42688锟斤拷锟斤拷,锟斤拷锟斤拷锟斤拷碌母锟斤拷始锟斤拷锟�
                // 锟斤拷锟竭达拷锟斤拷锟斤拷锟矫伙拷薪雍锟�
                // 锟斤拷锟斤拷太锟斤拷,通锟斤拷失锟斤拷
            }
        }
    }
    Write_Data_ICM42688(ICM42688_PWR_MGMT0, 0x00);      // 锟斤拷位锟借备
    ICM42688_DELAY_MS(10);                              // 锟斤拷锟斤拷锟斤拷PWR锟斤拷MGMT0锟侥达拷锟斤拷锟斤拷200us锟节诧拷锟斤拷锟斤拷锟轿何讹拷写锟侥达拷锟斤拷锟侥诧拷锟斤拷

    // 锟斤拷锟斤拷ICM42688锟斤拷锟劫度计猴拷锟斤拷锟斤拷锟角碉拷锟斤拷锟教猴拷锟斤拷锟斤拷锟斤拷锟�
    Set_LowpassFilter_Range_ICM42688(ICM42688_AFS_16G, ICM42688_AODR_200HZ, ICM42688_GFS_2000DPS, ICM42688_GODR_200HZ);

    Write_Data_ICM42688(ICM42688_PWR_MGMT0, 0x0f);      // 锟斤拷锟斤拷GYRO_MODE,ACCEL_MODE为锟斤拷锟斤拷锟斤拷模式
    ICM42688_DELAY_MS(10);
}

/**
*
* @brief    锟斤拷锟絀CM42688锟斤拷锟斤拷锟角硷拷锟劫讹拷
* @param
* @return   void
* @notes    锟斤拷位:g(m/s^2),锟矫伙拷锟斤拷锟斤拷
* Example:  Get_Acc_ICM42688();
*
**/
void Get_Acc_ICM42688(void)
{
    unsigned char data[6];
    Read_Datas_ICM42688(ICM42688_ACCEL_DATA_X1, data, 6);
    icm42688_acc_x = icm42688_acc_inv * (short int)(((short int)data[0] << 8) | data[1]);
    icm42688_acc_y = -icm42688_acc_inv * (short int)(((short int)data[2] << 8) | data[3]);
    icm42688_acc_z = -icm42688_acc_inv * (short int)(((short int)data[4] << 8) | data[5]);
}

/**
*
* @brief    锟斤拷锟絀CM42688锟斤拷锟斤拷锟角角硷拷锟劫讹拷
* @param
* @return   void
* @notes    锟斤拷位为:锟斤拷/s,锟矫伙拷锟斤拷锟斤拷
* Example:  Get_Gyro_ICM42688();
*
**/
void Get_Gyro_ICM42688(void)
{
    unsigned char data[6];
    Read_Datas_ICM42688(ICM42688_GYRO_DATA_X1, data, 6);
    icm42688_gyro_x = icm42688_gyro_inv * (short int)(((short int)data[0] << 8) | data[1]) - gyro_x_correction;
    icm42688_gyro_y = -icm42688_gyro_inv * (short int)(((short int)data[2] << 8) | data[3]) - gyro_y_correction;
    icm42688_gyro_z = -icm42688_gyro_inv * (short int)(((short int)data[4] << 8) | data[5]) - gyro_z_correction;
}
void Get_RAW_Gyro_ICM42688(void)
{
    unsigned char data[6];
    Read_Datas_ICM42688(ICM42688_GYRO_DATA_X1, data, 6);
    icm42688_gyro_x_r = (short int)(((short int)data[0] << 8) | data[1]);
    icm42688_gyro_y_r = (short int)(((short int)data[2] << 8) | data[3]);
    icm42688_gyro_z_r = (short int)(((short int)data[4] << 8) | data[5]);
}

/**
*
* @brief    锟斤拷锟斤拷ICM42688锟斤拷锟斤拷锟角碉拷通锟剿诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
* @param    afs                 // 锟斤拷锟劫度硷拷锟斤拷锟斤拷,锟斤拷锟斤拷dmx_icm42688.h锟侥硷拷锟斤拷枚锟劫讹拷锟斤拷锟叫查看
* @param    aodr                // 锟斤拷锟劫度硷拷锟斤拷锟斤拷锟斤拷锟�,锟斤拷锟斤拷dmx_icm42688.h锟侥硷拷锟斤拷枚锟劫讹拷锟斤拷锟叫查看
* @param    gfs                 // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷,锟斤拷锟斤拷dmx_icm42688.h锟侥硷拷锟斤拷枚锟劫讹拷锟斤拷锟叫查看
* @param    godr                // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�,锟斤拷锟斤拷dmx_icm42688.h锟侥硷拷锟斤拷枚锟劫讹拷锟斤拷锟叫查看
* @return   void
* @notes    ICM42688.c锟侥硷拷锟节诧拷锟斤拷锟斤拷,锟矫伙拷锟斤拷锟斤拷锟斤拷贸锟斤拷锟�
* Example:  Set_LowpassFilter_Range_ICM42688(ICM42688_AFS_16G,ICM42688_AODR_32000HZ,ICM42688_GFS_2000DPS,ICM42688_GODR_32000HZ);
*
**/
void Set_LowpassFilter_Range_ICM42688(enum icm42688_afs afs, enum icm42688_aodr aodr, enum icm42688_gfs gfs, enum icm42688_godr godr)
{
    Write_Data_ICM42688(ICM42688_ACCEL_CONFIG0, (afs << 5) | (aodr + 1));   // 锟斤拷始锟斤拷ACCEL锟斤拷锟教猴拷锟斤拷锟斤拷锟斤拷锟�(p77)
    Write_Data_ICM42688(ICM42688_GYRO_CONFIG0, (gfs << 5) | (godr + 1));    // 锟斤拷始锟斤拷GYRO锟斤拷锟教猴拷锟斤拷锟斤拷锟斤拷锟�(p76)

    switch(afs)
    {
    case ICM42688_AFS_2G:
        icm42688_acc_inv = 2000 / 32768.0f;             // 锟斤拷锟劫度硷拷锟斤拷锟斤拷为:锟斤拷2g
        break;
    case ICM42688_AFS_4G:
        icm42688_acc_inv = 4000 / 32768.0f;             // 锟斤拷锟劫度硷拷锟斤拷锟斤拷为:锟斤拷4g
        break;
    case ICM42688_AFS_8G:
        icm42688_acc_inv = 8000 / 32768.0f;             // 锟斤拷锟劫度硷拷锟斤拷锟斤拷为:锟斤拷8g
        break;
    case ICM42688_AFS_16G:
        icm42688_acc_inv = 16000 / 32768.0f;            // 锟斤拷锟劫度硷拷锟斤拷锟斤拷为:锟斤拷16g
        break;
    default:
        icm42688_acc_inv = 1;                           // 锟斤拷转锟斤拷为实锟斤拷锟斤拷锟斤拷
        break;
    }
    switch(gfs)
    {
    case ICM42688_GFS_15_625DPS:
        icm42688_gyro_inv = 15.625f / 32768.0f;         // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷为:锟斤拷15.625dps
        break;
    case ICM42688_GFS_31_25DPS:
        icm42688_gyro_inv = 31.25f / 32768.0f;          // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷为:锟斤拷31.25dps
        break;
    case ICM42688_GFS_62_5DPS:
        icm42688_gyro_inv = 62.5f / 32768.0f;           // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷为:锟斤拷62.5dps
        break;
    case ICM42688_GFS_125DPS:
        icm42688_gyro_inv = 125.0f / 32768.0f;          // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷为:锟斤拷125dps
        break;
    case ICM42688_GFS_250DPS:
        icm42688_gyro_inv = 250.0f / 32768.0f;          // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷为:锟斤拷250dps
        break;
    case ICM42688_GFS_500DPS:
        icm42688_gyro_inv = 500.0f / 32768.0f;          // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷为:锟斤拷500dps
        break;
    case ICM42688_GFS_1000DPS:
        icm42688_gyro_inv = 1000.0f / 32768.0f;         // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷为:锟斤拷1000dps
        break;
    case ICM42688_GFS_2000DPS:
        icm42688_gyro_inv = 2000.0f / 32768.0f;         // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷为:锟斤拷2000dps
        break;
    default:
        icm42688_gyro_inv = 1;                          // 锟斤拷转锟斤拷为实锟斤拷锟斤拷锟斤拷
        break;
    }
}

/**
*
* @brief    ICM42688锟斤拷锟斤拷锟斤拷写锟斤拷锟斤拷
* @param    reg                 锟侥达拷锟斤拷
* @param    data                锟斤拷要写锟斤拷锟矫寄达拷锟斤拷锟斤拷锟斤拷锟斤拷
* @return   void
* @notes    ICM42688.c锟侥硷拷锟节诧拷锟斤拷锟斤拷,锟矫伙拷锟斤拷锟斤拷锟斤拷贸锟斤拷锟�
* Example:  Write_Data_ICM42688(0x00,0x00);
*
**/
static void Write_Data_ICM42688(unsigned char reg, unsigned char data)
{
    ICM42688_CS_LEVEL(0);
    ICM42688_Write_Reg(reg, data);
    ICM42688_CS_LEVEL(1);
}

/**
*
* @brief    ICM42688锟斤拷锟斤拷锟角讹拷锟斤拷锟斤拷
* @param    reg                 锟侥达拷锟斤拷
* @param    data                锟窖讹拷锟斤拷锟斤拷锟斤拷锟捷达拷锟斤拷data
* @param    num                 锟斤拷锟捷革拷锟斤拷
* @return   void
* @notes    ICM42688.c锟侥硷拷锟节诧拷锟斤拷锟斤拷,锟矫伙拷锟斤拷锟斤拷锟斤拷贸锟斤拷锟�
* Example:  Read_Datas_ICM42688(0x00,data,1);
*
**/
static void Read_Datas_ICM42688(unsigned char reg, unsigned char *data, unsigned int num)
{
    ICM42688_CS_LEVEL(0);
    ICM42688_Read_Regs(reg, data, num);
    ICM42688_CS_LEVEL(1);
}
void Print_ICM42688_Data(void)
{
//    printf("%.2f锟斤拷,%.2f锟斤拷,%.2f锟斤拷\n",
//           icm42688_roll, icm42688_pitch, icm42688_yaw);

//    printf("Acc: X=%.2f Y=%.2f Z=%.2f | ",
//           icm42688_acc_x, icm42688_acc_y, icm42688_acc_z);
//
//    printf("Gyro: X=%.2f Y=%.2f Z=%.2f\n",
//           icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z);
//
//        // 锟斤拷示Roll锟斤拷
        ips200_show_string(0,0,"roll");
        ips200_show_float(50, 0, Roll_a, 6, 2);    // x=0, y=0, 锟斤拷锟斤拷6位, 小锟斤拷2位
//        // 锟斤拷示Pitch锟斤拷
        ips200_show_string(0,20,"pitch");
        ips200_show_float(50, 20, Pitch_a, 6, 2);  // x=0, y=50
        // 锟斤拷示Yaw锟斤拷
        ips200_show_string(0,40,"yaw");
        ips200_show_float(50, 40, Yaw_a, 6, 2);   // x=0, y=100

//        ips200_show_string(0,60,"icm42688_gyro_x:");
//        ips200_show_float(140,60,icm42688_gyro_x,2,3);
//        ips200_show_string(0,80,"icm42688_gyro_y:");
//        ips200_show_float(140,80,icm42688_gyro_y,2,3);
//        ips200_show_string(0,100,"icm42688_gyro_z:");
//        ips200_show_float(140,100,icm42688_gyro_z,2,3);
//        ips200_show_string(0,120,"steering_angle:");
//


         //printf("yaw:%.2f\n",icm42688_roll);
        //printf("icm42688_gyro_x:%.2f\n",icm42688_gyro_x);


}
