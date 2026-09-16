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
 * @file       dmx_icm42688.h
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

#ifndef __DMX_ICM42688_H
#define __DMX_ICM42688_H

// ICM42688绠¤剼瀹忓畾涔�
#define ICM42688_SPI_SPEED      24*1000*1000      // 纭欢SPI閫熺巼
#define ICM42688_SPI            SPI_0             // 纭欢SPI鍙�
#define ICM42688_SPC_PIN        SPI0_SCLK_P20_11   // 纭欢SPI_SCK寮曡剼
#define ICM42688_SDI_PIN        SPI0_MOSI_P20_14  // 纭欢SPI_MOSI寮曡剼
#define ICM42688_SDO_PIN        SPI0_MISO_P20_12  // 纭欢SPI_MISO寮曡剼

// ICM42688,CS绠¤剼瀵瑰簲CH32V307VCT6寮曡剼C10
#define ICM42688_CS_PIN         P20_13

// ICM42688锟斤拷时
#define ICM42688_DELAY_MS(time)  (system_delay_ms(time))
// 锟斤拷ICM42688锟斤拷CS锟杰脚斤拷锟叫高低碉拷平锟斤拷锟斤拷
#define ICM42688_CS_LEVEL(level) gpio_set_level(ICM42688_CS_PIN  , level);

extern float icm42688_acc_inv, icm42688_gyro_inv;

// 锟斤拷锟斤拷ICM42688锟斤拷锟劫度硷拷锟斤拷锟斤拷
extern float icm42688_acc_x  , icm42688_acc_y  , icm42688_acc_z  ;
// 锟斤拷锟斤拷ICM42688锟角硷拷锟劫讹拷锟斤拷锟斤拷
extern float icm42688_gyro_x , icm42688_gyro_y , icm42688_gyro_z ;
extern short int icm42688_gyro_x_r , icm42688_gyro_y_r , icm42688_gyro_z_r ;

enum icm42688_afs
{
    ICM42688_AFS_16G,   // default
    ICM42688_AFS_8G,
    ICM42688_AFS_4G,
    ICM42688_AFS_2G,
    NUM_ICM42688__AFS
};
enum icm42688_aodr
{
    ICM42688_AODR_32000HZ,
    ICM42688_AODR_16000HZ,
    ICM42688_AODR_8000HZ,
    ICM42688_AODR_4000HZ,
    ICM42688_AODR_2000HZ,
    ICM42688_AODR_1000HZ,// default
    ICM42688_AODR_200HZ,
    ICM42688_AODR_100HZ,
    ICM42688_AODR_50HZ,
    ICM42688_AODR_25HZ,
    ICM42688_AODR_12_5HZ,
    ICM42688_AODR_6_25HZ,
    ICM42688_AODR_3_125HZ,
    ICM42688_AODR_1_5625HZ,
    ICM42688_AODR_500HZ,
    NUM_ICM42688_AODR
};

enum icm42688_gfs
{
    ICM42688_GFS_2000DPS,// default
    ICM42688_GFS_1000DPS,
    ICM42688_GFS_500DPS,
    ICM42688_GFS_250DPS,
    ICM42688_GFS_125DPS,
    ICM42688_GFS_62_5DPS,
    ICM42688_GFS_31_25DPS,
    ICM42688_GFS_15_625DPS,
    NUM_ICM42688_GFS
};
enum icm42688_godr
{
    ICM42688_GODR_32000HZ,
    ICM42688_GODR_16000HZ,
    ICM42688_GODR_8000HZ,
    ICM42688_GODR_4000HZ,
    ICM42688_GODR_2000HZ,
    ICM42688_GODR_1000HZ,// default
    ICM42688_GODR_200HZ,
    ICM42688_GODR_100HZ,
    ICM42688_GODR_50HZ,
    ICM42688_GODR_25HZ,
    ICM42688_GODR_12_5HZ,
    ICM42688_GODR_X0HZ,
    ICM42688_GODR_X1HZ,
    ICM42688_GODR_X2HZ,
    ICM42688_GODR_500HZ,
    NUM_ICM42688_GODR
};

void SPI_switch_to_ICM42688(void);

/**
*
* @brief    ICM42688锟斤拷锟斤拷锟角筹拷始锟斤拷
* @param
* @return   void
* @notes    锟矫伙拷锟斤拷锟斤拷
* Example:  Init_ICM42688();
*
**/
void Init_ICM42688(void);

/**
*
* @brief    锟斤拷锟絀CM42688锟斤拷锟斤拷锟角硷拷锟劫讹拷
* @param
* @return   void
* @notes    锟斤拷位:g(m/s^2),锟矫伙拷锟斤拷锟斤拷
* Example:  Get_Acc_ICM42688();
*
**/
void Get_Acc_ICM42688(void);

/**
*
* @brief    锟斤拷锟絀CM42688锟斤拷锟斤拷锟角角硷拷锟劫讹拷
* @param
* @return   void
* @notes    锟斤拷位为:锟斤拷/s,锟矫伙拷锟斤拷锟斤拷
* Example:  Get_Gyro_ICM42688();
*
**/
void Get_Gyro_ICM42688(void);
void Get_RAW_Gyro_ICM42688(void);
void Print_ICM42688_Data(void);
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
void Set_LowpassFilter_Range_ICM42688(enum icm42688_afs afs, enum icm42688_aodr aodr, enum icm42688_gfs gfs, enum icm42688_godr godr);

// ICM42688Bank0锟节诧拷锟斤拷址
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_DRIVE_CONFIG              0x13
#define ICM42688_INT_CONFIG                0x14
#define ICM42688_FIFO_CONFIG               0x16
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA0                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_ACCEL_DATA_X0             0x20
#define ICM42688_ACCEL_DATA_Y1             0x21
#define ICM42688_ACCEL_DATA_Y0             0x22
#define ICM42688_ACCEL_DATA_Z1             0x23
#define ICM42688_ACCEL_DATA_Z0             0x24
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_GYRO_DATA_X0              0x26
#define ICM42688_GYRO_DATA_Y1              0x27
#define ICM42688_GYRO_DATA_Y0              0x28
#define ICM42688_GYRO_DATA_Z1              0x29
#define ICM42688_GYRO_DATA_Z0              0x2A
#define ICM42688_TMST_FSYNCH               0x2B
#define ICM42688_TMST_FSYNCL               0x2C
#define ICM42688_INT_STATUS                0x2D
#define ICM42688_FIFO_COUNTH               0x2E
#define ICM42688_FIFO_COUNTL               0x2F
#define ICM42688_FIFO_DATA                 0x30
#define ICM42688_APEX_DATA0                0x31
#define ICM42688_APEX_DATA1                0x32
#define ICM42688_APEX_DATA2                0x33
#define ICM42688_APEX_DATA3                0x34
#define ICM42688_APEX_DATA4                0x35
#define ICM42688_APEX_DATA5                0x36
#define ICM42688_INT_STATUS2               0x37
#define ICM42688_INT_STATUS3               0x38
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_INTF_CONFIG0              0x4C
#define ICM42688_INTF_CONFIG1              0x4D
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_GYRO_CONFIG1              0x51
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52
#define ICM42688_ACCEL_CONFIG1             0x53
#define ICM42688_TMST_CONFIG               0x54
#define ICM42688_APEX_CONFIG0              0x56
#define ICM42688_SMD_CONFIG                0x57
#define ICM42688_FIFO_CONFIG1              0x5F
#define ICM42688_FIFO_CONFIG2              0x60
#define ICM42688_FIFO_CONFIG3              0x61
#define ICM42688_FSYNC_CONFIG              0x62
#define ICM42688_INT_CONFIG0               0x63
#define ICM42688_INT_CONFIG1               0x64
#define ICM42688_INT_SOURCE0               0x65
#define ICM42688_INT_SOURCE1               0x66
#define ICM42688_INT_SOURCE3               0x68
#define ICM42688_INT_SOURCE4               0x69
#define ICM42688_FIFO_LOST_PKT0            0x6C
#define ICM42688_FIFO_LOST_PKT1            0x6D
#define ICM42688_SELF_TEST_CONFIG          0x70
#define ICM42688_WHO_AM_I                  0x75
#define ICM42688_REG_BANK_SEL              0x76// Banks
#define ICM42688_SENSOR_CONFIG0            0x03
#define ICM42688_GYRO_CONFIG_STATIC2       0x0B
#define ICM42688_GYRO_CONFIG_STATIC3       0x0C
#define ICM42688_GYRO_CONFIG_STATIC4       0x0D
#define ICM42688_GYRO_CONFIG_STATIC5       0x0E
#define ICM42688_GYRO_CONFIG_STATIC6       0x0F
#define ICM42688_GYRO_CONFIG_STATIC7       0x10
#define ICM42688_GYRO_CONFIG_STATIC8       0x11
#define ICM42688_GYRO_CONFIG_STATIC9       0x12
#define ICM42688_GYRO_CONFIG_STATIC10      0x13
#define ICM42688_XG_ST_DATA                0x5F
#define ICM42688_YG_ST_DATA                0x60
#define ICM42688_ZG_ST_DATA                0x61
#define ICM42688_TMSTVAL0                  0x62
#define ICM42688_TMSTVAL1                  0x63
#define ICM42688_TMSTVAL2                  0x64
#define ICM42688_INTF_CONFIG4              0x7A
#define ICM42688_INTF_CONFIG5              0x7B
#define ICM42688_INTF_CONFIG6              0x7C

#endif /* __DMX_ICM42688_H */
