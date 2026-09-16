#include "___Control.h"

/* 瑙掑害 */
float Roll_A0, Pitch_A0, Yaw_A0;
float Pitch_a, Roll_a, Yaw_a;
float Pitch_a_Pi, Roll_a_Pi, Yaw_a_Pi;
/* 瑙掗�熷害 */
float Pitch_g, Roll_g, Yaw_g;
float Pitch_g_F, Roll_g_F, Yaw_g_F;
float gyro_x_correction, gyro_y_correction, gyro_z_correction;
/* 鍔犻�熷害 */
float Pitch_acc, Roll_acc, Yaw_acc;
float Pitch_acc_F, Roll_acc_F, Yaw_acc_F;

biquad_state Pitch_g_biquad, Roll_g_biquad, Yaw_g_biquad,
             Pitch_acc_biquad, Roll_acc_biquad, Yaw_acc_biquad;
float icm42688_roll = 0.0;
float icm42688_yaw = 0.0;

void Filter_Init(void){
    biquad_filter_init(&Pitch_g_biquad, BIQUAD_LOWPASS, 200, 30, 0.7071);
    biquad_filter_init(&Roll_g_biquad, BIQUAD_LOWPASS, 200, 30, 0.7071);
    biquad_filter_init(&Yaw_g_biquad, BIQUAD_LOWPASS, 200, 30, 0.7071);
    biquad_filter_init(&Pitch_acc_biquad, BIQUAD_LOWPASS, 200, 10, 0.7071);
    biquad_filter_init(&Roll_acc_biquad, BIQUAD_LOWPASS, 200, 10, 0.7071);
    biquad_filter_init(&Yaw_acc_biquad, BIQUAD_LOWPASS, 200, 10, 0.7071);
}

void Control(void){
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();

    Pitch_acc = -icm42688_acc_y;
    Roll_acc  =  icm42688_acc_x;
    Yaw_acc   =  icm42688_acc_z;

    Pitch_acc_F = biquad(&Pitch_acc_biquad, Pitch_acc);
    Roll_acc_F  = biquad(&Roll_acc_biquad, Roll_acc);
    Yaw_acc_F   = biquad(&Yaw_acc_biquad, Yaw_acc);

    Pitch_g = -icm42688_gyro_y;
    Roll_g  =  icm42688_gyro_x;
    Yaw_g   =  icm42688_gyro_z;

    Pitch_g_F = biquad(&Pitch_g_biquad, Pitch_g);
    Roll_g_F  = biquad(&Roll_g_biquad, Roll_g);
    Yaw_g_F   = biquad(&Yaw_g_biquad, Yaw_g);
    float gz = icm42688_gyro_z;
    if(fabsf(gz) < 0.15f) gz = 0;

    MahonyAHRSupdateIMU(icm42688_gyro_x, icm42688_gyro_y, gz,
                        icm42688_acc_x, icm42688_acc_y, icm42688_acc_z);
    Pitch_a = Rad2Ang * (Pitch_a_Pi - Pitch_A0);
    Roll_a  = Rad2Ang * (Roll_a_Pi - Roll_A0);
    Yaw_a   = Rad2Ang * (Yaw_a_Pi - Yaw_A0);

    icm42688_roll = Roll_a;
    icm42688_yaw = Yaw_a;

}

void Reset_Control(void){
    Pitch_a = Roll_a = Yaw_a = 0;
    Pitch_a_Pi = Roll_a_Pi = Yaw_a_Pi = 0;
    Pitch_g = Roll_g = Yaw_g = 0;
    Pitch_g_F = Roll_g_F = Yaw_g_F = 0;
    Pitch_acc = Roll_acc = Yaw_acc = 0;
    Pitch_acc_F = Roll_acc_F = Yaw_acc_F = 0;
}

#define IMU_CALIBRATION_SECONDS 3
void Gyro_Calibration(void){
    int s_cnt = 0, g_cnt = 0;
    float gyro_history[3][IMU_CALIBRATION_SECONDS];
    long gyro_history_sum[3] = {0, 0, 0};

    gyro_x_correction = 0;
    gyro_y_correction = 0;
    gyro_z_correction = 0;

    //ips200_full(RGB565_BLACK);
    ips200_show_string(0, 0, "IMU Calibration");
    ips200_show_string(0, 1, "Don't move me!");
    ips200_show_string(0, 6, "3"); system_delay_ms(1000);
    ips200_show_string(0, 6, "2"); system_delay_ms(1000);
    ips200_show_string(0, 6, "1"); system_delay_ms(1000);
    //ips200_full(RGB565_BLACK);
    //ips200_show_string(0, 0, "Calibrating...");

    do {
        system_delay_us(250);
        Get_RAW_Gyro_ICM42688();

        if(g_cnt < 4000){
            gyro_history_sum[0] += icm42688_gyro_x_r;
            gyro_history_sum[1] += icm42688_gyro_y_r;
            gyro_history_sum[2] += icm42688_gyro_z_r;
            g_cnt++;
        } else {
            gyro_history[0][s_cnt] = (float)gyro_history_sum[0] / 4000.0f;
            gyro_history[1][s_cnt] = (float)gyro_history_sum[1] / 4000.0f;
            gyro_history[2][s_cnt] = (float)gyro_history_sum[2] / 4000.0f;
            gyro_history_sum[0] = 0;
            gyro_history_sum[1] = 0;
            gyro_history_sum[2] = 0;
            g_cnt = 0;
            s_cnt++;
        }
    } while(s_cnt < IMU_CALIBRATION_SECONDS);

    float sum[3] = {0, 0, 0};
    float k = icm42688_gyro_inv / IMU_CALIBRATION_SECONDS;
    for(int i = 0; i < IMU_CALIBRATION_SECONDS; i++){
        sum[0] += gyro_history[0][i];
        sum[1] += gyro_history[1][i];
        sum[2] += gyro_history[2][i];
    }
    gyro_x_correction =  k * sum[0];
    gyro_y_correction = -k * sum[1];
    gyro_z_correction = -k * sum[2];

    //ips200_full(RGB565_BLACK);
    ips200_show_string(0, 0, "IMU Calibration OK");
//    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
//    ips200_show_float(0, 2, gyro_x_correction, 3, 2);
//    ips200_show_float(0, 3, gyro_y_correction, 3, 2);
//    ips200_show_float(0, 4, gyro_z_correction, 3, 2);
}

// 2nd-order Butterworth: q_value=0.7071
// 2nd-order Chebyshev (ripple 1 dB): q_value=0.9565
// 2nd-order Thomson-Bessel: q_value=0.5773
// fs: 閲囨牱棰戠巼, fc: 鎴棰戠巼(甯﹂�氫负涓績棰戠巼)
void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value)
{
    sample_t w0, sin_w0, cos_w0, alpha;
    sample_t b0, b1, b2, a0, a1, a2;

    w0 = 2 * PI * fc / fs;
    sin_w0 = sinf(w0);
    cos_w0 = cosf(w0);
    alpha = sin_w0 / (2.0 * q_value);

    switch(type)
    {
    case BIQUAD_LOWPASS:
        b0 = (1.0 - cos_w0) / 2.0;
        b1 = b0 * 2;
        b2 = b0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_HIGHPASS:
        b0 = (1.0 + cos_w0) / 2.0;
        b1 = -b0 * 2;
        b2 = b0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_BANDPASS_PEAK:
        b0 = alpha;
        b1 = 0.0;
        b2 = -alpha;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_BANDSTOP_NOTCH:
        b0 = 1.0;
        b1 = -2.0 * cos_w0;
        b2 = 1.0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    }
    state->a0 = b0 / a0;
    state->a1 = b1 / a0;
    state->a2 = b2 / a0;
    state->a3 = a1 / a0;
    state->a4 = a2 / a0;
    state->x1 = state->x2 = 0.0;
    state->y1 = state->y2 = 0.0;
}

sample_t biquad(biquad_state *state, sample_t data)
{
    sample_t result = 0;
    result = state->a0 * data + state->a1 * state->x1 + state->a2 * state->x2
           - state->a3 * state->y1 - state->a4 * state->y2;
    state->x2 = state->x1;
    state->x1 = data;
    state->y2 = state->y1;
    state->y1 = result;
    return result;
}
