#ifndef SRC_USER_BALANCE_H_
#define SRC_USER_BALANCE_H_
#include "zf_common_headfile.h"

/* 瑙掑害 */
extern float Roll_A0, Pitch_A0, Yaw_A0;
extern float Pitch_a, Roll_a, Yaw_a;
extern float Pitch_a_Pi, Roll_a_Pi, Yaw_a_Pi;
/* 瑙掗�熷害 */
extern float Pitch_g, Roll_g, Yaw_g;
extern float Pitch_g_F, Roll_g_F, Yaw_g_F;
extern float gyro_x_correction, gyro_y_correction, gyro_z_correction;
/* 鍔犻�熷害 */
extern float Pitch_acc, Roll_acc, Yaw_acc;
extern float Pitch_acc_F, Roll_acc_F, Yaw_acc_F;

extern float icm42688_roll;
extern float icm42688_yaw;
typedef float sample_t;

typedef enum {
    BIQUAD_LOWPASS,
    BIQUAD_HIGHPASS,
    BIQUAD_BANDPASS_PEAK,
    BIQUAD_BANDSTOP_NOTCH,
} biquad_type;

typedef struct
{
    sample_t a0, a1, a2, a3, a4;
    sample_t x1, x2, y1, y2;
} biquad_state;

void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value);
sample_t biquad(biquad_state *state, sample_t data);

void Filter_Init(void);
void Control(void);
void Reset_Control(void);
void Gyro_Calibration(void);

#endif /* SRC_USER_BALANCE_H_ */
