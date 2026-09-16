#include "Wireless.h"
#include <string.h>
#include <stdlib.h>
#include "Control.h"
#include "Motor.h"

#define SAVE_DEBOUNCE_MS   500

static char   rx_line[256];
static uint32 rx_len          = 0;
static uint32 save_pending_ms = 0;   // 闃叉姈璁℃椂锛坢s锛�

void wu_send_str(const char *s)
{
    wireless_uart_send_buffer((uint8*)s, (uint32)strlen(s));
}

static void send_current_params(void)
{
    char buf[256];
    sprintf(buf,
        "PARAMS: AKP=%.3f AKD=%.3f RKD=%.3f YP=%.3f SPD=%d SKP=%.3f SKI=%.3f SKD=%.3f\r\n",
        g_params.angle_kp, g_params.angle_kd, g_params.rate_kd, g_params.yp,
        g_params.target_speed,
        g_params.speed_kp, g_params.speed_ki, g_params.speed_kd);
    wu_send_str(buf);
}

static void apply_cmd(const char *key, float val_float)
{
    int matched = 1;

    if      (strcmp(key, "SPD") == 0) { g_params.target_speed = (int32)val_float; Ctrl_lastSpeed = val_float; }
    else if (strcmp(key, "RKP") == 0) { g_params.rate_kp  = val_float; Rate_Servo.kp   = val_float; }
    else if (strcmp(key, "RKI") == 0) { g_params.rate_ki  = val_float; Rate_Servo.ki   = val_float; }
    else if (strcmp(key, "RKD") == 0) { g_params.rate_kd  = val_float; Rate_Servo.kd   = val_float; }

    else if (strcmp(key, "AKP") == 0) { g_params.angle_kp = val_float; RollAnglePID.kp = val_float; }
    else if (strcmp(key, "AKI") == 0) { g_params.angle_ki = val_float; RollAnglePID.ki = val_float; }
    else if (strcmp(key, "AKD") == 0) { g_params.angle_kd = val_float; RollAnglePID.kd = val_float; }

    else if (strcmp(key, "KP")  == 0) { g_params.kp  = val_float; AngleServoPID.kp = val_float; }
    else if (strcmp(key, "KD")  == 0) { g_params.kd  = val_float; AngleServoPID.kd = val_float; }
    else if (strcmp(key, "KI")  == 0) { g_params.ki  = val_float; AngleServoPID.ki = val_float; }

    else if (strcmp(key, "YP")  == 0) { g_params.yp  = val_float; YawPID.kp        = val_float; }
    
    else if (strcmp(key, "SKP") == 0) { g_params.speed_kp = val_float; speed_pid.kp = val_float; }
    else if (strcmp(key, "SKI") == 0) { g_params.speed_ki = val_float; speed_pid.ki = val_float; }
    else if (strcmp(key, "SKD") == 0) { g_params.speed_kd = val_float; speed_pid.kd = val_float; }



    else { matched = 0; }

    if (matched) {
        save_pending_ms = SAVE_DEBOUNCE_MS;   // 鍚姩 500ms 闃叉姈锛岀◢鍚庝繚瀛�

        char feedback[64];
        sprintf(feedback, "SET %s=%.4f OK\r\n", key, val_float);
        wu_send_str(feedback);
    } else {
        wu_send_str("ERR: Unknown command\r\n");
    }
}

static void handle_line(char *line)
{
    int len = (int)strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == ' '))
        line[--len] = '\0';

    char *trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    len = (int)strlen(trimmed);

    if (len < 1 || len > 30) {
        wu_send_str("ERR: Invalid length\r\n");
        return;
    }

    char *eq = strchr(trimmed, '=');

    if (!eq) {
        if (strcmp(trimmed, "GET") == 0)   { send_current_params(); return; }
        if (strcmp(trimmed, "RESET") == 0) {
            g_params = DEFAULT_PARAMS;
            Param_ApplyToPID();
            Ctrl_lastSpeed = (float)g_params.target_speed;
            Param_SaveNow();
            wu_send_str("RESET to default OK\r\n");
            return;
        }
        wu_send_str("ERR: No operator found\r\n");
        return;
    }

    *eq = '\0';
    char *key     = trimmed;
    char *val_str = eq + 1;

    if (*val_str == '\0') {
        wu_send_str("ERR: Empty value\r\n");
        return;
    }

    char *p = val_str;
    if (*p == '-') p++;
    for (; *p; p++) {
        if ((*p < '0' || *p > '9') && *p != '.') {
            wu_send_str("ERR: Invalid value format\r\n");
            return;
        }
    }

    float val_float = atof(val_str);

    apply_cmd(key, val_float);
}

void uart_rx_process(void)
{
    uint8  buf[256];
    uint32 rn = wireless_uart_read_buffer(buf, sizeof(buf));
    if (!rn) return;

    for (uint32 i = 0; i < rn; i++)
    {
        char ch = (char)buf[i];

        if (ch == '\n')
        {
            if (rx_len > 0)
            {
                rx_line[rx_len] = '\0';
                handle_line(rx_line);
                rx_len = 0;
            }
        }
        else if (ch == '\r')
        {

        }
        else if (ch >= 32 && ch <= 126)
        {
            if (rx_len < sizeof(rx_line) - 1)
            {
                rx_line[rx_len++] = ch;
            }
            else
            {
                wu_send_str("ERR: Line too long\r\n");
                rx_len = 0;
            }
        }
    }
}

void wireless_tick_5ms(void)
{
    // 闃叉姈璁℃椂锛氬弬鏁版敼鍔ㄥ悗寤惰繜 500ms 鍐嶅啓鍏� Flash锛岄伩鍏嶉绻佹摝鍐�
    if (save_pending_ms > 0) {
        if (save_pending_ms <= 5) {
            save_pending_ms = 0;
            Param_SaveNow();
        } else {
            save_pending_ms -= 5;
        }
    }
}

void wireless_send_data(void){
    char buf[128];
    sprintf(buf, "%.1f,%.1f,%.1f,%.4f,%.4f\r\n",
            Roll_a, Pitch_a, Yaw_a, motor_get_speed(),Yaw_g_F);
    wu_send_str(buf);
}
