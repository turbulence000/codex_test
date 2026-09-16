/*
 * route.h — XY 路径点导航（输入坐标 → 回放）
 *
 * 编码器: 91 脉冲 = 0.5cm, 1 格 = 20cm = 3640 脉冲
 *
 * 用法:
 *   route_pt_t path[] = {{0,0}, {0,5}, {3,10}};
 *   route_set_path(path, 3);
 *   route_start();
 *   while (route_is_running())
 *       target_yaw = route_step(diff_navigation);
 */

#ifndef CODE_ROUTE_H_
#define CODE_ROUTE_H_

#include "zf_common_headfile.h"

/* ============================================================
 * 常量
 * ============================================================ */
#define ROUTE_PULSE_PER_CM      182
#define ROUTE_GRID_CM            20
#define ROUTE_PULSE_PER_GRID   3640     // 20 * 182

#define ROUTE_MAX_POINTS        512
#define ROUTE_FLASH_PAGE          0

/* ============================================================
 * 路径点: X/Y 偏移 (格子单位, 右/前为正)
 * ============================================================ */
typedef struct {
    int16 x;
    int16 y;
} route_pt_t;

/* ============================================================
 * 路径实例
 * ============================================================ */
typedef struct {
    route_pt_t pts[ROUTE_MAX_POINTS];
    uint16_t   count;
    uint8_t    running;
    float      mileage;          // 累计里程 (脉冲)
    float      target_yaw;       // 当前目标偏航角 (route_step 更新)
} route_t;

extern route_t g_route;

/* ============================================================
 * API — 路径管理
 * ============================================================ */
void route_init(void);
void route_set_path(const route_pt_t *points, uint16_t n);
void route_clear(void);

/* ============================================================
 * API — 回放
 * ============================================================ */
void    route_start(void);
void    route_stop(void);
uint8_t route_is_running(void);
float   route_step(int32_t enc_delta);   // 喂编码器，返回目标偏航角 (度)

/* ============================================================
 * API — Flash
 * ============================================================ */
void route_save_flash(void);
void route_load_flash(void);

/* ============================================================
 * API — 按键控制
 * ============================================================ */
void key_route(void);       // 1=回放 2=停止 3=清除 4=解除停车

/* ============================================================
 * API — 屏幕
 * ============================================================ */
void route_print_info(void);

#endif /* CODE_ROUTE_H_ */
