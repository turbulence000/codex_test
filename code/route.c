/*
 * route.c — XY 路径点导航（输入坐标 → 回放）
 *
 * 只管两件事:
 *   1. 接收 XY 路径数组 → 存 Flash
 *   2. 回放: 喂编码器 → 沿路径插值 → 返回目标偏航角
 *
 * 手推录制在 navigation.c, 这里不管。
 */

#include "zf_common_headfile.h"
#include "route.h"




/* ============================================================
 * 全局实例
 * ============================================================ */
route_t g_route;

/* 预计算: 每段距离 (脉冲), set_path 时刷新 */
static float seg_pulse[ROUTE_MAX_POINTS];

/* ============================================================
 * 内部
 * ============================================================ */
static float pt_dist(route_pt_t a, route_pt_t b)
{
    float dx = (float)(b.x - a.x);
    float dy = (float)(b.y - a.y);
    return sqrtf(dx * dx + dy * dy);
}

static void rebuild_seg_table(void)
{
    uint16_t i;
    for (i = 0; i < ROUTE_MAX_POINTS; i++) seg_pulse[i] = 0.0f;
    for (i = 1; i < g_route.count; i++) {
        seg_pulse[i] = pt_dist(g_route.pts[i - 1], g_route.pts[i])
                       * (float)ROUTE_PULSE_PER_GRID;
    }
}

/* ============================================================
 * 路径管理
 * ============================================================ */
void route_init(void)
{
    memset(&g_route, 0, sizeof(g_route));
    memset(seg_pulse, 0, sizeof(seg_pulse));
    route_load_flash();
    rebuild_seg_table();
}

void route_set_path(const route_pt_t *points, uint16_t n)
{
    uint16_t i;
    if (n > ROUTE_MAX_POINTS) n = ROUTE_MAX_POINTS;

    g_route.count = n;
    for (i = 0; i < n; i++)
        g_route.pts[i] = points[i];

    rebuild_seg_table();
    route_save_flash();
}

void route_clear(void)
{
    g_route.count   = 0;
    g_route.running = 0;
    g_route.mileage = 0;
    memset(seg_pulse, 0, sizeof(seg_pulse));
    flash_erase_page(0, ROUTE_FLASH_PAGE);
}

/* ============================================================
 * 回放
 * ============================================================ */
void route_start(void)
{
    if (g_route.count < 2) return;
    g_route.running = 1;
    g_route.mileage = 0;
}

void route_stop(void)
{
    g_route.running = 0;
}

uint8_t route_is_running(void)
{
    return g_route.running;
}

/*
 * route_step — 核心
 *
 * 喂编码器增量, 返回目标偏航角 (度, 与 icm42688_yaw 同坐标系).
 * 里程到达终点自动停止.
 *
 * 内部: 累计里程 → 沿 seg_pulse[] 定位当前段 → 线性插值当前位置
 *      → atan2(Δx, Δy) 求方向角
 */
float route_step(int32_t enc_delta)
{
    float  total_pulse, accum, seg_len, t;
    float  cur_x, cur_y, dx, dy;
    uint16_t seg;
    static int32_t last_enc = 0;

    if (!g_route.running || g_route.count < 2) {
        g_route.target_yaw = 0.0f;
        last_enc = 0;
        return 0.0f;
    }

    // 5ms调用但编码器10ms才更新一次, 同值跳过避免重复累加
    if (enc_delta == last_enc) {
        return g_route.target_yaw;
    }
    last_enc = enc_delta;

    g_route.mileage += (float)enc_delta;

    // 路径总长
    total_pulse = 0.0f;
    for (seg = 1; seg < g_route.count; seg++)
        total_pulse += seg_pulse[seg];

    if (g_route.mileage >= total_pulse) {
        g_route.running  = 0;
        g_route.target_yaw = 0.0f;
        stop_flag = 1;                  // 跑完停车
        return 0.0f;
    }

    // 定位当前段
    accum = 0.0f;
    seg   = 1;
    while (seg < g_route.count) {
        if (g_route.mileage < accum + seg_pulse[seg]) break;
        accum += seg_pulse[seg];
        seg++;
    }
    if (seg >= g_route.count) seg = g_route.count - 1;

    // 段内插值
    seg_len = pt_dist(g_route.pts[seg - 1], g_route.pts[seg]);
    t = (seg_len > 0.001f)
        ? (g_route.mileage - accum) / seg_pulse[seg]
        : 0.0f;
    if (t > 1.0f) t = 1.0f;

    cur_x = (float)g_route.pts[seg - 1].x
          + (float)(g_route.pts[seg].x - g_route.pts[seg - 1].x) * t;
    cur_y = (float)g_route.pts[seg - 1].y
          + (float)(g_route.pts[seg].y - g_route.pts[seg - 1].y) * t;

    dx = (float)g_route.pts[seg].x - cur_x;
    dy = (float)g_route.pts[seg].y - cur_y;

    g_route.target_yaw = atan2f(dx, dy) * 57.29578f;
    return g_route.target_yaw;
}

/* ============================================================
 * Flash
 * ============================================================ */
void route_save_flash(void)
{
    uint16_t i;

    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = g_route.count;

    for (i = 0; i < g_route.count; i++) {
        flash_union_buffer[1 + i * 2].uint32_type     = (uint32_t)(int32_t)g_route.pts[i].x;
        flash_union_buffer[1 + i * 2 + 1].uint32_type = (uint32_t)(int32_t)g_route.pts[i].y;
    }

    flash_erase_page(0, ROUTE_FLASH_PAGE);
    flash_write_page_from_buffer(0, ROUTE_FLASH_PAGE);
}

void route_load_flash(void)
{
    uint32_t n;
    uint16_t i;

    flash_buffer_clear();
    flash_read_page_to_buffer(0, ROUTE_FLASH_PAGE);

    n = flash_union_buffer[0].uint32_type;
    if (n > ROUTE_MAX_POINTS) n = 0;

    g_route.count = (uint16_t)n;
    for (i = 0; i < g_route.count; i++) {
        g_route.pts[i].x = (int16_t)(int32_t)flash_union_buffer[1 + i * 2].uint32_type;
        g_route.pts[i].y = (int16_t)(int32_t)flash_union_buffer[1 + i * 2 + 1].uint32_type;
    }
}

/* ============================================================
 * 按键控制
 *
 *   1: 开始回放
 *   2: 停止回放
 *   3: 清除路径
 *   4: 解除停车 (stop_flag = 0)
 * ============================================================ */
void key_route(void)
{
    uint8_t key = key_get_short_press();

    if (key == 0) return;

    switch (key) {
        case 1:
            route_start();
            break;
        case 2:
            route_stop();
            break;
        case 3:
            route_clear();
            break;
        case 4:
            stop_flag = 0;
            break;
        default:
            break;
    }
}

/* ============================================================
 * 屏幕
 * ============================================================ */
void route_print_info(void)
{
    uint16_t i, line_y;
    float d, cm, ang;

    ips200_show_string(0, 60, "route pts:");
    ips200_show_int(60, 60, g_route.count, 3);
    ips200_show_string(84, 60, g_route.running ? "RUN" : "   ");

    ips200_show_string(0, 70, "mile(cm):");
    ips200_show_float(54, 70, g_route.mileage / (float)ROUTE_PULSE_PER_CM, 6, 1);

    for (i = 1; i < g_route.count; i++) {
        line_y = 85 + (i - 1) * 10;
        if (line_y >= 310) break;

        d  = pt_dist(g_route.pts[i - 1], g_route.pts[i]);
        cm = d * (float)ROUTE_GRID_CM;
        ang = atan2f(
            (float)(g_route.pts[i].x - g_route.pts[i - 1].x),
            (float)(g_route.pts[i].y - g_route.pts[i - 1].y)
        ) * 57.29578f;

        ips200_show_uint(0, line_y, i, 2);
        ips200_show_float(18, line_y, cm, 4, 1);
        ips200_show_string(52, line_y, "cm");
        ips200_show_float(70, line_y, ang, 4, 1);
    }
}
