/*
 * Fliter.c
 *
 *  Created on: 2026年1月22日
 *      Author: MR
 */

#include "zf_common_headfile.h"
/*********************************************************************************************************************
 * @brief 简单一阶低通滤波函数
 * @param none
 * @example
 * @date 2026.1.22
 * @remark
 * @remark
 * @remark
 ********************************************************************************************************************/
// 一阶低通：alpha ∈ (0,1)，比如 0.02~0.1（1kHz时常用）

float ori_filter(float data, float alpha)
{
    static float y = 0.0f;      // 上一次输出（滤波状态）
    static uint8_t inited = 0;

    if (!inited) {              // 第一次进来用输入初始化，避免慢慢爬
        y = data;
        inited = 1;
    }

    y += alpha * (data - y);
    return y;
}

