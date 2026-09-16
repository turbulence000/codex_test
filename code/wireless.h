#ifndef WIRELESS_H_
#define WIRELESS_H_

#include "zf_common_headfile.h"

void wu_send_str(const char *s); // 发送字符串
void uart_rx_process(void);      // 串口接收处理
void wireless_tick_5ms(void);    // 保留兼容（参数改动后已立即保存，此函数为空）
void wireless_send_data(void);   //无线串口发送数据
#endif
