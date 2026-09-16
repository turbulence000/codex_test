/*
 * Battery.h
 *
 *  Created on: 2025Äê10ÔÂ19ÈÕ
 *      Author: MR
 */

#ifndef CODE_BATTERY_H_
#define CODE_BATTERY_H_

extern float voltage;
void read_battery_voltage(adc_channel_enum battery_channel);
void Beep_Init(void);


#endif /* CODE_BATTERY_H_ */
