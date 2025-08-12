/*
 * sensor.h
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#ifndef MAIN_INC_SENSOR_H_
#define MAIN_INC_SENSOR_H_

#include <stdint.h>

#include "../../MDK-ARM/Inc/lptim.h"
#include "../../MDK-ARM/Inc/main.h"
void TIM6_Sensor_IRQ(void);
void Sensor_Test_Raw(void);
void Sensor_Test_Menu(void);
void Sensor_Calibration(void);
void Sensor_Normalized(void);
void Sensor_State(void);
void Position_Test(void);
void Back_To_Menu(void);
void Sensor_Start();
void Battery_LPTIM3_IRQ();
void Battery_LCD();
void Battery_Start();
void Battery_Stop();
void Sensor_Start();
void Sensor_Stop();

extern int32_t positionValue;
extern uint32_t sensorThreshold;

extern uint32_t MARK_STATE_LEFT[16];
extern uint32_t MARK_STATE_RIGHT[16];
extern uint32_t MARK_STATE_CENTER[16];
extern int windowStart;

extern int window;

typedef struct{
	uint16_t left;
	uint16_t center;
	uint16_t right;
}window_t;


extern uint8_t windowStartIndex;
extern uint8_t windowEndIndex;
extern uint16_t sensorState;
extern uint32_t whiteMax[16];
extern uint32_t blackMax[16];

extern window_t Window;

extern float batteryVolt;

#endif /* MAIN_INC_SENSOR_H_ */
