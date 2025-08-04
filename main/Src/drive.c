/*
 * drive.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#include "init.h"
#include "lcd.h"
#include "tim.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "lptim.h"
#include "stdbool.h"
#include "custom_switch.h"
#include "setting.h"
#define STATE_IDLE 0
#define STATE_CROSS 1
#define STATE_MARK 2
#define STATE_DECISION 3
#define STATE_CROSS_DECISION 4

#define STATE_STRAIGHT 4
#define STATE_LEFT 1
#define STATE_RIGHT 2

#define STATE_ACCEL 1
#define STATE_DECCEL 2
#define STATE_END_BERORE 4

#define ABS(x) ((x>0) ? x:(-x))

#define MARK_NONE 0
#define MARK_CROSS 8
#define MARK_LEFT 1
#define MARK_RIGHT 2
#define MARK_END 4

#define MASKLEFT (((uint32_t)0xFFFFFFFF)<<19)
#define MASKRIGHT (((uint32_t) 0xFFFFFFFF) >> 20)

int32_t positionCenter[15] = { -28000, -24000, -20000, -16000, -12000, -8000,
		-4000, 0, 4000, 8000, 12000, 16000, 20000, 24000, 28000 };

volatile float accel;
volatile float accelSetting = 4.5f;
volatile float decelSetting = 8.0f;
volatile float decel;
volatile float pitInLine = 0.3f;
volatile float curveRate = 0.000068f;
float curveDecel = 19000.f;

//state
volatile uint32_t sensorStateSum = 0;
volatile uint8_t state = STATE_IDLE;
volatile int indexMarkcnt = 0;

volatile float currentVelocity;
volatile float targetVelocity;
volatile float targetVelocitySetting = 1.f;

volatile float peakVelocity = 8.0f;

//output
uint16_t markSaveFirst[400];
uint32_t markLengthFirst[400];
uint16_t markSaveSecond[400];
uint32_t markLengthSecond[400];

uint32_t markLength[400];
uint8_t indexLength = 0;

//

//sec
uint32_t secIndexmark = 0;

uint32_t safeDistance = 600;

menu_t drive_menu[] = { { "4.encoder", Encoder_Test }, { "1.left gP ",
		Motor_Left_Gain_P }, { "2.Right gP", Motor_Right_Gain_P }, {
		"4.encoder", Encoder_Test }, { "PWM Test", Motor_Test_76EHWAN }, {
		"5.S posi", Position_Test }, { "6.S Test ", Sensor_Test_Menu }, {
		"back menu", Back_To_Menu } };

uint32_t DRIVE_MENU_CNT = (sizeof(drive_menu) / sizeof(menu_t));

void Drive_Menu_Print(uint32_t index) {
	for (uint32_t i = 0; i < DRIVE_MENU_CNT; i++) {
		Set_Color(index, i);
		Custom_LCD_Printf(0, i + 1, "%s", (drive_menu + i)->name);

	}
}

void Drive_Test_Menu() {
	uint32_t numOfDriveMenu = sizeof(drive_menu) / sizeof(menu_t);
	uint32_t selected_index = 0;
	uint32_t sw = CUSTOM_JS_NONE;
	while (1) {
		POINT_COLOR = WHITE;
		BACK_COLOR = BLACK;
		Custom_LCD_Printf(0, 0, "Drive M");
		while (1) {
			Set_Color(numOfDriveMenu, selected_index);
			Drive_Menu_Print(selected_index);
			sw = Custom_Switch_Read();
			if (sw == CUSTOM_JS_U_TO_D) {
				selected_index++;
				selected_index = (selected_index % 8);
			} else if (sw == CUSTOM_JS_D_TO_U) {
				selected_index--;
				selected_index = (selected_index % 8);
			} else if (sw == CUSTOM_JS_L_TO_R) {
				Custom_LCD_Clear();
				(drive_menu + selected_index)->func();
			}
			Set_Color(numOfDriveMenu, selected_index);

		}
	}
}

void Save_Length(uint32_t *temp, uint32_t index) {
	for (uint32_t i = 0; i < index; i++) {
		markLength[i] = temp[i];

	}
	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
		Custom_LCD_Printf(0, 0, "saved");
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
	}
	Custom_LCD_Clear();
}

void Drive_LPTIM5_IRQ() {
//	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 1);
	if (currentVelocity < targetVelocity) {
		currentVelocity += accel * 0.0005f;
		if (currentVelocity > targetVelocity) {
			currentVelocity = targetVelocity;
		}
	} else if (currentVelocity > targetVelocity) {
		currentVelocity -= decel * 0.0005f;
		if (currentVelocity < targetVelocity) {
			currentVelocity = targetVelocity;
		}
	}
	float velocity_center = currentVelocity * curveDecel
			/ (curveDecel + ABS(positionValue));
	MotorR.v = velocity_center * (1 - curveRate * (float) positionValue);
	MotorL.v = velocity_center * (1 + curveRate * (float) positionValue);

}

void Drive_Start() {
	HAL_LPTIM_Counter_Start_IT(&hlptim5, 0); // 이 함수의 period는 ARR값임. 32KHz이며 div 16 해서 2000Hz 500us면 적당함
}

void Drive_Stop() {
	HAL_LPTIM_Counter_Stop_IT(&hlptim5);
}

__STATIC_INLINE uint32_t Position_Center(int32_t position, uint16_t state) {
	int32_t position_index = (position + 30000 + 2000) / 4000;
	uint32_t extended_state = (uint32_t) (state);
	uint32_t center_sorted_state = extended_state << position_index;
	return center_sorted_state;
}

__STATIC_INLINE uint8_t State_Machine() {
	uint16_t windowMask = 0;
	for (int j = windowStartIndex; j <= windowEndIndex; j++) {
		windowMask |= (1 << (15 - j));
	}
	uint16_t markerMask = ~windowMask;
	bool isMarkerDetected = sensorState & markerMask;

	switch (state) {
	case STATE_IDLE:
		if (__builtin_popcount(sensorState & windowMask) > 4
				|| isMarkerDetected) {
			sensorStateSum = Position_Center(positionValue, sensorState);
			state = STATE_MARK;
		}
		break;

	case STATE_MARK:
		if (!isMarkerDetected) {
			state = STATE_DECISION;
			break;
		}
		sensorStateSum |= Position_Center(positionValue, sensorState);
		break;

	case STATE_DECISION:
		state = STATE_IDLE;

		bool isLeftDetected = sensorStateSum & MASKLEFT;
		bool isRightDetected = sensorStateSum & MASKRIGHT;
		bool isSensorStateFull = (sensorStateSum & 0x007FFF00) == 0x007FFF00;

		if (isSensorStateFull) {

			return MARK_CROSS;
		}

		if (isLeftDetected && isRightDetected) {
			return MARK_END;
		}

		if (isLeftDetected) {
			return MARK_LEFT;
		}

		if (isRightDetected) {
			return MARK_RIGHT;
		}
	}
	return MARK_NONE;
}

__STATIC_INLINE uint8_t Pre_State(uint8_t mark) {
	static uint8_t secondState = STATE_STRAIGHT;
	switch (secondState) {
	case STATE_STRAIGHT:
		if (mark == MARK_LEFT) {
			secondState = STATE_LEFT;
		} else if (mark == MARK_RIGHT) {
			secondState = STATE_RIGHT;
		} else {
			secondState = STATE_STRAIGHT;

		}
		break;

	case STATE_LEFT:
		if (mark == MARK_LEFT) {
			secondState = STATE_STRAIGHT;
		} else if (mark == MARK_RIGHT) {
			secondState = STATE_RIGHT;
		} else {
			secondState = STATE_LEFT;
		}
		break;
	case STATE_RIGHT:
		if (mark == MARK_LEFT) {
			secondState = STATE_LEFT;
		} else if (mark == MARK_RIGHT) {
			secondState = STATE_STRAIGHT;
		} else {
			secondState = STATE_RIGHT;
		}
		break;
	}

	return secondState;

}

void Drive_First() {
//output

	uint32_t tempMarkRead[400];
	uint32_t tempMarkLength[400];
	uint8_t endmarkCNT = 0;
	uint8_t crossCNT = 0;
	uint8_t markLeftCNT = 0;
	uint8_t markRightCNT = 0;
	uint32_t markIndex = 0;
	uint8_t mark;
	currentVelocity = 0;

//input
	uint8_t sw = CUSTOM_JS_NONE;
	accel = accelSetting;
	targetVelocity = targetVelocitySetting;
	decel = decelSetting;

	if (!(whiteMax[1] - blackMax[1])) {
		while (1) {
			POINT_COLOR = RED;
			BACK_COLOR = BLACK;
//			Custom_LCD_Printf(0, 0, "do cali");
		}
	}

	Sensor_Start();
	HAL_Delay(10);
	Motor_Start();
	Drive_Start();
	while (endmarkCNT < 2) {
//		Custom_LCD_Printf(0, 0, "in fisrt");
		mark = State_Machine();
		if (mark == MARK_END) {
			endmarkCNT++;
		} else if (mark == MARK_CROSS) {
			crossCNT++;
		} else if (mark == MARK_RIGHT) {
			markRightCNT++;
		} else if (mark == MARK_LEFT) {
			markLeftCNT++;
		}
		if (mark) {
			tempMarkRead[markIndex] = mark;
			markIndex++;
			if (!markIndex) {
				tempMarkLength[markIndex] = (MotorL.Integral + MotorR.Integral)
						/ 2 - tempMarkRead[markIndex];
			} else {
				tempMarkLength[markIndex] = (MotorL.Integral + MotorR.Integral)
						/ 2;

			}
//			Custom_LCD_Printf(0, 6, "%d", mark);

		}
		if (!sensorState) {
			break;

		}
	}

	decel = (currentVelocity * currentVelocity) / (2 * pitInLine);
	targetVelocity = 0;

	while (currentVelocity > 0.01) {
//		Custom_LCD_Printf(0, 0, "%f", currentVelocity);
//		Custom_LCD_Printf(0, 1, "%f", decel);
	}

	HAL_Delay(100);
	Drive_Stop();
	Motor_Stop();
	Sensor_Stop();
	Custom_LCD_Clear();
	while (1) {
		sw = Custom_Switch_Read();
		Custom_LCD_Printf(0, 0, "Save down");
		Custom_LCD_Printf(0, 1, "Left %d", markLeftCNT);
		Custom_LCD_Printf(0, 2, "Right %d", markRightCNT);
		Custom_LCD_Printf(0, 3, "cross %d", crossCNT);
		Custom_LCD_Printf(0, 4, "end %d", endmarkCNT);
		Custom_LCD_Printf(0, 7, "X up");
		if (sw == CUSTOM_JS_U_TO_D) {
			for (int i = 0; i < markIndex; i++) {
				markSaveFirst[i] = tempMarkRead[i];
				markLengthFirst[i] = tempMarkLength[i];
			}
			break;
		} else if (sw == CUSTOM_JS_D_TO_U) {
			for (int i = 0; i < markIndex; i++) {
				tempMarkRead[i] = 0;
				tempMarkLength[i] = 0;
			}
			break;
		}
	}
}

__STATIC_INLINE void Second_State_Machine(uint8_t currentState, uint8_t mark,
		uint32_t index) {
	static uint8_t secondState = STATE_STRAIGHT;
	switch (secondState) {
	case STATE_IDLE:
		if (currentState == STATE_STRAIGHT) {
			secondState = STATE_ACCEL;
		}
		break;

	case STATE_ACCEL:
		uint32_t currentTick = (MotorL.Integral + MotorL.Integral) / 2;
		uint32_t decelTick = (uint32_t) (currentVelocity * currentVelocity
				- targetVelocity * targetVelocity) / decel;
		if (markLengthFirst[secIndexmark] < safeDistance) {
			secondState = STATE_DECCEL;
		} else if (currentTick
				> markLengthFirst[index] - safeDistance - decelTick) {
			secondState = STATE_DECCEL;
		} else if (currentTick > safeDistance + decelTick) {
			targetVelocity = peakVelocity;
		}
		break;
	case STATE_DECCEL:
		targetVelocity = targetVelocitySetting;
		secondState = STATE_IDLE;
		break;
	}

}
void First_Drive_Mark_Debug() {
	uint8_t sw = CUSTOM_JS_NONE;
	uint32_t i = 0;

	while (1) {
		sw = Custom_Switch_Read();
		if (sw == CUSTOM_JS_L_TO_R) {
			i++;
			Custom_LCD_Printf(0, 0, "%d", markSaveFirst[i]);
		} else if (sw == CUSTOM_JS_R_TO_L) {
			i--;
			Custom_LCD_Printf(0, 0, "%d", markSaveFirst[i]);
		} else if (sw == CUSTOM_JS_U_TO_D) {
			break;
		}
	}
}

void Drive_Second() {
	//output
	uint32_t tempMarkRead[400];
	uint32_t tempMarkLength[400];
	uint8_t secEndmarkCNT = 0;
	uint8_t secCrossCNT = 0;
	uint8_t secMarkLeftCNT = 0;
	uint8_t secMarkRightCNT = 0;

	uint8_t mark;
	currentVelocity = 0;
	//Input
	accel = accelSetting;
	targetVelocity = targetVelocitySetting;
	decel = decelSetting;

	Sensor_Start();
	Motor_Start();
	Drive_Start();
	bool secDrive = 1;
	while (secEndmarkCNT < 2) {
		mark = State_Machine();

		if ((markSaveFirst[secIndexmark] != mark) && (mark > 0) && (mark < 4)) {
			secDrive = 0;
			targetVelocity = targetVelocitySetting;
		}

		if (secDrive) {

		}
		if (mark == MARK_END) {
			secEndmarkCNT++;
			tempMarkLength[secIndexmark] = (MotorL.Integral + MotorR.Integral)
					/ 2;
		} else if (mark == MARK_CROSS) {
			secCrossCNT++;
			tempMarkLength[secIndexmark] = (MotorL.Integral + MotorR.Integral)
					/ 2;
		} else if (mark == MARK_LEFT) {
			secMarkLeftCNT++;
			tempMarkLength[secIndexmark] = (MotorL.Integral + MotorR.Integral)
					/ 2;
		} else if (mark == MARK_RIGHT) {
			secMarkRightCNT++;
			tempMarkLength[secIndexmark] = (MotorL.Integral + MotorR.Integral)
					/ 2;
		}

	}

}

void Drive_Third() {

}

void Drive_Forth() {

}

void Drive_Test_Without_Motor() {
	accel = accelSetting;
	targetVelocity = targetVelocitySetting;
	decel = decelSetting;

	uint8_t endmarkCNT = 0;
	uint8_t mark = MARK_NONE;

	Sensor_Start();
	HAL_Delay(10);
	Motor_Start();
	Drive_Start();
	while (endmarkCNT < 2) {
		//		Custom_LCD_Printf(0, 0, "in fisrt");
		mark = State_Machine();
		if (mark == MARK_END) {
			endmarkCNT++;
		}

		if (!sensorState) {
			break;

		}

		char upper[9] = { 0 };
		char lower[9] = { 0 };
		for (int i = 0; i < 8; i++) {
			upper[i] = ((sensorState >> (15 - i)) & 1) ? '1' : '0';
			lower[i] = ((sensorState >> (7 - i)) & 1) ? '1' : '0';
		}
		Custom_LCD_Printf(0, 0, "STATE");

		Custom_LCD_Printf(0, 1, upper);

		Custom_LCD_Printf(0, 2, lower);
		Custom_LCD_Printf(0, 3, "%6d", positionValue);
		Custom_LCD_Printf(0, 4, "Left %f", MotorL.v);
		Custom_LCD_Printf(0, 5, "Right %f", MotorR.v);

	}

	decel = (currentVelocity * currentVelocity) / (2 * pitInLine);
	targetVelocity = 0;

	Custom_LCD_Clear();
	while (currentVelocity > 0.01) {
		Custom_LCD_Printf(0, 0, "%f", currentVelocity);
		Custom_LCD_Printf(0, 1, "%f", decel);
	}

	HAL_Delay(100);
	Drive_Stop();
	Motor_Stop();
	Sensor_Stop();
	Custom_LCD_Clear();

}
