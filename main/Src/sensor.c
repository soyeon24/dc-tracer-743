/*
 * sensor.c
 *
 *  Created on: Apr 23, 2025
 *      Author: kth59
 */

#include "init.h"
#include "sensor.h"
#include "lcd.h"
#include "motor.h"
#include "setting.h"
#include "../../MDK-ARM/Inc/adc.h"
#include "../../MDK-ARM/Inc/custom_switch.h"
#include "../../MDK-ARM/Inc/lptim.h"
#include "../../MDK-ARM/Inc/main.h"
#include "../../MDK-ARM/Inc/tim.h"
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

int32_t positionValue = 0;
int32_t weightedSum = 0;
int32_t weightedNormalized = 0;

uint32_t sensorRaw[16] = { 0 };
uint32_t whiteMax[16] = { 0 };
uint32_t blackMax[16] = { 0 };
uint32_t sensorNormalized[16] = { 0 };
uint8_t center = 7;
uint16_t sensorState = 0;
uint32_t sensorThreshold = 45;
uint8_t windowStartIndex = 17;
uint8_t windowEndIndex = 0;
window_t Window;
int32_t sensorPosition[16] = {
//
		-30000, -26000, -22000, -18000, -14000, -10000, -6000, -2000,
		//
		2000, 6000, 10000, 14000, 18000, 22000, 26000, 30000 };

uint32_t MARK_STATE_LEFT[16] = { 0x0000, 0x0000, 0x0000, 0x8000, 0xC000, 0xE000,
		0xF000, 0xF800, 0xFC00, 0xFE00, 0xFF00, 0xFF80, 0xFFC0, 0xFFE0, 0xFFF0,
		0xFFF0 //
		};

uint32_t MARK_STATE_RIGHT[16] = { 0x0FFF, 0x07FF, 0x03FF, 0x01FF, 0x00FF,
		0x007F, 0x003F, 0x001F, 0x000F, 0x0007, 0x0003, 0x0001, 0x0000, 0x0000,
		0x0000, 0x0000 //
		};

uint32_t MARK_STATE_CENTER[16] = { 0xF000, 0xF100, 0xFC00, 0x7E00, 0x3F00,
		0x1F80, 0x0FC0, 0x07E0, 0x03F0, 0x01F8, 0x00FC, 0x007E, 0x003F, 0x001F,
		0x000F, 0x000F //
		};

menu_t sensor_menu[] = { { "1.S Raw", Sensor_Test_Raw }, { "2.S Norm",
		Sensor_Normalized }, { "3.S state", Sensor_State }, { "4.S posit",
		Position_Test }, { "5.S Test ", Sensor_Test_Menu }, { "6.adcread",
		Battery_LCD }, { "cali", Sensor_Calibration }, { "8. back ",
		Back_To_Menu } };

float_t batteryVolt;

uint32_t SENSOR_MENU_CNT = (sizeof(sensor_menu) / sizeof(menu_t));

__STATIC_INLINE uint32_t Battery_ADC_Read() {
	//Custom_LCD_Printf(0, 0,"batt adc");

	static uint32_t adcValue = 0;
	__disable_irq();
	HAL_ADC_Start(&hadc2);

	if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK) {
		adcValue = HAL_ADC_GetValue(&hadc2);

	}
//		else {
//		Custom_LCD_Printf(0, 1, "poll f");
//	}

	HAL_ADC_Stop(&hadc2);
	__enable_irq();
	return adcValue;
}

uint32_t adc = 0;


int window;
void Battery_LPTIM3_IRQ() {
	//Custom_LCD_Printf(0, 4, "tim3");
	adc = Battery_ADC_Read();
	batteryVolt = ((float) adc) * 0.000849f - 0.866f;
	//Custom_LCD_Printf(0, 0, "%f", batteryVolt);
}

void Battery_LCD() {
//	Custom_LCD_Printf(0,0,"in");
	Battery_Start();
	//HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 1);
	//Custom_LCD_Printf(5,0,"start ok");
	HAL_GPIO_WritePin(E3_GPIO_Port, GPIO_PIN_3, GPIO_PIN_SET);
	while (1) {
		Custom_LCD_Printf(0, 0, "%5f", batteryVolt);
		Custom_LCD_Printf(0, 1, "%d", adc);

	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
	}
//	while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
//
//	}
//	HAL_GPIO_WritePin(E3_GPIO_Port, GPIO_PIN_3, GPIO_PIN_SET);
//
//	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
//		Custom_LCD_Printf(1, 6, "E");
//	}

	Battery_Stop();
}

void Battery_Start() {

//	Custom_LCD_Printf(0,0,"in");

	HAL_LPTIM_Counter_Start_IT(&hlptim3, 0);
	//Custom_LCD_Printf(0,0,"in");
}
void Battery_Stop() {

	HAL_LPTIM_Counter_Stop_IT(&hlptim3);
}
void Sensor_Start() {
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_Delay(10);
}
void Sensor_Stop() {
	HAL_TIM_Base_Stop_IT(&htim6);
	HAL_Delay(10);
}
void Sensor_Menu_Print(uint32_t index) {
	for (uint32_t i = 0; i < SENSOR_MENU_CNT; i++) {
		Set_Color(index, i);
		Custom_LCD_Printf(0, i + 1, "%s", (sensor_menu + i)->name);

	}
}
void Sensor_Test_Menu() {
	uint32_t numOfSensorMenu = sizeof(sensor_menu) / sizeof(menu_t);
	uint32_t selected_index = 0;
	uint32_t sw = CUSTOM_JS_NONE;
	while (1) {
		POINT_COLOR = WHITE;
		BACK_COLOR = BLACK;

		while (1) {
			Custom_LCD_Printf(0, 0, "Sensor M");
			Battery_LCD_Picture();
			Set_Color(numOfSensorMenu, selected_index);
			Sensor_Menu_Print(selected_index);
			sw = Custom_Switch_Read();
			if (sw == CUSTOM_JS_U_TO_D) {
				selected_index++;
				selected_index = (selected_index % 8);
			} else if (sw == CUSTOM_JS_D_TO_U) {
				selected_index--;
				selected_index = (selected_index % 8);
			} else if (sw == CUSTOM_JS_L_TO_R) {
				Custom_LCD_Clear();
				(sensor_menu + selected_index)->func();
			}
			Set_Color(numOfSensorMenu, selected_index);

		}
	}
}
int windowStart;
int windowSizeHalf = 3;
void TIM6_Sensor_IRQ() {
	static uint8_t i = 0;
	uint32_t rawValue1;
	uint32_t rawValue2;
	uint32_t rawValue3;
	uint32_t rawValue;

	HAL_GPIO_WritePin(Sensor_MUX4_GPIO_Port, Sensor_MUX4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Sensor_MUX0_GPIO_Port, Sensor_MUX0_Pin, i & 0x1);
	HAL_GPIO_WritePin(Sensor_MUX1_GPIO_Port, Sensor_MUX1_Pin, i & 0x2);
	HAL_GPIO_WritePin(Sensor_MUX2_GPIO_Port, Sensor_MUX2_Pin, i & 0x4);
	HAL_GPIO_WritePin(Sensor_MUX3_GPIO_Port, Sensor_MUX3_Pin, i & 0x8);
	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
		rawValue1 = HAL_ADC_GetValue(&hadc1);
		rawValue2 = HAL_ADC_GetValue(&hadc1);
		rawValue3 = HAL_ADC_GetValue(&hadc1);
	}
	if ((rawValue1 >= rawValue2 && rawValue1 <= rawValue3)
			|| (rawValue1 <= rawValue2 && rawValue1 >= rawValue3)) {
		rawValue = rawValue1;
	} else if ((rawValue2 >= rawValue1 && rawValue2 <= rawValue3)
			|| (rawValue2 <= rawValue1 && rawValue2 >= rawValue3)) {
		rawValue = rawValue2;
	} else {
		rawValue = rawValue3;
	}

	HAL_ADC_Stop(&hadc1);

	HAL_GPIO_WritePin(Sensor_MUX4_GPIO_Port, Sensor_MUX4_Pin, GPIO_PIN_RESET);

	*(sensorRaw + i) = rawValue;
	if (rawValue < blackMax[i]) {
		sensorNormalized[i] = 0;
	} else if (rawValue > whiteMax[i]) {
		sensorNormalized[i] = 255;
	} else {
		sensorNormalized[i] = 255 * (sensorRaw[i] - blackMax[i])
				/ (whiteMax[i] - blackMax[i]);
	}
	sensorState = (sensorState & ~(0x01 << (15 - i)))
			| ((sensorNormalized[i] > sensorThreshold) << (15 - i));

//	window = (positionValue + 30000) / 4000;
//
//	weightedSum = 0;
//	weightedNormalized = 0;
//	windowStart = MAX(window - windowSizeHalf + 1, 0);
//	int windowEnd = MIN(window + windowSizeHalf, 15);
//	for (int i = windowStart; i <= windowEnd; i++) {
//		weightedNormalized += sensorNormalized[i] * sensorPosition[i];
//		weightedSum += sensorNormalized[i];
//	}
//	if (!weightedSum) {
//		positionValue = 0;
//	} else {
//		positionValue = weightedNormalized / weightedSum;
//	}
//	Window.center = MARK_STATE_CENTER[window];
//	Window.left = MARK_STATE_LEFT[window];
//	Window.right = MARK_STATE_RIGHT[window];
	if (!(i % 15)) {
		weightedSum = 0;
		weightedNormalized = 0;

		float windowStart = positionValue / 4000 + 4.5;
		float windowEnd = positionValue / 4000 + 10.5;
		uint8_t currentWindowStartIndex = 17;
		uint8_t currentWindowEndIndex = 0;
		for (int sensorIndex = 0; sensorIndex < 16; sensorIndex++) {
			if (sensorIndex < windowStart) {
				continue;
			}
			if (sensorIndex > windowEnd) {
				continue;
			}
			if (sensorIndex < currentWindowStartIndex) {
				currentWindowStartIndex = sensorIndex;
			}
			if (sensorIndex > currentWindowEndIndex) {
				currentWindowEndIndex = sensorIndex;
			}
			weightedSum += sensorPosition[sensorIndex]
					* ((int32_t) sensorNormalized[sensorIndex]);
			weightedNormalized += sensorNormalized[sensorIndex];

		}
		if (!weightedNormalized) {
			positionValue = 0;
		} else {
			positionValue = weightedSum / weightedNormalized;
		}

		windowStartIndex = currentWindowStartIndex;
		windowEndIndex = currentWindowEndIndex;

//		Window.center = 0;
//		Window.left = 0;
//		Window.right = 0;
//		for (int j = windowStartIndex; j <= windowEndIndex; j++) {
//			Window.center |= (1 << (15 - j));
//		}
//		Window.left = ((uint16_t) (0xffff)) << (16 - windowStartIndex);
//		Window.right = ((uint16_t) (0xffff)) >> (windowEndIndex + 1);
	}

	i = (i + 1) & 0x0f;
}

void Sensor_Printf(const char *name, uint32_t *sensor) {
	Custom_LCD_Printf(0, 0, "%s", name);
	Custom_LCD_Printf(0, 1, "%02x %02x %02x %02x", sensor[0], sensor[1],
			sensor[2], sensor[3]);
	Custom_LCD_Printf(0, 2, "%02x %02x %02x %02x", sensor[4], sensor[5],
			sensor[6], sensor[7]);
	Custom_LCD_Printf(0, 3, "%02x %02x %02x %02x", sensor[8], sensor[9],
			sensor[10], sensor[11]);
	Custom_LCD_Printf(0, 4, "%02x %02x %02x %02x", sensor[12], sensor[13],
			sensor[14], sensor[15]);
}

void Sensor_Printf_16(const char *name, uint32_t *sensor) {
	Custom_LCD_Printf(0, 0, "%s", name);
	Custom_LCD_Printf(0, 1, "%02x %02x %02x", sensor[0] >> 8, sensor[1] >> 8,
			sensor[2] >> 8);
	Custom_LCD_Printf(0, 2, "%02x %02x %02x", sensor[3] >> 8, sensor[4] >> 8,
			sensor[5] >> 8);
	Custom_LCD_Printf(0, 3, "%02x %02x %02x", sensor[6] >> 8, sensor[7] >> 8,
			sensor[8] >> 8);
	Custom_LCD_Printf(0, 4, "%02x %02x %02x", sensor[9] >> 8, sensor[10] >> 8,
			sensor[11] >> 8);
	Custom_LCD_Printf(0, 5, "%02x %02x %02x", sensor[12] >> 8, sensor[13] >> 8,
			sensor[14] >> 8);
	Custom_LCD_Printf(0, 6, "%02x ", sensor[15] >> 8);
}

void Sensor_Test_Raw2() {
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
	Sensor_Start();
	while (1) {
		Sensor_Printf_16("S Raw", sensorRaw);

		if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
			while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
				;
			break;
		}
	}

	Sensor_Stop();

	Custom_LCD_Clear();
}

void Senor_Raw_Draw_Bar_Graph(uint32_t *sensor) {
	for (uint32_t i = 0; i < 16; i++) {
		//상자
		ST7735_LCD_Driver.FillRect(&st7735_pObj, 1, 1 + 10 * i, 1, 8, 0xFFFF); //왼쪽 세로

		ST7735_LCD_Driver.FillRect(&st7735_pObj, 43, 1 + 10 * i, 1, 8, 0xFFFF);	//오른쪽 세로

		ST7735_LCD_Driver.FillRect(&st7735_pObj, 1, 1 + 10 * i, 43, 1, 0xFFFF);	//위 가로

		ST7735_LCD_Driver.FillRect(&st7735_pObj, 1, 8 + 10 * i, 43, 1, 0xFFFF);	//아래 가로

		uint32_t raw = (uint32_t) ((sensor[i] >> 8) * 39 / 256);
		if ((i + 1) % 2) {
			Custom_LCD_Printf(6, i / 2, "%02x", sensor[i] >> 8);	//센서 막대
		} else {
			Custom_LCD_Printf(8, i / 2, "%02x", sensor[i] >> 8);
		}
		ST7735_LCD_Driver.FillRect(&st7735_pObj, 3, 3 + 10 * i, raw, 4, 0x7DF6);
		ST7735_LCD_Driver.FillRect(&st7735_pObj, raw + 4, 3 + 10 * i, 38 - raw,
				4, BLACK);

	}
}

void In_Box() {
	for (uint32_t i = 0; i < 16; i++) {
//		ST7735_LCD_Driver.FillRect(&st7735_pObj, 3, 3 + 16*i,(uint32_t)((sensorRaw[i]>>8)*39/255), 10, 0x7DF6);
		//	Custom_LCD_Printf(8, i, "02x", sensorRaw[i]>>8);
	}

}

void Sensor_Test_Raw() {
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, 0);
	Sensor_Start();
	while (1) {
		Senor_Raw_Draw_Bar_Graph(sensorRaw);
		In_Box();

		if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
			while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
				;
			break;
		}
	}

	Sensor_Stop();

	Custom_LCD_Clear();
}

void WhiteMax_Draw_Bar_Graph(uint32_t *sensor) {
	for (uint32_t i = 0; i < 16; i++) {
		//상자
		ST7735_LCD_Driver.FillRect(&st7735_pObj, 1, 1 + 10 * i, 1, 8, 0xFFFF);//왼쪽 세로

		ST7735_LCD_Driver.FillRect(&st7735_pObj, 43, 1 + 10 * i, 1, 8, 0xFFFF);	//오른쪽 세로

		ST7735_LCD_Driver.FillRect(&st7735_pObj, 1, 1 + 10 * i, 43, 1, 0xFFFF);	//위 가로

		ST7735_LCD_Driver.FillRect(&st7735_pObj, 1, 8 + 10 * i, 43, 1, 0xFFFF);	//아래 가로

		uint32_t cali = (uint32_t) ((sensor[i] >> 8) * 39 / 256);
		if ((i + 1) % 2) {
			Custom_LCD_Printf(6, i / 2, "%02x", sensor[i] >> 8);	//센서 막대
		} else {
			Custom_LCD_Printf(8, i / 2, "%02x", sensor[i] >> 8);
		}
		ST7735_LCD_Driver.FillRect(&st7735_pObj, 3, 3 + 10 * i, cali, 4,
				0x8E37);

	}
}

void Black_Draw_Bar_Graph(uint32_t *sensor) {
	for (uint32_t i = 0; i < 16; i++) {

		uint32_t cali = (uint32_t) ((sensor[i] >> 8) * 39 / 256);
		if ((i + 1) % 2) {
			Custom_LCD_Printf(6, i / 2, "%02x", sensor[i] >> 8);	//센서 막대
		} else {
			Custom_LCD_Printf(8, i / 2, "%02x", sensor[i] >> 8);
		}
		ST7735_LCD_Driver.FillRect(&st7735_pObj, 3, 3 + 10 * i, cali, 4,
				0x5575);

	}
}

void Sensor_Calibration() {
	Sensor_Start();
	Custom_LCD_Clear();
	uint8_t sw = 0;
	while (1) {
		sw = Custom_Switch_Read();
		for (uint8_t i = 0; i < 16; i++) {
			if (sensorRaw[i] > whiteMax[i]) {
				whiteMax[i] = sensorRaw[i];
			}
		}
		WhiteMax_Draw_Bar_Graph(whiteMax);
		if (sw == CUSTOM_JS_L_TO_R) {
			break;
		}
	}
	sw = Custom_Switch_Read();
	while (1) {
		sw = Custom_Switch_Read();
		for (uint8_t i = 0; i < 16; i++) {
			if (sensorRaw[i] > blackMax[i]) {
				blackMax[i] = sensorRaw[i];
			}
		}
		Black_Draw_Bar_Graph(blackMax);
		if (sw == CUSTOM_JS_L_TO_R) {
			Custom_LCD_Clear();
			break;
		}
	}

	Sensor_Stop();

}
void Sensor_Calibration_origin() {
	Sensor_Start();
	Custom_LCD_Clear();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		for (uint8_t i = 0; i < 16; i++) {
			if (sensorRaw[i] > whiteMax[i]) {
				whiteMax[i] = sensorRaw[i];
			}
		}
		WhiteMax_Draw_Bar_Graph(whiteMax);
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
		for (uint8_t i = 0; i < 16; i++) {
			if (sensorRaw[i] > blackMax[i]) {
				blackMax[i] = sensorRaw[i];
			}
		}
		Black_Draw_Bar_Graph(blackMax);
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
	}
	Custom_LCD_Clear();
	Sensor_Stop();

}

void Sensor_Normalized() {
	Sensor_Start();
	Custom_LCD_Clear();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {

		Sensor_Printf("normal", sensorNormalized);
	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
	}
	Custom_LCD_Clear();
	Sensor_Stop();

}

void Sensor_State() {
	Sensor_Start();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {

		char upper[9] = { 0 };
		char lower[9] = { 0 };

		char upper2[9] = { 0 };
		char lower2[9] = { 0 };
		for (int i = 0; i < 8; i++) {
			upper[i] = ((sensorState >> (15 - i)) & 1) ? '1' : '0';
			lower[i] = ((sensorState >> (7 - i)) & 1) ? '1' : '0';
		}
		Custom_LCD_Printf(0, 0, "STATE");

		Custom_LCD_Printf(0, 1, upper);

		Custom_LCD_Printf(0, 2, lower);

		for (int i = 0; i < 8; i++) {
			upper2[i] = ((Window.left >> (15 - i)) & 1) ? '1' : '0';
			lower2[i] = ((Window.left >> (7 - i)) & 1) ? '1' : '0';
		}

		Custom_LCD_Printf(0, 3, upper2);

		Custom_LCD_Printf(0, 4, lower2);

	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
	}
	Sensor_Stop();
	Custom_LCD_Clear();
}

void Position_Test() {
	Sensor_Start();
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_SET) {
//		int32_t sum_of_position = 0;
//		int32_t normalized_value = 0;
//		for (int i = 0; i < 16; i++) {
//			sum_of_position += sensorPosition[i] * sensorNormalized[i];
//			normalized_value += sensorNormalized[i];
//		}
//		if (!normalized_value) {
//			positionValue = 0;
//		} else {
//			positionValue = sum_of_position / normalized_value;
//		}
		Custom_LCD_Printf(0, 0, "pos %-6d", positionValue);

	}
	while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET) {
	}
	Sensor_Stop();
}
