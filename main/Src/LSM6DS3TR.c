/*
 * LSM6DS3TR.c
 *
 *  Created on: Sep 3, 2025
 *      Author: psybe
 */

#include "LSM6DS3TR.h"

// ---------------------- 기본 Read/Write ----------------------

__STATIC_INLINE void LSM6DS3TR_C_ReadReg(uint8_t reg_addr, uint8_t *rx_byte,
		uint8_t size) {
	uint8_t tx_byte = reg_addr | 0x80; // 읽기 플래그(MSB=1)

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &tx_byte, 1, HAL_MAX_DELAY);   // 주소 전송
	HAL_SPI_Receive(&hspi2, rx_byte, size, HAL_MAX_DELAY);  // 데이터 수신
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

__STATIC_INLINE void LSM6DS3TR_C_WriteReg(uint8_t reg_addr, uint8_t *setting,
		uint8_t size) {
	uint8_t *tx_byte = (uint8_t*) malloc(size + 1);
	tx_byte[0] = reg_addr;
	for (uint8_t i = 0; i < size; i++) {
		tx_byte[i + 1] = setting[i];
	}

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, tx_byte, size + 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	free(tx_byte);
}

// ---------------------- 편의 함수 ----------------------

// 단일 바이트 읽기
uint8_t LSM6DS3TR_C_ReadU8(uint8_t reg_addr) {
	uint8_t v;
	LSM6DS3TR_C_ReadReg(reg_addr, &v, 1);
	return v;
}

// 단일 바이트 쓰기
void LSM6DS3TR_C_WriteU8(uint8_t reg_addr, uint8_t value) {
	LSM6DS3TR_C_WriteReg(reg_addr, &value, 1);
}

// ---------------------- 초기화 ----------------------

void LSM6DS3TR_C_Init(void) {
	int16_t GyroRaw[3]={0};
	int16_t ACCRaw[3]={0};

	// 1. WHO_AM_I 확인
	uint8_t who_am_i = LSM6DS3TR_C_ReadU8(LSM6DS3TR_C_WHO_AM_I_REG);
	Custom_LCD_Printf(0, 0, "WHO_AM_I:");
	Custom_LCD_Printf(0, 1, "0x%02X", who_am_i);
	HAL_Delay(1000);
	// 2. CTRL 설정
//	LSM6DS3TR_C_ConfigCTRL3C();

	LSM6DS3TR_C_ConfigCTRL();
//
//	// 3. CTRL3_C 확인
//	LSM6DS3TR_C_CheckCTRL3C();
	LSM6DS3TR_C_CheckCTRL();

//data준비 확인
	if (LSM6DS3TR_data_ready()) {
		//데이터읽기
		LSM6_ReadGyroRaw(GyroRaw[3]);
		LSM6_ReadAccelRaw(ACCRaw[3]);
	}

}

void LSM6DS3TR_C_ConfigCTRL3C(void) {
	uint8_t v = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL3_C);

	// 원하는 비트만 세팅/클리어
	v |= BIT(CTRL3_BDU_Pos);        // BDU=1
	v |= BIT(CTRL3_IF_INC_Pos);     // IF_INC=1
	v &= ~BIT(CTRL3_SIM_Pos);        // SIM=0 (4-wire)
	v &= ~BIT(CTRL3_PP_OD_Pos);      // PP_OD=0 (push-pull)
	v &= ~BIT(CTRL3_H_LACTIVE_Pos);  // H_LACTIVE=0 (active high)
	v &= ~BIT(CTRL3_BLE_Pos);        // BLE=0

	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL3_C, CTRL3_C);
}

void LSM6DS3TR_C_ConfigCTRL(void) {
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL3_C, CTRL3_C);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL1_XL, CTRL1_XL);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL2_G, CTRL2_G);
//312순서를권장함
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL4_C, CTRL4_C);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL5_C, CTRL5_C);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL6_C, CTRL6_C);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL7_G, CTRL7_G);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL8_XL, CTRL8_XL);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL9_XL, CTRL9_XL);
	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL10_C, CTRL10_C);
}

void LSM6DS3TR_C_CheckCTRL3C(void) {
	uint8_t v = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL3_C);

	// 기대하는 값 계산
	uint8_t expected = 0;
	expected |= BIT(CTRL3_BDU_Pos);
	expected |= BIT(CTRL3_IF_INC_Pos);
	while (1) {
		if ((v & expected) == expected) {
			Custom_LCD_Printf(0, 2, "CTRL3_C OK");
			Custom_LCD_Printf(0, 3, "0x%02X", v);
		} else {
			Custom_LCD_Printf(0, 2, "CTRL3_C ERR: 0x%02X");
			Custom_LCD_Printf(0, 3, "0x%02X", v);
		}
	}
}

void LSM6DS3TR_C_CheckCTRL() {
	uint8_t read_ctrl3 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL3_C);
	uint8_t read_ctrl1 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL1_XL);
	uint8_t read_ctrl2 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL2_G);
	uint8_t read_ctrl4 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL4_C);
	uint8_t read_ctrl5 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL5_C);
	uint8_t read_ctrl6 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL6_C);
	uint8_t read_ctrl7 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL7_G);
	uint8_t read_ctrl8 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL8_XL);
	uint8_t read_ctrl9 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL9_XL);
	uint8_t read_ctrl10 = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL10_C);

	if (read_ctrl3 == CTRL3_C) {
		Custom_LCD_Printf(0, 2, "CTRL3 OK");
		Custom_LCD_Printf(0, 3, "0x%02X", read_ctrl3);
	} else {
		Custom_LCD_Printf(0, 2, "CTRL3_C ERR");
		Custom_LCD_Printf(0, 3, "0x%02X", read_ctrl3);
	}
	HAL_Delay(200);
	if (read_ctrl1 == CTRL1_XL) {
		Custom_LCD_Printf(0, 4, "CTRL1 OK");
		Custom_LCD_Printf(0, 5, "0x%02X", read_ctrl1);
	} else {
		Custom_LCD_Printf(0, 4, "CTRL1_XL ERR");
		Custom_LCD_Printf(0, 5, "0x%02X", read_ctrl1);
	}
	HAL_Delay(200);
	if (read_ctrl2 == CTRL2_G) {
		Custom_LCD_Printf(0, 6, "CTRL2 OK");
		Custom_LCD_Printf(0, 7, "0x%02X", read_ctrl2);
	} else {
		Custom_LCD_Printf(0, 6, "CTRL2_XL ERR");
		Custom_LCD_Printf(0, 7, "0x%02X", read_ctrl2);
	}
	HAL_Delay(2000);
	if (read_ctrl4 == CTRL4_C) {
		Custom_LCD_Printf(0, 2, "CTRL4 OK");
		Custom_LCD_Printf(0, 3, "0x%02X", read_ctrl4);
	} else {
		Custom_LCD_Printf(0, 2, "CTRL3_C ERR");
		Custom_LCD_Printf(0, 3, "0x%02X", read_ctrl4);
	}
	HAL_Delay(200);
	if (read_ctrl5 == CTRL5_C) {
		Custom_LCD_Printf(0, 4, "CTRL1 OK");
		Custom_LCD_Printf(0, 5, "0x%02X", read_ctrl5);
	} else {
		Custom_LCD_Printf(0, 4, "CTRL1_XL ERR");
		Custom_LCD_Printf(0, 5, "0x%02X", read_ctrl5);
	}
	HAL_Delay(200);
	if (read_ctrl6 == CTRL6_C) {
		Custom_LCD_Printf(0, 6, "CTRL6 OK");
		Custom_LCD_Printf(0, 7, "0x%02X", read_ctrl6);
	} else {
		Custom_LCD_Printf(0, 6, "CTRL6_XL ERR");
		Custom_LCD_Printf(0, 7, "0x%02X", read_ctrl6);
	}
	HAL_Delay(2000);
	if (read_ctrl7 == CTRL7_G) {
		Custom_LCD_Printf(0, 2, "CTRL7 OK");
		Custom_LCD_Printf(0, 3, "0x%02X", read_ctrl7);
	} else {
		Custom_LCD_Printf(0, 2, "CTRL7_C ERR");
		Custom_LCD_Printf(0, 3, "0x%02X", read_ctrl7);
	}
	HAL_Delay(200);
	if (read_ctrl8 == CTRL8_XL) {
		Custom_LCD_Printf(0, 4, "CTRL8 OK");
		Custom_LCD_Printf(0, 5, "0x%02X", read_ctrl7);
	} else {
		Custom_LCD_Printf(0, 4, "CTRL8_XL ERR");
		Custom_LCD_Printf(0, 5, "0x%02X", read_ctrl5);
	}
	HAL_Delay(200);
	if (read_ctrl9 == CTRL9_XL) {
		Custom_LCD_Printf(0, 6, "CTRL9 OK");
		Custom_LCD_Printf(0, 7, "0x%02X", read_ctrl9);
	} else {
		Custom_LCD_Printf(0, 6, "CTRL9 ERR");
		Custom_LCD_Printf(0, 7, "0x%02X", read_ctrl9);
	}
	HAL_Delay(2000);
	if (read_ctrl10 == CTRL10_C) {
		Custom_LCD_Printf(0, 2, "CTRL10 OK");
		Custom_LCD_Printf(0, 3, "0x%02X", read_ctrl10);
	} else {
		Custom_LCD_Printf(0, 2, "CTRL10_C ERR");
		Custom_LCD_Printf(0, 3, "0x%02X", read_ctrl10);
	}

}

uint8_t LSM6DS3TR_data_ready() {
	uint8_t s = LSM6DS3TR_C_ReadU8(LSM6DS3_STATUS_REG);
	return (s & 0x03) != 0;
}

/* 데이터 읽기 */
HAL_StatusTypeDef LSM6_ReadGyroRaw(int16_t g[3]) {
	uint8_t b[6];
	HAL_StatusTypeDef st = LSM6DS3TR_C_ReadReg(LSM6DS3_OUTX_L_G, b, 6);
	if (st != HAL_OK)
		return st;
	g[0] = LSM6_Merge16(b[0], b[1]);
	g[1] = LSM6_Merge16(b[2], b[3]);
	g[2] = LSM6_Merge16(b[4], b[5]);
	return HAL_OK;
}

HAL_StatusTypeDef LSM6_ReadAccelRaw(int16_t a[3]) {
	uint8_t b[6];
	HAL_StatusTypeDef st = LSM6DS3TR_C_ReadReg(LSM6DS3_OUTX_L_XL, b, 6);
	if (st != HAL_OK)
		return st;
	a[0] = LSM6_Merge16(b[0], b[1]);
	a[1] = LSM6_Merge16(b[2], b[3]);
	a[2] = LSM6_Merge16(b[4], b[5]);
	return HAL_OK;
}

HAL_StatusTypeDef LSM6_ReadGA12(int16_t g[3], int16_t a[3]) {
	uint8_t b[12];
	HAL_StatusTypeDef st = LSM6DS3TR_C_ReadReg(LSM6DS3_OUTX_L_G, b,
			sizeof(b)); // 0x22~0x2D
	if (st != HAL_OK)
		return st;
	g[0] = LSM6_Merge16(b[0], b[1]);
	g[1] = LSM6_Merge16(b[2], b[3]);
	g[2] = LSM6_Merge16(b[4], b[5]);
	a[0] = LSM6_Merge16(b[6], b[7]);
	a[1] = LSM6_Merge16(b[8], b[9]);
	a[2] = LSM6_Merge16(b[10], b[11]);
	return HAL_OK;
}
