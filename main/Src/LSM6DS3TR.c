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
	// 1. WHO_AM_I 확인
	uint8_t who_am_i = LSM6DS3TR_C_ReadU8(LSM6DS3TR_C_WHO_AM_I_REG);
	Custom_LCD_Printf(0, 0, "WHO_AM_I:");
	Custom_LCD_Printf(0, 1, "0x%02X", who_am_i);
	// 2. CTRL3_C 설정
	LSM6DS3TR_C_ConfigCTRL3C();

	// 3. CTRL3_C 확인
	LSM6DS3TR_C_CheckCTRL3C();
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

	LSM6DS3TR_C_WriteU8(LSM6DS3_CTRL3_C, v);
}

void LSM6DS3TR_C_CheckCTRL3C(void) {
	uint8_t v = LSM6DS3TR_C_ReadU8(LSM6DS3_CTRL3_C);

	// 기대하는 값 계산
	uint8_t expected = 0;
	expected |= BIT(CTRL3_BDU_Pos);
	expected |= BIT(CTRL3_IF_INC_Pos);
	while (1) {
		if ((v & expected) == expected) {
			Custom_LCD_Printf(0, 2, "CTRL3_C OK", v);
			Custom_LCD_Printf(0, 3, "0x%02X", v);
		} else {
			Custom_LCD_Printf(0, 2, "CTRL3_C ERR: 0x%02X", v);
			Custom_LCD_Printf(0, 3, "0x%02X", v);
		}
	}
}
