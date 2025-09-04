/*
 * LSM6DS3TR.c
 *
 *  Created on: Sep 3, 2025
 *      Author: psybe
 */

#include "LSM6DS3TR.h"

#define MSB_BIT1 0x80
#define LSM6DS3_CTRL3_C      0x12
#define CTRL3_BOOT_Pos       7
#define CTRL3_BDU_Pos        6
#define CTRL3_H_LACTIVE_Pos  5
#define CTRL3_PP_OD_Pos      4
#define CTRL3_SIM_Pos        3
#define CTRL3_IF_INC_Pos     2
#define CTRL3_BLE_Pos        1
#define CTRL3_SW_RESET_Pos   0

#define BIT(n) (1U<<(n))


__STATIC_INLINE void LSM6DS3TR_C_ReadReg(uint8_t reg_addr, uint8_t *rx_byte, uint8_t size) {
   uint8_t tx_byte = reg_addr | 0x80; // Read operation (MSB = 1)//주소 7비트 최상위비트가 0이면 데이터를 쓰고 1이면 읽는다

   HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); // CS Low
   HAL_SPI_Transmit(&hspi2, &tx_byte, 1, HAL_MAX_DELAY);//hspi2는버퍼라고 생각 자이로에서전솔할 주소에 최상위비트1로 해서 보냄
   HAL_SPI_Receive(&hspi2, rx_byte, size, HAL_MAX_DELAY);//버퍼에서 값을 받아오는데, 그 값은 stm의 래지스터에 저장
   HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);  // CS High
}

__STATIC_INLINE void LSM6DS3TR_C_WriteReg(uint8_t reg_addr, uint8_t *setting, uint8_t size) {
   uint8_t *tx_byte;
   tx_byte = (uint8_t *)malloc(size * sizeof(uint8_t) + 1);//size +1  만큼의 메모리 확보
   tx_byte[0] = reg_addr;//0번째에는 주소 저장
   for(uint8_t i = 0; i < size; i++){//이후부터는 setting 저장
      *(tx_byte + i + 1) = *(setting + i);
   }

   HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
   HAL_SPI_Transmit(&hspi2, tx_byte, size + 1, HAL_MAX_DELAY);
   HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
   free(tx_byte);
}

void LSM6DS3TR_C_Init(void) {
   // 1. WHO_AM_I 레지스터 읽어서 장치 확인
	uint8_t who_am_i = 0;
	while (1) {
      LSM6DS3TR_C_ReadReg(LSM6DS3TR_C_WHO_AM_I_REG, &who_am_i, 1);
      Custom_LCD_Printf(0, 0, "%x", who_am_i);
    }
	LSM6DS3TR_C_ConfigCTRL3C();
	LSM6DS3TR_C_CheckCTRL3C();
}

//void LSM6DS3TR_C_ConfigCTRL3C(void) {
//    uint8_t v = LSM6DS3TR_C_ReadReg(LSM6DS3_CTRL3_C);
//
//    // 원하는 비트만 세팅/클리어
//    v |=  BIT(CTRL3_BDU_Pos);    // BDU=1
//    v |=  BIT(CTRL3_IF_INC_Pos); // IF_INC=1
//    v &= ~BIT(CTRL3_SIM_Pos);    // SIM=0 (4-wire)
//    v &= ~BIT(CTRL3_PP_OD_Pos);  // PP_OD=0 (push-pull)
//    v &= ~BIT(CTRL3_H_LACTIVE_Pos); // H_LACTIVE=0 (active high)
//    v &= ~BIT(CTRL3_BLE_Pos);    // BLE=0
//
//    LSM6DS3TR_C_WriteReg(LSM6DS3_CTRL3_C, &v, 1);
//}
//
//void LSM6DS3TR_C_CheckCTRL3C(void) {
//    uint8_t v = LSM6DS3TR_C_ReadReg(LSM6DS3_CTRL3_C);
//
//    // 기대하는 값 계산 (예: BDU=1, IF_INC=1, 나머지는 0)
//    uint8_t expected = 0;
//    expected |= BIT(CTRL3_BDU_Pos);
//    expected |= BIT(CTRL3_IF_INC_Pos);
//
//    if ((v & expected) == expected) {
//        // 원하는 비트가 제대로 세팅됨
//        printf("CTRL3_C configured OK (0x%02X)\r\n", v);
//    } else {
//        // 설정 실패 → 전송 오류나 전원/통신 문제 가능
//        printf("CTRL3_C config failed, read=0x%02X\r\n", v);
//    }
//}
