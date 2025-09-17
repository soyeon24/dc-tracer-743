/*
 * LSM6DS3TR.h
 *
 *  Created on: Sep 3, 2025
 *      Author: psybe
 */

#ifndef INC_LSM6DS3TR_H_
#define INC_LSM6DS3TR_H_


#include "init.h"
#include "lcd.h"
#include "sensor.h"
#include "motor.h"
#include "drive.h"
#include "setting.h"

#include "spi.h"

#include "math.h"
#include <stdint.h>
#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "../../MDK-ARM/Inc/custom_switch.h"


#define LSM6DS3TR_C_WHO_AM_I_REG    0x0F


#define MSB_BIT1 0x80

//주소
#define LSM6DS3_CTRL1_XL   0x10
#define LSM6DS3_CTRL2_G    0x11
#define LSM6DS3_CTRL3_C    0x12
#define LSM6DS3_CTRL4_C    0x13
#define LSM6DS3_CTRL5_C    0x14
#define LSM6DS3_CTRL6_C    0x15
#define LSM6DS3_CTRL7_G    0x16
#define LSM6DS3_CTRL8_XL    0x17
#define LSM6DS3_CTRL9_XL    0x18
#define LSM6DS3_CTRL10_C    0x19

#define LSM6DS3_OUTX_L_G   0x22
#define LSM6DS3_OUTX_L_XL  0x28

#define LSM6DS3_STATUS_REG 0x1E

#define LSM6DS3_OUTX_L_G   0x22  // G: X_L, X_H, Y_L, Y_H, Z_L, Z_H 자이로
#define LSM6DS3_OUTX_L_XL  0x28  // XL: X_L, X_H, Y_L, Y_H, Z_L, Z_H 가속도




#define CTRL3_BOOT_Pos       7
#define CTRL3_BDU_Pos        6
#define CTRL3_H_LACTIVE_Pos  5
#define CTRL3_PP_OD_Pos      4
#define CTRL3_SIM_Pos        3
#define CTRL3_IF_INC_Pos     2
#define CTRL3_BLE_Pos        1
#define CTRL3_SW_RESET_Pos   0

//ODR_XL 가속도 출력 데이터 속도 Hz에 따라
#define ODR_XL_1HZ6 0x0B
#define ODR_XL_12HZ5 0x01
#define ODR_XL_26HZ 0x02
#define ODR_XL_52HZ 0x03
#define ODR_XL_104HZ 0x04
#define ODR_XL_208HZ 0x05
#define ODR_XL_416HZ 0x06
#define ODR_XL_833HZ 0x07
#define ODR_XL_1KHZ66 0x08
#define ODR_XL_3KHZ33 0x09
#define ODR_XL_6KHZ66 0x0A

#define ODR_XL_BASE ODR_XL_104HZ
#define ODR_XL (ODR_XL_BASE<<4)

//FS_XL 기본 해상도
#define FS_XL_2G 0x00
#define FS_XL_16G 0x01
#define FS_XL_4G 0x02
#define FS_XL_8G 0x03

#define FS_XL_BASE FS_XL_2G
#define FS_XL (FS_XL_BASE<<2)

//LPF1_BW_SEL
#define LPF1_BW_ODR_DIV2 0x00
#define LPF1_BW_ODR_DIV4 0x01

#define LPF1_BW_SEL_BASE LPF1_BW_ODR_DIV4
#define LPF1_BW_SEL (LPF1_BW_SEL_BASE<<1)

//BW0_XL
#define BW0_XL_1KHZ5 0x00
#define BW0_XL_400HZ 0x01

#define BW0_XL_BASE BW0_XL_400HZ
#define BW0_XL BW0_XL_BASE

//CTRL1_XL
#define CTRL1_XL (ODR_XL|FS_XL|LPF1_BW_SEL|BW0_XL)

#define BIT(n) (1U<<(n))

//CTRL2_G
//ODR_G
#define ODR_G_NONE 0x00
#define ODR_G_12HZ5 0x01
#define ODR_G_26HZ 0x02
#define ODR_G_52HZ 0x03
#define ODR_G_104HZ 0x04
#define ODR_G_208HZ 0x05
#define ODR_G_416HZ 0x06
#define ODR_G_833HZ 0x07
#define ODR_G_1KHZ66 0x08
#define ODR_G_3KHZ33 0x09
#define ODR_G_6KHZ66 0x0A

#define ODR_G_BASE ODR_G_104HZ
#define ODR_G (ODR_G_BASE<<4)

//FS_G
#define FS_G_245DPS 0x00
#define FS_G_500DPS 0x01
#define FS_G_1000DPS 0x02
#define FS_G_2000DPS 0x03

#define FS_G_BASE FS_G_500DPS //1카운트는0.0175degree/sec
#define FS_G (FS_G_BASE<<2)

//FS_125

#define FS_125_EN 0x01
#define FS_125_DIS 0x00

#define FS_125_BASE FS_125_DIS
#define FS_125 (FS_125_BASE<<1)

#define CTRL2_G (ODR_G|FS_G|FS_125)


//CTRL3 0x44
//BOOT
#define BOOT_NORMAL_MODE 0x00
#define BOOT_REBOOT_MEM 0x01

#define BOOT_BASE BOOT_NORMAL_MODE
#define BOOT (BOOT_BASE<<7)

//BDU
#define BDU_CONTINUOUS_UD 0x00
#define BDU_CONDITIONAL_UD 0x01

#define BDU_BASE BDU_CONDITIONAL_UD
#define BDU (BDU_BASE<<6)

//H_LACTIVE
#define H_LACTIVE_HIGH 0x01
#define H_LACTIVE_LOW 0x00

#define H_LACTIVE_BASE H_LACTIVE_LOW
#define H_LACTIVE (H_LACTIVE_BASE<<5)

//PP_OD
#define PP_OD_HIGH 0x00
#define PP_OD_LOW 0x01
#define PP_OD_BASE PP_OD_HIGH
#define PP_OD (PP_OD_BASE<<4)

//SIM
#define SIM_4WIRE 0x00
#define SIM_3WIRE 0x01
#define SIM_BASE SIM_4WIRE
#define SIM (SIM_BASE<<3)

//IF_INC
#define IF_INC_EN 0x01
#define IF_INC_DIS 0x00

#define IF_INC_BASE IF_INC_EN
#define IF_INC (IF_INC_BASE<<2)

//BLE
#define BLE_LSB 0x00
#define BLE_MSB 0x01

#define BLE_BASE BLE_LSB
#define BLE (BLE_BASE<<1)

//SW_RESET
#define SW_RESET_NORMAL_MODE 0x00
#define SW_RESET_RESET_DEVICE 0x01

#define SW_RESET_BASE SW_RESET_NORMAL_MODE
#define SW_RESET ((SW_RESET_BASE)<<0)

//CTRL3_C
#define CTRL3_C (BOOT|BDU|H_LACTIVE|PP_OD|SIM|IF_INC|BLE|SW_RESET)

//CRRL4_C 0x06
//DEN_XL_EN
#define DEN_XL_EN_EN 0x01
#define DEN_XL_EN_DIS 0x00

#define DEN_XL_EN_BASE DEN_XL_EN_DIS
#define DEN_XL_EN (DEN_XL_EN_BASE<<7)
//SLEEP
#define GYRO_SLEEP_EN 0x01
#define GYRO_SLEEP_DIS 0x00

#define GYRO_SLEEP_BASE GYRO_SLEEP_DIS
#define GYRO_SLEEP (GYRO_SLEEP_BASE<<6)

//DEN_DRDY_INT1
#define DEN_DRDY_INIT1_EN 0x01
#define DEN_DRDY_INIT1_DIS 0x00

#define DEN_DRDY_INIT1_BASE DEN_DRDY_INIT1_DIS
#define DEN_DRDY_INIT1 (DEN_DRDY_INIT1_BASE<<5)

//INT2_ON_INT1
#define INT2_ON_INT1_IT_DIV_EN 0x00
#define INT2_ON_INT1_ALL_IT 0x01

#define INT2_ON_INT1_BASE INT2_ON_INT1_IT_DIV_EN
#define INT2_ON_INT1 (INT2_ON_INT1_BASE<<4)

//DRDY_MASK
#define DRDY_MASK_DATIMER_DIS 0x00
#define DRDY_MASK_DATIMER_EN 0x01

#define DRDY_MASK_DATIMER_BASE DRDY_MASK_DATIMER_DIS
#define DRDY_MASK_DATIMER (DRDY_MASK_DATIMER_BASE<<3)

//I2C_DIABLE
#define I2C_DIS_BOTH_EN 0x00
#define I2C_DIS_SPI_ONLY_EN 0x01

#define I2C_DIS_BASE I2C_DIS_SPI_ONLY_EN
#define I2C_DIS (I2C_DIS_BASE<<2)

//LPF1_SEL_G
#define LPF1_SEL_G_DIS 0x00
#define LPF1_SEL_G_EN 0x01
#define LPF1_SEL_G_BASE LPF1_SEL_G_EN
#define LPF1_SEL_G (LPF1_SEL_G_BASE<<1)

//CTRL4_C
#define CTRL4_C (DEN_XL_EN|GYRO_SLEEP|DEN_DRDY_INIT1|INT2_ON_INT1|DRDY_MASK_DATIMER|I2C_DIS|LPF1_SEL_G)

//CTRL5_C
//rounding
#define ROUNDING_NONE     0x00
#define ROUNDING_ACC      0x01
#define ROUNDING_GYRO     0x02
#define ROUNDING_GYRO_ACC 0x03
#define ROUNDING_HUB6     0x04
#define ROUNDING_ACC_HUB6 0x05
#define ROUNDING_ALL_HUB12	0X06
#define ROUNDING_ALL_HUB6	0x07

#define ROUNDING_BASE ROUNDING_NONE
#define ROUNDING (ROUNDING_BASE<<5)

//DEN_LH
#define DEN_LH_LOW 0x00
#define DEN_LH_HIGH 0x01
#define DEN_LH_BASE DEN_LH_LOW
#define DEN_LH (DEN_LH_BASE<<4)

//ST_G
#define ST_G_NORMAL 0x00
#define ST_G_POSITIVE 0x01
#define ST_G_NEGATIVE 0x03
#define ST_G_BASE ST_G_NORMAL
#define ST_G (ST_G_BASE<<2)

//ST_
#define ST_XL_NORMAL 0x00
#define ST_XL_POSITIVE 0x01
#define ST_XL_NEGATIVE 0x02
#define ST_XL_BASE ST_XL_NORMAL
#define ST_XL ST_XL_BASE

//CTRL5_C 0x00
#define CTRL5_C (ROUNDING|DEN_LH|ST_G|ST_XL)

//CTRL6_C
//TRIG_EN
#define TRIG_EN_EN 0x01
#define TRIG_EN_DIS 0x00
#define TRIG_EN_BASE TRIG_EN_DIS
#define TRIG_EN (TRIG_EN_BASE<<7)

//LVL
#define LVL_EN_EDGE_TRIGGER 0x00
#define LVL_EN_LEVEL_TRIGGER 0x02
#define LVL_EN_LEVEL_LATCHED 0x03
#define LVL_EN_FIFO_EN 0x02
#define LVL_EN_NONE 0x00

#define LVL_EN_BASE LVL_EN_NONE
#define LVL_EN (LVL_EN_BASE<<5)

//XL_HM_MODE
#define XL_HM_MODE_EN 0x00
#define XL_HM_MODE_DIS 0x01
#define XL_HM_MODE_BASE XL_HM_MODE_EN
#define XL_HM_MODE (XL_HM_MODE_BASE<<4)

//USR_OFF_W
#define USR_OFF_W_2POW10 0x00
#define USR_OFF_W_2POW6 0x01
#define USR_OFF_W_BASE USR_OFF_W_2POW10
#define USR_OFF_W (USR_OFF_W_BASE<<3)

//FTYPE 권장값 ODR에 따라
#define FTYPE_800HZ 0x00
#define FTYPE_1KHZ6 0x01
#define FTYPE_3KHZ3 0x10
#define FTYPE_6KHZ6 0x11

#define FTYPE_BASE FTYPE_800HZ
#define FTYPE FTYPE_BASE

#define CTRL6_C (TRIG_EN|LVL_EN|XL_HM_MODE|USR_OFF_W|FTYPE_BASE)

//CTRL7_G 0x00
//G_HM_MODE
#define G_HM_MODE_EN 0x01
#define G_HM_MODE_DIS 0x00

#define G_HM_MODE_BASE G_HM_MODE_DIS
#define G_HM_MODE (G_HM_MODE_BASE<<7)

//HP_EN_G
#define HP_EN_G_EN 0x01
#define HP_EN_G_DIS 0x00
#define HP_EN_G_BASE HP_EN_G_DIS
#define HP_EN_G (HP_EN_G_BASE<<6)

//HPM_G
#define HP_HPM_G_0HZ016 0x00
#define HP_HPM_G_0HZ065 0x01
#define HP_HPM_G_0HZ26 0x02
#define HP_HPM_G_1HZ04 0x03
#define HP_HPM_G_BASE HP_HPM_G_0HZ016
#define HP_HPM_G (HP_HPM_G_BASE<<4)

//ROUNDING STATUS
#define ROUNDING_STATUS_EN 0x01
#define ROUNDING_STATUS_DIS 0x00
#define ROUNDING_STATUS_BASE ROUNDING_STATUS_DIS
#define ROUNDING_STATUS (ROUNDING_STATUS_BASE<<2)

//CTRL7_G
#define CTRL7_G (G_HM_MODE|HP_EN_G|HP_HPM_G|ROUNDING_STATUS)

//CTRL8_XL
//LPF2_XL_EN
#define LPF2_XL_EN_EN 0x01
#define LPF2_XL_EN_DIS 0x00
#define LPF2_XL_EN_BASE LPF2_XL_EN_EN
#define LPF2_XL_EN ( LPF2_XL_EN_BASE<<7)
//HPCF_XL datasheet ODR에 따라 다르다
#define HPCF_XL_00 0x00
#define HPCF_XL_01 0x01
#define HPCF_XL_10 0x02
#define HPCF_XL_11 0x03
#define HPCF_XL_BASE HPCF_XL_00
#define HPCF_XL (HPCF_XL_BASE<<5)
//High-pass filter reference 모드 HPF를 켰을 때 기준점을 DC로 고정해두는 기능, 중력 성분을 없애고 진동만 볼때 사용
#define HP_REF_MODE_EN 0x01
#define HP_REF_MODE_DIS 0x00
#define HP_REF_MODE_BASE  HP_REF_MODE_DIS
#define  HP_REF_MODE (HP_REF_MODE_BASE <<4)

//INPUT_COMPOSITE data sheet Table 73. Accelerometer bandwidth selection
#define INPUT_COMPOSITE_LOW_NOISE 0x01
#define INPUT_COMPOSITE_LOW_NLATENCY_AND_HIGH_PASS_PATH 0x00
#define INPUT_COMPOSITE_BASE  INPUT_COMPOSITE_LOW_NLATENCY_AND_HIGH_PASS_PATH
#define INPUT_COMPOSITE (INPUT_COMPOSITE_BASE<<3)

//HP_SLOPE_XL_EN 1이면 가속도 출력에서 저주파 성분 제거
#define HP_SLOPE_XL_EN_EN 0x01
#define HP_SLOPE_XL_EN_DIS 0x00
#define HP_SLOPE_XL_EN_BASE HP_SLOPE_XL_EN_DIS
#define HP_SLOPE_XL_EN (HP_SLOPE_XL_EN_BASE<<2)

//LOW_PASS_ON_6D 6D 제스처인식에 사용
#define LOW_PASS_ON_6D_EN 0x01
#define LOW_PASS_ON_6D_DIS 0x00
#define LOW_PASS_ON_6D_BASE LOW_PASS_ON_6D_DIS
#define LOW_PASS_ON_6D LOW_PASS_ON_6D_BASE

//CTRL8_XL
#define CTRL8_XL (LPF2_XL_EN|HPCF_XL|HP_REF_MODE|INPUT_COMPOSITE|HP_SLOPE_XL_EN|LOW_PASS_ON_6D)

//CTRL9_XL
//DEN_X, Y, Z 각각 LSB에 스탬핑할지 선택 다른 센서/시스템에 타임싱크 필요시 사용, 미사용시 기본값1 유지
//DEN_X
#define DEN_X_STORE 0x01
#define DEN_X_NOT_STORE 0x00
#define DEN_X_BASE DEN_X_STORE
#define DEN_X (DEN_X_BASE<<7)
//DEN_Y
#define DEN_Y_STORE 0x01
#define DEN_Y_NOT_STORE 0x00
#define DEN_Y_BASE DEN_Y_STORE
#define DEN_Y (DEN_Y_BASE<<6)
//DEN_Z
#define DEN_Z_STORE 0x01
#define DEN_Z_NOT_STORE 0x00
#define DEN_Z_BASE DEN_Z_STORE
#define DEN_Z (DEN_Z_BASE<<5)

//DEN_XL DEN 신호를 어디에 스탬핑할지 선택 0 = 자이로 1 = 가속도 사용안함 기본 0 유지
#define DEN_XL_GYRO 0x00
#define DEN_XL_ACC 0x01
#define DEN_XL_BASE DEN_XL_GYRO
#define DEN_XL (DEN_XL_BASE<<4)

//SOFT_EN soft-iron 보정 알고리즘 가속도/자이로만 사용하는 경우 필요x
#define SOFT_EN_EN 0x01
#define SOFT_EN_DIS 0x00
#define SOFT_EN_BASE SOFT_EN_DIS
#define SOFT_EN (SOFT_EN_DIS<<2)
//CTRL9_XL
#define CTRL9_XL (DEN_X|DEN_Y|DEN_Z|DEN_XL|SOFT_EN)

//CTRL10_C
//WRIST_TILT 손목 tilte 알고리즘 켜기 웨어러블 용
#define WRIST_TILT_EN_EN 0x01
#define WRIST_TILT_EN_DIS 0x00
#define WRIST_TILT_EN_BASE WRIST_TILT_EN_DIS
#define WRIST_TILT_EN (WRIST_TILT_EN_BASE <<7)

//TIMER_EN 내부 타임스탬프 카운터 활성화
#define TIMER_EN_TIMESTAMP_DIS 0x00
#define TIMER_EN_TIMESTAMP_EN 0x01
#define TIMER_EN_BASE TIMER_EN_TIMESTAMP_DIS
#define TIMER_EN (TIMER_EN_BASE<<5)

//PEDO_EN 걸음수계산 알고리즘
#define PEDO_EN_DIS 0x00
#define PEDO_EN_EN 0x01
#define PEDO_EN_BASE PEDO_EN_DIS
#define PEDO_EN (PEDO_EN_BASE<<4)

//TILT_EN tilt 알고리즘 기울기
#define TILT_EN_EN  0x01
#define TILT_EN_DIS 0x00
#define TILT_EN_BASE TILT_EN_DIS
#define TILT_EN (TILT_EN_BASE<<3)

//FUNC_EN 내장 알고리즘 전체
#define FUNC_EN_EN 0x01
#define FUNC_EN_DIS 0x00
#define FUNC_EN_BASE FUNC_EN_DIS
#define FUNC_EN (FUNC_EN_BASE<<2)

//PEDO_RST_STEP pedo reset
#define PEDO_RST_STEP_DIS 0x00
#define PEDO_RST_STEP_EN 0x01
#define PEDO_RST_STEP_BASE PEDO_RST_STEP_DIS
#define PEDO_RST_STEP (PEDO_RST_STEP_BASE<<1)

//SIGN_MOTION_EN
#define SIGN_MOTION_EN_EN 0x01
#define SIGN_MOTION_EN_DIS 0x00
#define SIGN_MOTION_EN_BASE SIGN_MOTION_EN_DIS
#define SIGN_MOTION_EN SIGN_MOTION_EN_BASE

//CTRL10_C
#define CTRL10_C (WRIST_TILT_EN|TIMER_EN|PEDO_EN|TILT_EN|FUNC_EN|PEDO_RST_STEP|SIGN_MOTION_EN)
/* ___________________________________________________________________________*/


#define DEG2RAD (M_PI/180.f)
#define RAD2DEG (180.f/M_PI)

// ★자이로 FS=±500 dps일 때 감도(데이터시트)
#define GYRO_SENS_mdps_PER_LSB 17.5f   // mdps/LSB
// 편의용: dps/LSB(=0.0175)
#define GYRO_SENS_dps_PER_LSB  (GYRO_SENS_mdps_PER_LSB * 0.001f)


/* ___________________________________________________________________________*/


extern int16_t GyroRaw[3];
extern int16_t ACCRaw[3];
extern volatile float yaw_rad;
extern volatile float roll_rad;
extern volatile float pitch_rad;


extern float gyro_sens_dps_per_lsb;

/* CTRL2_G 레지스터 값에서 감도(dps/LSB) 계산
 * 비트필드: ODR_G[7:4] | FS_G[3:2] | FS_125[1] | (0)
 * FS_125가 1이면 ±125dps 고정(0.004375 dps/LSB)
 */
float LSM6_get_gyro_sens_dps_per_lsb_from_CTRL2(uint8_t ctrl2);

/* 디바이스에서 CTRL2_G를 읽어 전역 감도(gyro_sens_dps_per_lsb) 갱신 */
void LSM6_update_gyro_sens_from_device(void);

/* raw → dps / rad/s 변환 유틸 */
float gyro_raw_to_dps(int16_t raw);
float gyro_raw_to_rads(int16_t raw);


void LSM6DS3TR_C_Init();
void LSM6DS3TR_C_Routine();
void LSM6DS3TR_C_ConfigCTRL3C();
void LSM6DS3TR_C_CheckCTRL3C();

void LSM6DS3TR_C_ConfigCTRL();
void LSM6DS3TR_C_CheckCTRL();

uint8_t LSM6DS3TR_data_ready();
int16_t LSM6_Merge16(uint8_t lo, uint8_t hi);
//HAL_StatusTypeDef LSM6_ReadGyroRaw(int16_t g[3]);
//HAL_StatusTypeDef LSM6_ReadAccelRaw(int16_t a[3]);
//HAL_StatusTypeDef LSM6_ReadGA12(int16_t g[3], int16_t a[3]); // G(6)+XL(6)
void IMU_GetGyroDps_Corrected(float *g_dps);
void IMU_GetGyroRadPS_Corrected(float g_radps[3]);
void IMU_CalcGyroBias_All_rad(uint16_t samples, uint16_t delay_ms_each);
//float gyro_raw_to_dps();


#endif /* INC_LSM6DS3TR_H_ */
