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

#define LSM6DS3TR_C_WHO_AM_I_REG    0x0F

#include "spi.h"

#include "math.h"
#include <stdint.h>
#include <stdlib.h>

void LSM6DS3TR_C_Init();
void LSM6DS3TR_C_Routine();
void LSM6DS3TR_C_ConfigCTRL3C();
void LSM6DS3TR_C_CheckCTRL3C();

#endif /* INC_LSM6DS3TR_H_ */
