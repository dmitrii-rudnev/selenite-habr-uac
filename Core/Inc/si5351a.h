/**
  *******************************************************************************
  *
  * @file    si5351a.h
  * @brief   Header for si5351a.c file
  * @version v1.0
  * @date    18.10.2022
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyrigh &copy; 2022 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *******************************************************************************
  */

#ifndef INC_SI5351A_H_
#define INC_SI5351A_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/

void Si5351a_Init (void);
void Si5351a_Set_Freq (int32_t, uint16_t, uint16_t);

/* Private defines -----------------------------------------------------------*/

#ifndef F_XTAL
#define F_XTAL                    24999900
#endif

#ifndef hi2c
#define hi2c                      hi2c1
#endif

#define I2CTIMEOUT                10

#ifndef SI5351_BUS_BASE_ADDR
#define SI5351_BUS_BASE_ADDR      0x60
#endif

enum ms_t {
  PLLA = 0, PLLB = 1,
  MSNA =-2, MSNB =-1,
  MS0  = 0, MS1  = 1, MS2 = 2 };

#endif /* INC_SI5351A_H_ */
