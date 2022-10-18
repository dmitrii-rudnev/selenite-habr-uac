
/**
  *******************************************************************************
  *
  * @file    codec_if.h
  * @brief   Header for codec_if.c file
  * @version v1.0
  * @date    23.09.2021
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyrigh &copy; 2021 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CODEC_IF_H_
#define CODEC_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <usbd_audio.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
extern I2S_HandleTypeDef hi2s3;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Codec_Init (void);
void Codec_Set_RX (void);
void Codec_Set_TX (void);
uint8_t Codec_AF_Vol (uint8_t);

/* Private defines -----------------------------------------------------------*/
#define I2S_IF                     &hi2s3

#ifndef CODEC_I2C_PORT
#define CODEC_I2C_PORT             hi2c1
extern I2C_HandleTypeDef CODEC_I2C_PORT;
#endif

#define CODEC_I2CTIMEOUT           10

#ifndef CODEC_BUS_BASE_ADDR
#define CODEC_BUS_BASE_ADDR        0x18
#endif

#ifndef USBD_AUDIO_FREQ
#define USBD_AUDIO_FREQ            96000U
#endif /* USBD_AUDIO_FREQ */

#ifndef AUDIO_OUT_PACKET_NUM
#define AUDIO_OUT_PACKET_NUM       2U
#endif /* AUDIO_OUT_PACKET_NUM */

#define CODEC_BUFF_PACKET_SIZE     (uint16_t)((USBD_AUDIO_FREQ * 2U) / 1000U)                    // Codec buffer packet size in samples

#define CODEC_BUFF_PACKET_NUM      (uint16_t)(AUDIO_OUT_PACKET_NUM)
#define CODEC_BUFF_SIZE            (uint16_t)((CODEC_BUFF_PACKET_SIZE * CODEC_BUFF_PACKET_NUM))  // Codec buffer size in samples
#define CODEC_BUFF_HALF_SIZE       (uint16_t)((CODEC_BUFF_SIZE / 2U))                            // Codec buffer half size

typedef struct
{
  uint16_t rx [CODEC_BUFF_SIZE];
  uint16_t tx [CODEC_BUFF_SIZE];
} Codec_Buff_TypeDef;

typedef struct Output_Level
{
  uint8_t percent;
  uint8_t reg_val;
} Output_Level;

#ifdef __cplusplus
}
#endif

#endif /* CODEC_IF_H_ */
