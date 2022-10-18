/**
  *******************************************************************************
  *
  * @file    dsp_if.c
  * @brief   Digital Signal Processor Interface
  * @version v2.0
  * @date    13.09.2022
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyrigh &copy; 2020 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "dsp_if.h"
#include "main.h"
#include <string.h>

#if (CODEC)
#include "codec_if.h"
#endif /* CODEC */

#if (SI5351)
#include "si5351a.h"
#endif /* SI5351 */

/* Private typedef -----------------------------------------------------------*/

#if (EMBED_ADC)
typedef struct
{
  uint16_t buff [ADC_BUFF_SIZE];
  uint32_t wr_ptr;
} ADC_Buff_TypeDef;
#endif /* ADC */

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
DSP_Buff_TypeDef dsp_out_buff;

#if (DSP)
DSP_Buff_TypeDef dsp_in_buff;
#endif /* DSP */

#if (EMBED_ADC)
ADC_Buff_TypeDef adc_buff;
#endif /* ADC */

/* Private function prototypes -----------------------------------------------*/


/* Private user code ---------------------------------------------------------*/


/* External variables --------------------------------------------------------*/

#if (EMBED_ADC)
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
#endif /* ADC */

/* Private functions ---------------------------------------------------------*/

/**
 * @brief This function writes a sample to DSP Out buffer
 *
 * @param a sample to write *
 */

void dsp_out_buff_write (uint16_t sample)
{
  dsp_out_buff.buff [dsp_out_buff.wr_ptr] = sample;
  dsp_out_buff.wr_ptr++;

  if (dsp_out_buff.wr_ptr == DSP_BUFF_SIZE)
  {
    dsp_out_buff.wr_ptr = 0U;
  }
}

/**
 * @brief This function writes to DSP Out buffer
 *
 * This function writes to DSP Out buffer and prevent read/write areas overlay
 *
 * @param Source buffer pointer
 * @param Number of bytes to write
 *
 */

void DSP_Out_Buff_Write (uint8_t *pbuf, uint32_t size)
{
  uint32_t i;

  uint16_t *buff = (uint16_t*) pbuf;

  size = size / 2U;

#if (DSP)
  uint16_t gap;

  if (dsp_out_buff.buff_enable == 0U)
  {
    dsp_out_buff.wr_ptr = dsp_out_buff.rd_ptr + DSP_BUFF_HALF_SIZE;

    if (dsp_out_buff.wr_ptr >= DSP_BUFF_SIZE)
    {
      dsp_out_buff.wr_ptr -= DSP_BUFF_SIZE;
    }

    dsp_out_buff.buff_enable = 1U;
  }

  gap = dsp_out_buff.wr_ptr;

  if (dsp_out_buff.rd_ptr > dsp_out_buff.wr_ptr)
  {
    gap += DSP_BUFF_SIZE;
  }

  gap -= dsp_out_buff.rd_ptr;

  if (gap > (3U * DSP_BUFF_SIZE / 4U))  // запись быстрее чтения
  {
    if (dsp_out_buff.wr_ptr < 2U)
    {
      dsp_out_buff.wr_ptr += DSP_BUFF_SIZE;
    }

    dsp_out_buff.wr_ptr -= 2U;  // смещаем указатель записи назад
  }

  if (gap < (DSP_BUFF_SIZE / 4U))  // чтение быстрее записи
  {
    dsp_out_buff.wr_ptr += 2U;  // смещаем указатель записи вперед

    if (dsp_out_buff.wr_ptr >= DSP_BUFF_SIZE)
    {
      dsp_out_buff.wr_ptr -= DSP_BUFF_SIZE;
    }
  }
#endif /* DSP */

  for (i = 0; i < size; i++)
  {
    dsp_out_buff_write (buff [i]);
  }
}

/**
 * @brief This function flush DSP Out buffer
 *
 * This function writes to DSP Out buffer zeros
 */

void DSP_Out_Buff_Mute (void)
{
  memset ((uint8_t*) dsp_out_buff.buff, 0, DSP_BUFF_SIZE * 2U);
}

#if (DSP)
/**
 * @brief This function writes to Codec TX buffer
 *
 *@param Codec TX buffer pointer
 *@param Number of samples to write
 */

void DSP_Out_Buff_Read (uint16_t *pbuf, uint16_t size)
{
  memcpy ((uint8_t*) pbuf, (uint8_t*) &dsp_out_buff.buff[dsp_out_buff.rd_ptr], size * 2U);

  dsp_out_buff.rd_ptr += size;

  if (dsp_out_buff.rd_ptr >= DSP_BUFF_SIZE)
  {
    dsp_out_buff.rd_ptr = 0U;
  }
}

/**
 * @brief This function writes a sample to DSP In buffer
 *
 * @param a sample to write
 */

void dsp_in_buff_write (uint16_t sample)
{
  dsp_in_buff.buff [dsp_in_buff.wr_ptr] = sample;
  dsp_in_buff.wr_ptr++;

  if (dsp_in_buff.wr_ptr == DSP_BUFF_SIZE)
  {
    dsp_in_buff.wr_ptr = 0U;
  }
}

#if (EMBED_ADC)
/**
 * @brief This function writes a sample to DSP In buffer from ADC buffer
 *
 * This This function writes a sample to DSP In buffer from ADC buffer
 * ADC sampling frequency is 8 * UAC frequency
 *
 * @param ADC buffer pointer
 * @param Number of samples to write
 */

void dsp_in_buff_write_adc (uint16_t *pbuf, uint16_t size)
{
  uint32_t i, j, k;
  int16_t  sample;

  uint16_t *buff = (uint16_t*) pbuf;

  for (i = 0; i < size; i += 16U)
  {
    for (j = 0; j < 14; j += 2)
    {
      buff [i+0] += buff [i+j+2];
      buff [i+1] += buff [i+j+3];
    }

    for (k = 0; k < 2; k++)
    {
      buff [i+k] *= 2U;
      buff [i+k] = buff [i+k] >> 2U;

      if (buff [i+k] > 8192)
      {
        sample = (int16_t)(buff [i+k] - 8192) * 4;
      }
      else
      {
        sample = ((int16_t)buff [i+k] - 8192) * 4;
      }

      dsp_in_buff_write (sample);
    }
  }
}
#endif /* ADC */

/**
 * @brief This function writes to DSP In buffer
 *
 * This function writes to DSP In buffer and prevent read/write areas overlay
 *
 * @param Source buffer pointer
 * @param Number of samples to write
 *
 */


void DSP_In_Buff_Write (uint16_t *pbuf, uint16_t size)
{
  uint16_t gap = 0U;

  if (dsp_in_buff.buff_enable)
  {
    gap = dsp_in_buff.wr_ptr;

    if (dsp_in_buff.rd_ptr > dsp_in_buff.wr_ptr)
    {
      gap += DSP_BUFF_SIZE;
    }

    gap -= dsp_in_buff.rd_ptr;
  }

  if (gap > (3U * DSP_BUFF_SIZE / 4U))  // wr is faster
  {
    if (dsp_in_buff.wr_ptr < 2U)
    {
      dsp_in_buff.wr_ptr += DSP_BUFF_SIZE;
    }

    dsp_in_buff.wr_ptr -= 2U;          // shift wr_ptr backward
  }

  if (gap < (DSP_BUFF_SIZE / 4U))      // rd is faster
  {
    dsp_in_buff.wr_ptr += 2U;          // shift wr_ptr forward

    if (dsp_in_buff.wr_ptr >= DSP_BUFF_SIZE)
    {
      dsp_in_buff.wr_ptr -= DSP_BUFF_SIZE;
    }
  }

#if (EMBED_ADC)
  dsp_in_buff_write_adc (pbuf, size);
#else /* ADC */
  uint16_t *buff = (uint16_t*) pbuf;

  for (uint32_t i = 0; i < size; i++)
  {
    dsp_in_buff_write (buff [i]);
  }
#endif /* ADC */
}

#if (EMBED_ADC)
void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef* hadc)
{
  DSP_In_Buff_Write (&adc_buff.buff[0U], ADC_BUFF_HALF_SIZE);
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc)
{
  DSP_In_Buff_Write (&adc_buff.buff[ADC_BUFF_HALF_SIZE], ADC_BUFF_HALF_SIZE);
}
#endif /* ADC */

#endif /* DSP */

/**
 * @brief This function writes to USBD In buffer
 *
 * @param USBD In buffer pointer
 * @param Number of bytes to write
 */

void DSP_In_Buff_Read (uint8_t *pbuf, uint32_t size)
{
#if (DSP)
  if (dsp_in_buff.buff_enable == 0U)
  {
    dsp_in_buff.rd_ptr = dsp_in_buff.wr_ptr + DSP_BUFF_HALF_SIZE;

    if (dsp_in_buff.rd_ptr >= DSP_BUFF_SIZE)
    {
      dsp_in_buff.rd_ptr = 0U;
    }

    dsp_in_buff.buff_enable = 1U;
  }

  memcpy (pbuf, (uint8_t*) &dsp_in_buff.buff[dsp_in_buff.rd_ptr], size);

  dsp_in_buff.rd_ptr += size / 2U;

  if (dsp_in_buff.rd_ptr >= DSP_BUFF_SIZE)
  {
    dsp_in_buff.rd_ptr = 0U;
  }
#else
  if (dsp_out_buff.buff_enable == 0U)
  {
    dsp_out_buff.rd_ptr = dsp_out_buff.wr_ptr + DSP_BUFF_HALF_SIZE;

    if (dsp_out_buff.rd_ptr >= DSP_BUFF_SIZE)
    {
      dsp_out_buff.rd_ptr = 0U;
    }

    dsp_out_buff.buff_enable = 1U;
  }

  memcpy (pbuf, (uint8_t*) &dsp_out_buff.buff[dsp_out_buff.rd_ptr], size);

  dsp_out_buff.rd_ptr += size / 2U;

  if (dsp_out_buff.rd_ptr >= DSP_BUFF_SIZE)
  {
    dsp_out_buff.rd_ptr = 0U;
  }
#endif  /* DSP */
}

#if (DSP)
/**
 * @brief This function sets DSP to RX mode
 *
 */

void DSP_Set_RX (void)
{
#if (CODEC)
  Codec_Set_RX ();
#endif  /* CODEC */
}

/**
 * @brief This function sets DSP to TX mode
 *
 */

void DSP_Set_TX (void)
{
#if (CODEC)
  Codec_Set_TX ();
#endif  /* CODEC */
}
/**
 * @brief This function initialize DSP and codec
 *
 */

void DSP_Init (void)
{
#if (CODEC)
  Codec_Init ();
#endif  /* CODEC */

#if (EMBED_ADC)
  HAL_TIM_Base_Start (&htim2);
  TIM2->ARR = (96000000U / USBD_AUDIO_FREQ / 8U) - 1;

  adc_buff.wr_ptr = 0U;

  HAL_ADC_Start_DMA (&hadc1, (uint32_t*) adc_buff.buff, ADC_BUFF_SIZE);
#endif /* ADC */
}
#else  /*DSP */

void DSP_Init (void)
{
}
#endif  /* DSP */

#if (SI5351)
/**
  * @brief This function calls VFO_Init () to sets HSE = 24.576 MHz
  *
  */

void DSP_Set_HSE (void)
{
  Si5351a_Init ();
  HAL_Delay (100);
}
#endif /* SI5351 */

/****END OF FILE****/
