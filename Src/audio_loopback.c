/**
  ******************************************************************************
  * @file    BSP/Src/audio_loopback.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    30-December-2016
  * @brief   This example code shows how to use the audio feature in the
  *          stm32746g_discovery driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
// Von Schmalensee
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "string.h"
#include <stdint.h>
#include <stdlib.h>
#include "ff.h"
#include "arm_const_structs.h"
#include "arm_math.h"


/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF = 1,
  BUFFER_OFFSET_FULL = 2,
}BUFFER_StateTypeDef;




//VARIBLES FOR WAVFUNCTION//
volatile int16_t *playBufferptr;
FIL wavFil;
char smplBuff[2*AUDIO_BLOCK_SIZE];
int16_t playBuffer[AUDIO_BLOCK_SIZE];
uint8_t delaycnt;
uint16_t sample;
//BUFFER_StateTypeDef audio_play_buffer_state;


uint8_t volatile DIRACioFlag;
uint8_t volatile stimuliFlag;

int16_t tempBuffer1[AUDIO_BLOCK_SIZE*2];
int16_t tempBuffer2[AUDIO_BLOCK_SIZE*2];
//int16_t tempBuffer1[AUDIO_BLOCK_SIZE];
//int16_t tempBuffer2[AUDIO_BLOCK_SIZE];
float floatBuffer1[AUDIO_BLOCK_SIZE*2];
float floatBuffer2[AUDIO_BLOCK_SIZE*2];
//float floatBuffer1[AUDIO_BLOCK_SIZE];
//float floatBuffer2[AUDIO_BLOCK_SIZE];
//------------Buffers for the stimulus sweep section-----------




FFT_TypeDef FFT1;
FIR_BuffersHandleTypeDef FIR1;

float volume = (float)(130)/80.0;//160.0;//0.2;//(float)(150/160);
uint8_t PlayFlag = 1;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t  audio_rec_buffer_state;
volatile uint32_t  audio_play_buffer_state;

/* Private function prototypes -----------------------------------------------*/
//static void AudioLoopback_SetHint(void);
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Audio Play demo
  * @param  None
  * @retval None
  */
void AudioLoopback_demo (void)
{

	if(audioFlag)
	{
		/* Initialize Audio Recorder */
		BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1, OUTPUT_DEVICE_HEADPHONE, AUDIO_FREQUENCY_48K, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);

		/*Setting the volume of the incoming signal*/
		BSP_AUDIO_IN_SetVolume(60);
		/*Setting the volume of the outgoing signal*/
		BSP_AUDIO_OUT_SetVolume(65);

		/* Initialize SDRAM buffers */

		memset((uint16_t*)AUDIO_BUFFER_IN, 0, AUDIO_BLOCK_SIZE*2);
		memset((uint16_t*)AUDIO_BUFFER_OUT, 0, AUDIO_BLOCK_SIZE*2);

		audio_rec_buffer_state = BUFFER_OFFSET_NONE;

		/* Start Recording */
		BSP_AUDIO_IN_Record((uint16_t*)AUDIO_BUFFER_IN, AUDIO_BLOCK_SIZE);
		//BSP_AUDIO_IN_Record((uint16_t*)(&input_buffer[0]), AUDIO_BLOCK_SIZE);

		/* Start Playback */
		BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
		BSP_AUDIO_OUT_Play((uint16_t*)AUDIO_BUFFER_OUT, AUDIO_BLOCK_SIZE * 2);
		//BSP_AUDIO_OUT_Play((uint16_t*)(&output_buffer[0]), AUDIO_BLOCK_SIZE * 2);
		initCMSISFIR(&FIR1);
		 RealFFTinit(&FFT1);

		while (audioFlag)
		{


			Get_TouchInput((float*)&volume, (uint8_t*)&PlayFlag);

			/* Wait end of half block recording */
			while((audio_rec_buffer_state != BUFFER_OFFSET_HALF))
			{
			}
			audio_rec_buffer_state = BUFFER_OFFSET_NONE;


			memcpy((uint16_t *)(AUDIO_BUFFER_OUT),
					(uint16_t *)(&tempBuffer2[0]),
					AUDIO_BLOCK_SIZE);
			memcpy((uint16_t*)(&tempBuffer1[0]),
					(uint16_t *)(AUDIO_BUFFER_IN),
					AUDIO_BLOCK_SIZE);


			IntToFloat(tempBuffer1, floatBuffer1, AUDIO_BLOCK_SIZE, 0);


			//Volume

			for(int i=0; i<(AUDIO_BLOCK_SIZE*2); i++)
			{
				floatBuffer1[i] = volume*floatBuffer1[i];
			}



			//muteLeft(floatBuffer1, 0);
			//shiftIn(floatBuffer1, floatBuffer2, FIRBuff, AUDIO_BLOCK_SIZE, FIR_SIZE2, 0);
			//convolveLR(&FIR1,floatBuffer1, floatBuffer2, 0);
			Cmsis_RealFFT(&FFT1 ,floatBuffer1, floatBuffer2, 0);
			//fftCMSIS(floatBuffer1, floatBuffer2, 0);
			//cFFT(floatBuffer1, floatBuffer2, 0);
			FloatToInt(floatBuffer2,tempBuffer2, AUDIO_BLOCK_SIZE, 0);


			/* Wait end of one block recording */
			while((audio_rec_buffer_state != BUFFER_OFFSET_FULL))
			{
			}
			audio_rec_buffer_state = BUFFER_OFFSET_NONE;


			memcpy((uint16_t *)(AUDIO_BUFFER_OUT + (AUDIO_BLOCK_SIZE)),
					(uint16_t *)(&tempBuffer2[AUDIO_BLOCK_SIZE]),
					AUDIO_BLOCK_SIZE);
			memcpy((uint16_t *)(&tempBuffer1[AUDIO_BLOCK_SIZE]),
					(uint16_t *)(AUDIO_BUFFER_IN + (AUDIO_BLOCK_SIZE)),
					AUDIO_BLOCK_SIZE);


			IntToFloat(tempBuffer1, floatBuffer1, AUDIO_BLOCK_SIZE, AUDIO_BLOCK_SIZE);


			//Volume

			for(int i=AUDIO_BLOCK_SIZE; i<(AUDIO_BLOCK_SIZE*2); i++)
			{
				floatBuffer1[i] = volume*floatBuffer1[i];
			}



			//muteLeft(floatBuffer1, AUDIO_BLOCK_SIZE);
			//shiftIn(floatBuffer1, floatBuffer2, FIRBuff, AUDIO_BLOCK_SIZE, FIR_SIZE2, AUDIO_BLOCK_SIZE);
			//convolveLR(&FIR1,floatBuffer1, floatBuffer2, AUDIO_BLOCK_SIZE);
			//fftCMSIS(floatBuffer1, floatBuffer2, AUDIO_BLOCK_SIZE);
			//cFFT(floatBuffer1, floatBuffer2, AUDIO_BLOCK_SIZE);
			Cmsis_RealFFT(&FFT1,floatBuffer1, floatBuffer2, AUDIO_BLOCK_SIZE);


			FloatToInt(floatBuffer2,tempBuffer2, AUDIO_BLOCK_SIZE, AUDIO_BLOCK_SIZE);

		}
		BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
	}
}


void playWav(void)
{
	if(stimuliFlag)
	{


		uint16_t bytesRead;
		uint32_t totBytesRead = 0;
		char channel = 'L';
		uint8_t sweepCnt = 0;
		playBufferptr = playBuffer;
		f_open(&wavFil, "Sweep.wav", FA_OPEN_ALWAYS | FA_READ);
		BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 90, AUDIO_FREQUENCY_48K);

		audio_play_buffer_state = BUFFER_OFFSET_NONE;
		f_lseek(&wavFil, 44);
		uint16_t i;
		f_read(&wavFil, (void *)smplBuff, AUDIO_BLOCK_SIZE, (void *)&bytesRead);
		totBytesRead = totBytesRead + bytesRead;
		memset(playBuffer, 0, sizeof(playBuffer));

#if 0
		for(i=0; i<AUDIO_BLOCK_SIZE/2; i++) //Fyll med nollor
		{
			playBuffer[2*i] = ((int16_t)((0xff00 & (smplBuff[2*i] << 8))) | ((int16_t)(0x00ff & smplBuff[(2*i)+1])));
		}
#endif



		BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
		BSP_AUDIO_OUT_Play((uint16_t*)playBuffer, AUDIO_BLOCK_SIZE*2);


		for(; sweepCnt < 3 && stimuliFlag != 0;){

				f_read(&wavFil, (void *)smplBuff, AUDIO_BLOCK_SIZE/2, (void *)&bytesRead);
				totBytesRead = totBytesRead + bytesRead;
				BSP_LED_Toggle(LED1);

				while(audio_play_buffer_state == BUFFER_OFFSET_NONE)
				{

				}
				audio_play_buffer_state = BUFFER_OFFSET_NONE;


				for(i=0; i<(AUDIO_BLOCK_SIZE/4); i++)
				{
					uint8_t byteMsb = smplBuff[(2*i)+1];
					uint8_t byteLsb = smplBuff[2*i];
					int16_t twoBytes = ((0xff00 & (byteMsb << 8)) | (0x00ff & byteLsb));

					if(channel == 'L')
					{
						*(playBufferptr+2*i) = twoBytes;
						*(playBufferptr+2*i+1) = 0;
					}
					else
					{
						*(playBufferptr+(2*i)+1) = twoBytes;
						*(playBufferptr+(2*i)) = 0;
					}

				}



				if(totBytesRead >= 2*264000)
				{
					totBytesRead = 0;

					f_close(&wavFil);
					f_open(&wavFil, "Sweep.wav", FA_OPEN_ALWAYS | FA_READ);
					f_lseek(&wavFil, 44);
					if(channel == 'L')
					{
						channel = 'R';
					}
					else
					{
						channel = 'L';
					}
					sweepCnt++;

					if(sweepCnt >= 3)
					{
						f_close(&wavFil);

					}
				}

		}
		BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
		f_close(&wavFil);
	}
}




void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{

if(audio_play_buffer_state != BUFFER_OFFSET_NONE){
	return;
}
else{

	playBufferptr = playBuffer+AUDIO_BLOCK_SIZE/2;
	audio_play_buffer_state = BUFFER_OFFSET_FULL;
	return;
}
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
	if(audio_play_buffer_state != BUFFER_OFFSET_NONE){
		return;
	}

	playBufferptr = playBuffer;

	audio_play_buffer_state = BUFFER_OFFSET_HALF;
	return;

}



void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  audio_rec_buffer_state = BUFFER_OFFSET_FULL;
  return;
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  audio_rec_buffer_state = BUFFER_OFFSET_HALF;
  return;
}

