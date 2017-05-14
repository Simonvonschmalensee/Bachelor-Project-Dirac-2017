/**
  ******************************************************************************
  * @file    BSP/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    30-December-2016 
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_sd.h"
#include "stm32746g_discovery_eeprom.h"
#include "stm32746g_discovery_camera.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_qspi.h"

#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sd.h"

#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Macros --------------------------------------------------------------------*/
//#ifdef USE_FULL_ASSERT
/* Assert activated */
/*#define ASSERT(__condition__)                do { if(__condition__) \
                                                   {  assert_failed(__FILE__, __LINE__); \
                                                      while(1);  \
                                                    } \
                                              }while(0)
#else*/
/* Assert not activated : macro has no effect */
/*#define ASSERT(__condition__)                  do { if(__condition__) \
                                                   {  ErrorCounter++; \
                                                    } \
                                              }while(0)
#endif*/ /* USE_FULL_ASSERT */

#define RGB565_BYTE_PER_PIXEL     2
#define ARBG8888_BYTE_PER_PIXEL   4

/* Camera have a max resolution of VGA : 640x480 */
#define CAMERA_RES_MAX_X          640
#define CAMERA_RES_MAX_Y          480

/**
  * @brief  LCD FB_StartAddress
  * LCD Frame buffer start address : starts at beginning of SDRAM
  */
#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR

/**
  * @brief  Camera frame buffer start address
  * Assuming LCD frame buffer is of size 480x800 and format ARGB8888 (32 bits per pixel).
  */
#define CAMERA_FRAME_BUFFER       ((uint32_t)(LCD_FRAME_BUFFER + (RK043FN48H_WIDTH * RK043FN48H_HEIGHT * ARBG8888_BYTE_PER_PIXEL)))

/**
  * @brief  SDRAM Write read buffer start address after CAM Frame buffer
  * Assuming Camera frame buffer is of size 640x480 and format RGB565 (16 bits per pixel).
  */
#define SDRAM_WRITE_READ_ADDR        ((uint32_t)(CAMERA_FRAME_BUFFER + (CAMERA_RES_MAX_X * CAMERA_RES_MAX_Y * RGB565_BYTE_PER_PIXEL)))

#define SDRAM_WRITE_READ_ADDR_OFFSET ((uint32_t)0x0800)
#define SRAM_WRITE_READ_ADDR_OFFSET  SDRAM_WRITE_READ_ADDR_OFFSET

#define AUDIO_REC_START_ADDR         SDRAM_WRITE_READ_ADDR

/* The Audio file is flashed with ST-Link Utility @ flash address =  AUDIO_SRC_FILE_ADDRESS */
#define AUDIO_SRC_FILE_ADDRESS       0x08080000   /* Audio file address in flash */

/* Exported types ------------------------------------------------------------*/
/*typedef struct
{
  void   (*DemoFunc)(void);
  uint8_t DemoName[50]; 
  uint32_t DemoIndex;
}BSP_DemoTypedef;
*/
/*
typedef enum {
  AUDIO_ERROR_NONE = 0,
  AUDIO_ERROR_NOTREADY,
  AUDIO_ERROR_IO,
  AUDIO_ERROR_EOF,
}AUDIO_ErrorTypeDef;
*/









//FFT_Struct//

#define AUDIO_BLOCK_SIZE   	1024//512//((uint32_t)512)
#define FIR_SIZE			31//896//5//121//120//60//29
//#define FIR_SIZE2			12*AUDIO_BLOCK_SIZE
#define BUFFER_SIZE			FIR_SIZE+(AUDIO_BLOCK_SIZE/2)//2*AUDIO_BLOCK_SIZE
#define FFT_SIZE			1024//2048//1024//2048//4096
#define FFT_BLOCK_SIZE		4096//3200 //128*25


//AUDIOLOOPBACK DEFINES
#define AUDIO_BUFFER_IN    AUDIO_REC_START_ADDR     /* In SDRAM */
#define AUDIO_BUFFER_OUT   (AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE * 2)) /* In SDRAM */


// AUDIOPROCESS TYPEDEFS//
typedef struct {

	uint8_t FFTloopcnt;

	float FFTin_BufferR[FFT_SIZE];
	  float FFTin_BufferL[FFT_SIZE];
	  float FFTout_BufferR[FFT_SIZE];
	  float FFTout_BufferL[FFT_SIZE];
	  float IFFT_outBufferR[FFT_SIZE];
	  float IFFT_outBufferL[FFT_SIZE];
	  float FilterBufferin1[FFT_SIZE];
	  float FilterBufferout1[FFT_SIZE];
	  float FFT_outFilteredR[FFT_SIZE];
	  float FFT_outFilteredL[FFT_SIZE];
   float OLASaveBufferR [FIR_SIZE-1];
    float OLASaveBufferL [FIR_SIZE-1];
    float *inputptr, *outputptr;

}FFT_TypeDef;

typedef struct {


	float inputL[AUDIO_BLOCK_SIZE/2];
	float inputR[AUDIO_BLOCK_SIZE/2];
	float outputL[AUDIO_BLOCK_SIZE/2];
	float outputR[AUDIO_BLOCK_SIZE/2];
	float *inputptr, *outputptr;

}FIR_BuffersHandleTypeDef;




extern volatile uint8_t DIRACioFlag;
extern volatile uint8_t stimuliFlag;
extern volatile uint8_t audioFlag;

extern const unsigned char stlogo[];
/* Exported variables ---------------------------------------------------*/
extern uint8_t     NbLoop;
extern uint8_t     MfxExtiReceived;
#ifndef USE_FULL_ASSERT
extern uint32_t    ErrorCounter;
#endif
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

#define COUNT_OF_EXAMPLE(x)    (sizeof(x)/sizeof(BSP_DemoTypedef))
/* Exported functions ------------------------------------------------------- */
void AudioPlay_demo (void);
void AudioRec_demo (void);
//void AudioLoopback_demo (void);
void Touchscreen_demo (void);
void LCD_demo (void);
void Log_demo(void);
void SDRAM_demo(void);
void SDRAM_DMA_demo(void);
void SD_demo (void);
void EEPROM_demo(void);
void Camera_demo(void);
uint8_t AUDIO_Process(void);
void QSPI_demo(void);
uint8_t CheckForUserInput(void);
void BSP_LCD_DMA2D_IRQHandler(void);
void Audio_Sweep(void);

uint8_t SD_CARD_Init(void);
uint8_t SD_CARD_OpenFile(char *filename);
void SD_CARD_CloseFile(void);
uint32_t SD_CARD_Write(char *writeString, uint16_t bytesToWrite);
char* SD_CARD_Read(uint16_t bytesToRead);
void SD_CARD_Gets(char* buffer, uint16_t length);

void WriteToSDCard(uint8_t TextToWrite[], uint8_t k);
void print_uart(void);
void UART_Init(void);
void UART_Transmitt(uint8_t *pData, uint16_t Size);

void GUI_Init(void);
void Get_TouchInput(float *volume, uint8_t *playFlag);

//AUDIOPROCESSING FUNCTIONS//
void IntToFloat(int16_t *input, float *output, uint32_t size, uint32_t offset);
void FloatToInt(float *input, int16_t *output, uint32_t size, uint32_t offset);
void muteRight(float *input, uint32_t offset);
void muteLeft(float *input, uint32_t offset);
void convolve(float *input, float *output, uint32_t offset);
void convolveLR(FIR_BuffersHandleTypeDef *FIR_Buffers,float *input, float *output, uint32_t offset);
void initCMSISFIR(FIR_BuffersHandleTypeDef *FIR_Buffers );
void Cmsis_RealFFT(FFT_TypeDef *fftStruct ,float *input, float *output, uint32_t offset);
void RealFFTinit(FFT_TypeDef *fftStruct);
void Cmsis_RealFFT(FFT_TypeDef *fftStruct ,float *input, float *output, uint32_t offset);
void fftCMSIS(float *input, float *output, uint16_t offset);
void floatToIntMono(float *input, int16_t *output, uint32_t offset);
void stimuli(void);
void readwav(void);
void playWav(void);










/*#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line);
#endif*/
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
