/**
  ******************************************************************************
  * @file    BSP/Src/touchscreen.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    30-December-2016
  * @brief   This example code shows how to use the touchscreen driver.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  CIRCLE_RADIUS        30
/* Private macro -------------------------------------------------------------*/
#define  CIRCLE_XPOS(i)       ((i * BSP_LCD_GetXSize()) / 5)
#define  CIRCLE_YPOS(i)       (BSP_LCD_GetYSize() - CIRCLE_RADIUS - 60)
#define PLAY 1
#define PAUSE 1

#define AUDIO_BLOCK_SIZE   ((uint32_t)512)
#define AUDIO_BUFFER_IN    AUDIO_REC_START_ADDR     /* In SDRAM */
#define AUDIO_BUFFER_OUT   (AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE * 2)) /* In SDRAM */


uint8_t Is_Playing;
uint8_t isTouched;
uint8_t touchxy;
float prevVolume;
float slideVolume;


void GUI_Init(void);
//void BSP_LCD_FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3);
/* Private variables ---------------------------------------------------------*/
static TS_StateTypeDef  TS_State;
/* Private function prototypes -----------------------------------------------*/
//static void Touchscreen_SetHint(void);
//static void Touchscreen_DrawBackground (uint8_t state);
static void LCD_Config(void);
static void GUI_SetHint(void);
static void Set_Play(void);
static void Set_Pause(void);
static void Set_VolumeBar(uint16_t ypos);
static void LCD_Error(void);
/* Private functions ---------------------------------------------------------*/



void GUI_Init(void)
{
	uint8_t  status = 1;
	LCD_Config();
	status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	if(status != TS_OK)
	{
		LCD_Error();
	}
	else
	{
		GUI_SetHint();
	}
}

void Get_TouchInput(float *volume, uint8_t *playFlag)
{

	uint16_t x, y;
	uint8_t  text[30];

	BSP_TS_GetState(&TS_State);
	if(TS_State.touchDetected)
	{
		/* Get X and Y position of the touch post calibrated */
		x = TS_State.touchX[0];
	 	y = TS_State.touchY[0];

	 	uint16_t yminpos = (BSP_LCD_GetYSize()/2)-20;
	 	uint16_t ymaxpos = (BSP_LCD_GetYSize()/2)+20;

	 	uint16_t xMinSlide = BSP_LCD_GetXSize()-80;
	 	uint16_t xMaxSlide = BSP_LCD_GetXSize()-80+50;
	 	uint16_t yMinSlide = (BSP_LCD_GetYSize()/2)-80;
	 	uint16_t yMaxSlide = (BSP_LCD_GetYSize()/2)-80+160;

	 	/*
	 	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	 	BSP_LCD_SetFont(&Font12);
	 	sprintf((char*)text, "Nb touch detected = %d", TS_State.touchDetected);
	 	BSP_LCD_DisplayStringAt(15, BSP_LCD_GetYSize() - 40, (uint8_t *)&text, LEFT_MODE);

	 	// Display 1st touch detected coordinates
	 	sprintf((char*)text, "1[%d,%d]    ", x, y);
	 	BSP_LCD_DisplayStringAt(15,
	 	                        BSP_LCD_GetYSize() - 25,
	 	                        (uint8_t *)&text,
	 	                        LEFT_MODE);
		*/
	 	if(((x > 60)&&(x < 100))&&((y > yminpos)&&(y < ymaxpos)))
 		{
	 		if(isTouched == 0)
	 		{
	 			if(Is_Playing)
	 			{
	 				Set_Pause();
	 				Is_Playing = 0;
	 				*playFlag = 1;
	 				*volume = slideVolume;//prevVolume;
	 			}
	 			else
	 			{
	 				Set_Play();
	 				Is_Playing = 1;
	 				*playFlag = 0;
	 				slideVolume = *volume;
	 				//prevVolume = *volume;
	 				*volume = 0;

	 			}
	 			isTouched = 1;
	 		}
	 	}


	 	else if(((x > xMinSlide)&&(x < xMaxSlide))&&((y > yMinSlide)&&(y < yMaxSlide)))
	 	{
	 		uint16_t barLevel = 160-(y-((BSP_LCD_GetYSize()-160)/2));
	 		Set_VolumeBar(barLevel);
	 		if(Is_Playing)
	 		{
	 			slideVolume = (float)(barLevel)/80.0;//160.0;

	 		}
	 		else
	 		{
	 			slideVolume = (float)(barLevel)/80.0;//160.0;
	 			*volume = slideVolume;
	 		}
	 	}


	 }
	else
	{
		isTouched = 0;
	}
	//return slideVolume;
}

static void LCD_Error(void)
{
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2, (uint8_t *)"FAILED TS INIT", CENTER_MODE);
}

static void LCD_Config(void)
{
  /* LCD Initialization */
  BSP_LCD_Init();

  /* LCD Initialization */
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);

  /* Enable the LCD */
  BSP_LCD_DisplayOn();

  /* Select the LCD Background Layer  */
  BSP_LCD_SelectLayer(0);

  /* Clear the Background Layer */
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  /* Configure the transparency for background */
  BSP_LCD_SetTransparency(0, 200);
}

static void GUI_SetHint(void)
{
	/* Clear the LCD */
	BSP_LCD_Clear(LCD_COLOR_WHITE);

	/* Set Touchscreen Demo description */
	  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	  BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

	  //BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	  //BSP_LCD_SetFont(&Font24);
	  //BSP_LCD_DisplayStringAt(0, 0, (uint8_t *)"Touchscreen", CENTER_MODE);
	  //BSP_LCD_SetFont(&Font12);
	  //BSP_LCD_DisplayStringAt(0, 30, (uint8_t *)"Please use the Touchscreen to", CENTER_MODE);
	  //BSP_LCD_DisplayStringAt(0, 45, (uint8_t *)"fill the colored circles according to pressure applied", CENTER_MODE);
	  //BSP_LCD_DisplayStringAt(0, 60, (uint8_t *)"Up to 5 finger touch coordinates are displayed", CENTER_MODE);
	  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

	  BSP_LCD_FillRect(BSP_LCD_GetXSize()-80, (BSP_LCD_GetYSize()/2)-80, 50, 160);

	  Set_Pause();
	  Is_Playing = 0;

	  BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
	  BSP_LCD_DrawRect(10, 10, BSP_LCD_GetXSize() - 20, BSP_LCD_GetYSize() - 20);
	  BSP_LCD_DrawRect(11, 11, BSP_LCD_GetXSize() - 22, BSP_LCD_GetYSize() - 22);

	  Set_VolumeBar(130);

}


static void Set_Play(void)
{
	uint16_t y1 = (BSP_LCD_GetYSize()/2)-20;
	uint16_t y2 = (BSP_LCD_GetYSize()/2)+20;
	uint16_t y3 = BSP_LCD_GetYSize()/2;
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillCircle(80, BSP_LCD_GetYSize()/2, 45);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillCircle(80, BSP_LCD_GetYSize()/2, 40);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillTriangle(65, 65, 105, y1, y2, y3);
	//BSP_LCD_FillPolygon(Points, 6);
}
static void Set_Pause(void)
{
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillCircle(80, BSP_LCD_GetYSize()/2, 45);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillCircle(80, BSP_LCD_GetYSize()/2, 40);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(60, (BSP_LCD_GetYSize()/2)-20, 10, 40);
	BSP_LCD_FillRect(90, (BSP_LCD_GetYSize()/2)-20, 10, 40);
}

static void Set_VolumeBar(uint16_t barPos)
{
	if((barPos >= 0) && (barPos <= 160))
	{
		uint16_t ypos = 160-barPos;
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(BSP_LCD_GetXSize()-80, (BSP_LCD_GetYSize()/2)-80, 50, 160);
		if(barPos <= 90)
		{
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
			BSP_LCD_FillRect(BSP_LCD_GetXSize()-80, (BSP_LCD_GetYSize()/2)-80+ypos, 50, 160-ypos);
		}
		else if((barPos > 90) && (barPos < 130))
		{
			BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
			BSP_LCD_FillRect(BSP_LCD_GetXSize()-80, (BSP_LCD_GetYSize()/2)-80+ypos, 50, 160-ypos);
		}
		else
		{
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
			BSP_LCD_FillRect(BSP_LCD_GetXSize()-80, (BSP_LCD_GetYSize()/2)-80+ypos, 50, 160-ypos);
		}
	}
}
