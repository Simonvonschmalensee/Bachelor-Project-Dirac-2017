/*
 * UART.c
 *
 *  Created on: 8 apr. 2017
 *      Von Schmalensee
 */

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#include "stm32f7xx_hal_usart.h"
#include "stm32f7xx_hal_usart_ex.h"
#include "stm32f7xx_hal_rcc.h"
#include "stm32746g_discovery.h"
#include "main.h"
#include "stm32f7xx_hal_def.h"
#include "stm32f7xx_hal_dma.h"
#include "main.h"


#define UART_PRIORITY         6
#define UART_RX_SUBPRIORITY   0
#define MAXCLISTRING          100 // Biggest string the user will type
#define SINE_SIZE			  28075
#define SINE_BUFFER_SIZE	  1024

uint8_t rxBuffer = '\000'; // where we store that one character that just came in
uint8_t rxString[MAXCLISTRING]; // where we build our string from characters coming in
int rxindex = 0; // index for going though rxString
float sineData[1024];
uint32_t sineIndex = 0;
uint32_t rowCount = 0;

UART_HandleTypeDef huart6;
GPIO_InitTypeDef GPIO_InitStruct;
DMA_HandleTypeDef hdma_usart6_rx;

uint8_t volatile DIRACioFlag;
uint8_t volatile stimuliFlag;
uint8_t volatile audioFlag;

void print_uart(void)
{
	//BSP_LCD_Clear(LCD_COLOR_WHITE);
	//BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2, (uint8_t *)rxString, CENTER_MODE);
}

void UART_Init(void)
{
	__GPIOC_CLK_ENABLE();
	__USART6_CLK_ENABLE();
	__DMA2_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP ;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;//230400;//9600;//57600;//9600;//230400;//9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	//UartInitType.OneBitSampling =
	HAL_UART_Init(&huart6);

	hdma_usart6_rx.Instance = DMA2_Stream2;
	hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
	hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart6_rx.Init.MemInc = DMA_MINC_DISABLE;
	hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;



	HAL_DMA_Init(&hdma_usart6_rx);
	__HAL_LINKDMA(&huart6, hdmarx, hdma_usart6_rx);

	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, UART_PRIORITY, UART_RX_SUBPRIORITY);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	__HAL_UART_FLUSH_DRREGISTER(&huart6);
	HAL_UART_Receive_DMA(&huart6, &rxBuffer, 1);
}

void UART_Transmitt(uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit(&huart6, pData, Size, 100);
}

void DMA2_Stream2_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
    HAL_DMA_IRQHandler(&hdma_usart6_rx);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(&huart6); // Clear the buffer to prevent overrun
    //rxString[0] = rxBuffer;
    int i = 0;
    DIRACioFlag = (uint8_t)rxBuffer;
    if(rxBuffer == 'S')
    {
    	stimuliFlag = 1;
    }
    else if(rxBuffer == 'E')
	{
    	stimuliFlag = 0;
	}
    else if(rxBuffer == 'A')
    {
    	audioFlag = 1;
    }
    else if(rxBuffer == 'P')
    {
    	audioFlag = 0;
    }


    UART_Transmitt(&rxBuffer, 1); //ECHOING

    char sdString[1];
    sdString[0] = rxBuffer;
    if(rowCount < 8)
    {
    	//SD_CARD_Write(sdString, 1);
    }

    if (rxBuffer == 8 || rxBuffer == 127) // If Backspace or del
    {
        //print(" \b"); // "\b space \b" clears the terminal character. Remember we just echoced a \b so don't need another one here, just space and \b
        rxindex--;
        rxString[rxindex] = 0;
        if (rxindex < 0) rxindex = 0;
    }

    else if (rxBuffer == '\n' || rxBuffer == '\r') // If Enter
    {

    	//rxString[rxindex] = '\n';
    	//SD_CARD_Write("\n", 1);
        rowCount++;
        if(rowCount >= 8)
        {
        	//SD_CARD_CloseFile();
        }
        //executeSerialCommand(rxString);
        rxindex = 0;

        for (i = 0; i < MAXCLISTRING; i++) rxString[i] = 0; // Clear the string buffer
    }

    else
    {

        rxString[rxindex] = rxBuffer; // Add that character to the string
        rxindex++;

        if (rxindex > MAXCLISTRING) // User typing too much, we can't have commands that big
        {
        	rxString[(rxindex-2)] = '\n';
        	//SD_CARD_Write((char*)rxString, 20);
            rxindex = 0;
            rowCount++;
            for (i = 0; i < MAXCLISTRING; i++) rxString[i] = 0;
            if(rowCount >= 4)
            {
            	//SD_CARD_CloseFile();
            }
            //for (i = 0; i < MAXCLISTRING; i++) rxString[i] = 0; // Clear the string buffer
            //print("\r\nConsole> ");
        }
    }



}

