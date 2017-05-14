/*
 * sdtest.c
 *
 *  Created on: 3 apr. 2017
 *      Author:von Schmalensee
 */

#include "main.h"

#define BLOCK_START_ADDR         0     /* Block start address      */
#define NUM_OF_BLOCKS            5     /* Total number of blocks   */
#define BUFFER_WORDS_SIZE        ((BLOCKSIZE * NUM_OF_BLOCKS) >> 2) /* Total data size in bytes */

uint32_t aTxBuffer[BUFFER_WORDS_SIZE];
uint32_t aRxBuffer[BUFFER_WORDS_SIZE];
static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset);
static uint8_t Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint16_t BufferLength);

void initGPIO(void)
{
	 /* GPIO CLOCK */
	RCC -> AHB1ENR |= (1 << 8);
	/* GPIOI pin 1 as output */
	GPIOI -> MODER |= (1 << 2);
	GPIOI -> MODER &= ~(1 << 3);
	/* PUSH-PULL*/
	GPIOI -> OTYPER &= ~(1 << 1);
	/* LOW FREQUENCY */
	GPIOI -> OSPEEDR &= ~((1 << 3) | (1 << 2));
	/*NO PULL UP/DOWN */
	GPIOI -> PUPDR &= ~((1 << 3) | (1 << 2));

	/*Input*/
	GPIOI -> MODER &= ~((1 << 23) | (1 << 22));
	GPIOI -> OTYPER &= ~(1 << 11);
	GPIOI -> OSPEEDR &= ~((1 << 23) | (1 << 22));
	GPIOI -> PUPDR |= (1 << 23);
	GPIOI -> PUPDR &= ~(1 << 22);
}

void delay(uint32_t del)
{
	uint32_t i,j;
	for(i=0; i<del; i++)
	{
		j++;
	}
}
void sdTest(void)
{
	initGPIO();
	BSP_SD_Init();

	Fill_Buffer(aTxBuffer, BUFFER_WORDS_SIZE, 0x2300);
	BSP_SD_WriteBlocks(aTxBuffer, BLOCK_START_ADDR, NUM_OF_BLOCKS, 10000);

	/* Wait until SD card is ready to use for new operation */
	while(BSP_SD_GetCardState() != SD_TRANSFER_OK)
	{
	}

	BSP_SD_ReadBlocks(aRxBuffer, BLOCK_START_ADDR, NUM_OF_BLOCKS, 10000);

	/* Wait until SD card is ready to use for new operation */
	while(BSP_SD_GetCardState() != SD_TRANSFER_OK)
	{
	}

	if (!(Buffercmp(aTxBuffer, aRxBuffer, BUFFER_WORDS_SIZE)))
	{
		GPIOI->ODR |= (1 << 1);
	}
	else
	{
		GPIOI->ODR &= ~(1 << 1);
	}
}

static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset)
{
  uint32_t tmpIndex = 0;

  /* Put in global buffer different values */
  for (tmpIndex = 0; tmpIndex < uwBufferLenght; tmpIndex++ )
  {
    pBuffer[tmpIndex] = tmpIndex + uwOffset;
  }
}

static uint8_t Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 1;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}
