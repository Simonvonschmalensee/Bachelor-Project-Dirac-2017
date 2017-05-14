

 /*Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "ff.h"
#include<string.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//FATFS SDFatFs;  /* File system object for SD card logical drive */
//FIL File1;     /* File object */
//char SDPath[4]; /* SD card logical drive path */


/*FATFS  File system Object*/
FATFS SDFatFs;

/*File object*/
FIL File1;

/*SD card logical drive path*/
char SDPath[4];

char writtenData[100];


/* Private function prototypes -----------------------------------------------*/
//static void SystemClock_Config(void);
//static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

uint8_t SD_CARD_Init(void)
{
	FATFS_LinkDriver(&SD_Driver, SDPath);

	/*Register the filesystem object to the Fatfs module*/
	uint8_t res = f_mount(&SDFatFs,(TCHAR const*)SDPath,1);

	char temp[20];
	temp;
	 if(res == 11){
     sprintf(temp, "%i\n", res);

	 UART_Transmitt((uint8_t*)temp, strlen(temp));
	 }

	return res;
}

uint8_t SD_CARD_OpenFile(char *filename)
{
	uint8_t res = f_open(&File1, filename, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	return res;
}

void SD_CARD_CloseFile(void)
{
	f_close(&File1);
}

uint32_t SD_CARD_Write(char *writeString, uint16_t bytesToWrite)
{
	uint32_t WrittenBytes;
	//uint16_t stringSize = (sizeof(writeString))-1;
	f_write(&File1, writeString, (UINT)bytesToWrite, (void *)&WrittenBytes);
	return WrittenBytes;//(UINT)bytesToWrite
}

char* SD_CARD_Read(uint16_t bytesToRead)
{
	uint16_t readBytes;;
	f_read (&File1, (void *)writtenData, (UINT)bytesToRead,(void *)&readBytes);
	return writtenData;
}


void SD_CARD_Gets(char* buffer, uint16_t length)
{
	f_gets((TCHAR*)buffer, (int)length, &File1);
}


/*
char* SD_CARD_Gets(char* buffer, uint16_t length)
{
	TCHAR* ptr;
	ptr = f_gets((TCHAR*)&buffer, (int)length, &File1);
	return (char*)ptr;
}
*/

void WriteToSDCard(uint8_t TextToWrite[], uint8_t k)
{

  /* Enable the CPU Cache */
  //CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
 // HAL_Init();

  /* Configure the system clock to 216 MHz */
  //SystemClock_Config();

  /* Configure LED1  */
  BSP_LED_Init(LED1);




  uint32_t WrittenBytes;

  /*Link the micro SD disk I/O driver*/
  //FATFS_LinkDriver(&SD_Driver, SDPath);

  /*Register the filesystem object to the Fatfs module*/
  //f_mount(&SDFatFs,(TCHAR const*)SDPath,0);


//if(  f_mkfs((TCHAR const*)SDPath, 0, 0) != FR_OK){
 // BSP_LED_Off(LED1);}
  /*Creates a new TXT file called TEST.TXT with write access*/
  //f_open(&File1, "STM32f7.TXT", FA_CREATE_ALWAYS | FA_WRITE);
  f_open(&File1, "STM32f7.TXT", FA_OPEN_EXISTING | FA_WRITE);


  /*Writes whats in the TextToWrite buffer to the file*/
  f_write(&File1, TextToWrite, k, (void *)&WrittenBytes);
  f_close(&File1);


  BSP_LED_On(LED1);


}
