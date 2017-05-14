/*
 * WAVtest.c
 *
 *  Created on: 5 maj 2017
 *      Author: Myren
 */

#include "main.h"
#include <stdio.h>
#include "ff.h"

#define AUDIO_BLOCK_SIZE	((uint32_t)512)

typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF = 1,
  BUFFER_OFFSET_FULL = 2,
}BUFFER_StateTypeDef;

FIL wavFil;
char smplBuff[2*AUDIO_BLOCK_SIZE];
uint16_t bytesRead;
uint32_t totBytesRead;
uint8_t delaycnt;
uint16_t sample;
BUFFER_StateTypeDef audio_play_buffer_state;
int16_t playBuffer[AUDIO_BLOCK_SIZE];

void delays(uint32_t delaycnt)
{
	uint32_t u,w;
	for(u=0; u<delaycnt; u++)
	{
		w++;
	}
}

void readwav(void)
{
	SD_CARD_Init();
	f_open(&wavFil, "Sweep.wav", FA_OPEN_ALWAYS | FA_READ);
	//f_lseek(&wavFil, 44);
	for(int i=0; i<100; i++)
	{
		//f_read (&File1, (void *)writtenData, (UINT)bytesToRead,(void *)&readBytes);
		f_read(&wavFil, (void *)smplBuff, 2, (void *)&bytesRead);
		sample = (0xff00 & (smplBuff[0] << 8)) | (0x00ff & smplBuff[1]);
		//delays(10000);
		//f_read(&wavFil, (void *)(smplBuff+1), 1, (void *)&bytesRead);
		UART_Transmitt((uint8_t *)smplBuff, 2);
		delays(1000);
	}
}
















































