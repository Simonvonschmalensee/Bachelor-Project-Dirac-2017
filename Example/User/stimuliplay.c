/*
 * stimuliplay.c

 */
#include "main.h"
#include <stdio.h>
#include "ff.h"

#define AUDIO_OUT_BUFFER_SIZE                      8192
#define AUDIO_IN_PCM_BUFFER_SIZE                   4*2304 /* buffer size in half-word */

#define FILEMGR_LIST_DEPDTH                        24
#define FILEMGR_FILE_NAME_SIZE                     40
#define FILEMGR_FULL_PATH_SIZE                     256
#define FILEMGR_MAX_LEVEL                          4
#define FILETYPE_DIR                               0
#define FILETYPE_FILE                              1

typedef enum
{
	BUFFER_OFFSET_NONE = 0,
	BUFFER_OFFSET_HALF,
	BUFFER_OFFSET_FULL
}BUFFER_StateTypeDef;

typedef struct {
  uint32_t ChunkID;       /* 0 */
  uint32_t FileSize;      /* 4 */
  uint32_t FileFormat;    /* 8 */
  uint32_t SubChunk1ID;   /* 12 */
  uint32_t SubChunk1Size; /* 16*/
  uint16_t AudioFormat;   /* 20 */
  uint16_t NbrChannels;   /* 22 */
  uint32_t SampleRate;    /* 24 */

  uint32_t ByteRate;      /* 28 */
  uint16_t BlockAlign;    /* 32 */
  uint16_t BitPerSample;  /* 34 */
  uint32_t SubChunk2ID;   /* 36 */
  uint32_t SubChunk2Size; /* 40 */
}WAVE_FormatTypeDef;

/* Audio buffer control struct */
typedef struct {
  uint16_t buff[AUDIO_OUT_BUFFER_SIZE]; //uint8_t
  BUFFER_StateTypeDef state;
  uint32_t fptr;
}AUDIO_OUT_BufferTypeDef;

typedef struct _FILELIST_LineTypeDef {
  uint8_t type;
  uint8_t name[FILEMGR_FILE_NAME_SIZE];
}FILELIST_LineTypeDef;


typedef struct _FILELIST_FileTypeDef {
  FILELIST_LineTypeDef  file[FILEMGR_LIST_DEPDTH] ;
  uint16_t              ptr;
}FILELIST_FileTypeDef;

typedef enum {
  AUDIO_ERROR_NONE = 0,
  AUDIO_ERROR_IO,
  AUDIO_ERROR_EOF,
  AUDIO_ERROR_INVALID_VALUE,
}AUDIO_ErrorTypeDef;

typedef enum {
  AUDIO_STATE_IDLE = 0,
  AUDIO_STATE_WAIT,
  AUDIO_STATE_INIT,
  AUDIO_STATE_PLAY,
  AUDIO_STATE_PRERECORD,
  AUDIO_STATE_RECORD,
  AUDIO_STATE_NEXT,
  AUDIO_STATE_PREVIOUS,
  AUDIO_STATE_FORWARD,
  AUDIO_STATE_BACKWARD,
  AUDIO_STATE_STOP,
  AUDIO_STATE_PAUSE,
  AUDIO_STATE_RESUME,
  AUDIO_STATE_VOLUME_UP,
  AUDIO_STATE_VOLUME_DOWN,
  AUDIO_STATE_ERROR,
}AUDIO_PLAYBACK_StateTypeDef;

FIL WavFile;
char filename[] = "Sweep.wav";
AUDIO_OUT_BufferTypeDef BufferCtl;
WAVE_FormatTypeDef WaveFormat;
uint32_t bytesread;
FILELIST_FileTypeDef FileList;
AUDIO_PLAYBACK_StateTypeDef AudioState;

static AUDIO_ErrorTypeDef GetFileInfo(uint16_t file_idx, WAVE_FormatTypeDef *info);
static AUDIO_ErrorTypeDef AUDIO_PLAYER_Start(uint8_t idx);
static AUDIO_ErrorTypeDef AUDIO_PLAYER_Init(void);

void stimuli(void)
{
	BSP_LED_Init(LED1);
	SD_CARD_Init();
	if(AUDIO_PLAYER_Init() == AUDIO_ERROR_IO)
	{
		//BSP_LED_On(LED1);
		while(1);
	}
	if(AUDIO_PLAYER_Start(0) == AUDIO_ERROR_IO)
	{
		//BSP_LED_On(LED1);
		while(1);
	}

	while(1)
	{
		 if(BufferCtl.fptr >= WaveFormat.FileSize)
		    {
		      BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
		    }

		    if(BufferCtl.state == BUFFER_OFFSET_HALF)
		    {
		      if(f_read(&WavFile,
		                &BufferCtl.buff[0],
		                AUDIO_OUT_BUFFER_SIZE/2,
		                (void *)&bytesread) != FR_OK)
		      {
		        BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
		      }
		      BufferCtl.state = BUFFER_OFFSET_NONE;
		      BufferCtl.fptr += bytesread;
		    }

		    if(BufferCtl.state == BUFFER_OFFSET_FULL)
		    {
		      if(f_read(&WavFile,
		                &BufferCtl.buff[AUDIO_OUT_BUFFER_SIZE /2],
		                AUDIO_OUT_BUFFER_SIZE/2,
		                (void *)&bytesread) != FR_OK)
		      {
		        BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
		      }

		      BufferCtl.state = BUFFER_OFFSET_NONE;
		      BufferCtl.fptr += bytesread;
		    }
	}

}


/*

  // @brief  Calculates the remaining file size and new position of the pointer.
  // @param  None
  // @retval None

void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  if(AudioState == AUDIO_STATE_PLAY)
  {
    BufferCtl.state = BUFFER_OFFSET_FULL;
  }
}


  // @brief  Manages the DMA Half Transfer complete interrupt.
  // @param  None
  // @retval None

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
  if(AudioState == AUDIO_STATE_PLAY)
  {
    BufferCtl.state = BUFFER_OFFSET_HALF;
  }
}
*/

static AUDIO_ErrorTypeDef GetFileInfo(uint16_t file_idx, WAVE_FormatTypeDef *info)
{
  uint32_t bytesread;
  uint32_t duration;
  uint8_t str[FILEMGR_FILE_NAME_SIZE + 20];

  if(f_open(&WavFile, (char *)FileList.file[file_idx].name, FA_OPEN_EXISTING | FA_READ) == FR_OK)
  {
    /* Fill the buffer to Send */
    f_read(&WavFile, info, sizeof(WaveFormat), (void *)&bytesread);

    f_close(&WavFile);
  }
  return AUDIO_ERROR_IO;
}

/**
  * @brief  Initializes Audio Interface.
  * @param  None
  * @retval Audio error
  */
static AUDIO_ErrorTypeDef AUDIO_PLAYER_Init(void)
{
  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 70, I2S_AUDIOFREQ_48K) == 0)
  {
	BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
    return AUDIO_ERROR_NONE;
  }
  else
  {
    return AUDIO_ERROR_IO;
  }
}

/**
  * @brief  Starts Audio streaming.
  * @param  idx: File index
  * @retval Audio error
  */
static AUDIO_ErrorTypeDef AUDIO_PLAYER_Start(uint8_t idx)
{
  uint32_t bytesread;

  f_close(&WavFile);
  if(1)//(AUDIO_GetWavObjectNumber() > idx)
  {
	f_open(&WavFile, filename, FA_OPEN_ALWAYS | FA_READ);
    GetFileInfo(idx, &WaveFormat);

    /*Adjust the Audio frequency */
    //PlayerInit(WaveFormat.SampleRate);

    BufferCtl.state = BUFFER_OFFSET_NONE;

    /* Get Data from USB Flash Disk */
    f_lseek(&WavFile, 0);

    /* Fill whole buffer at first time */
    if(f_read(&WavFile,
              &BufferCtl.buff[0],
              AUDIO_OUT_BUFFER_SIZE,
              (void *)&bytesread) == FR_OK)
    {
    	BSP_LED_On(LED1);
      AudioState = AUDIO_STATE_PLAY;
      BSP_LCD_DisplayStringAt(250, LINE(9), (uint8_t *)"  [PLAY ]", LEFT_MODE);
      {
        if(bytesread != 0)
        {
          BSP_AUDIO_OUT_Play((uint16_t*)&BufferCtl.buff[0], AUDIO_OUT_BUFFER_SIZE);
          BufferCtl.fptr = bytesread;
          return AUDIO_ERROR_NONE;
        }
      }
    }
  }
  return AUDIO_ERROR_IO;
}





































