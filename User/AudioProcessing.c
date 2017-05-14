/*
 * AudioProcessing.c
 *
 * SIMON VON SCHMALENSEE
 *
 */

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <float.h>
#include "main.h"
#include "arm_math.h"
#include "arm_common_tables.h"
#include <string.h>


float firCoeffs[FIR_SIZE] =
{
		-0.001732726,
		-0.001987483,
		-0.002421533,
		-0.002671134,
		-0.002119254,
		0.000000000,
		0.004438057,
		0.011742261,
		0.022081550,
		0.035128923,
		0.050032475,
		0.065489406,
		0.079915842,
		0.091684440,
		0.099386172,
		0.102066010,
		0.099386172,
		0.091684440,
		0.079915842,
		0.065489406,
		0.050032475,
		0.035128923,
		0.022081550,
		0.011742261,
		0.004438057,
		0.000000000,
		-0.002119254,
		-0.002671134,
		-0.002421533,
		-0.001987483,
		-0.001732726


};








float bufferR[BUFFER_SIZE];
float bufferL[BUFFER_SIZE];


//CMSIS STRUCTS
arm_fir_instance_f32 fir_filterL;
arm_fir_instance_f32 fir_filterR;


arm_rfft_fast_instance_f32 FFT_right;
arm_rfft_fast_instance_f32 FFT_left;



static void splitChannels(float *input, float *chL, float *chR, uint16_t offset, uint16_t bufferOffset);
static void floatPointBlockFir(float *x, uint16_t blkSize,
                        float *h, uint16_t order,
                        float *y, float *w, uint16_t *index);



uint32_t convR, convL;


uint16_t indexL;
uint16_t indexR;
float delayLineL[FIR_SIZE];
float delayLineR[FIR_SIZE];

float bufferCMSISL[FIR_SIZE+(AUDIO_BLOCK_SIZE/2)-1];
float bufferCMSISR[FIR_SIZE+(AUDIO_BLOCK_SIZE/2)-1];



void RealFFTinit(FFT_TypeDef *fftStruct)
{
	arm_rfft_fast_init_f32(&FFT_right, FFT_SIZE);
	arm_rfft_fast_init_f32(&FFT_left, FFT_SIZE);

	memset(fftStruct->FFTin_BufferR, 0, sizeof(fftStruct->FFTin_BufferR));
	memset(fftStruct->FFTin_BufferL, 0, sizeof(fftStruct->FFTin_BufferL));
	memset(fftStruct->FFTout_BufferR, 0, sizeof(fftStruct->FFTout_BufferR));
	memset(fftStruct->FFTout_BufferL, 0, sizeof(fftStruct->FFTout_BufferL));
	memset(fftStruct->FilterBufferin1, 0, sizeof(fftStruct->FilterBufferin1));
	memset(fftStruct->FilterBufferout1, 0, sizeof(fftStruct->FilterBufferout1));
	memset(fftStruct->IFFT_outBufferR, 0, sizeof(fftStruct->IFFT_outBufferR));
	memset(	fftStruct-> IFFT_outBufferL, 0, sizeof(fftStruct->IFFT_outBufferL));
	memset(	fftStruct-> FFT_outFilteredL, 0, sizeof(fftStruct->FFT_outFilteredL));
	memset(fftStruct-> 	FFT_outFilteredR, 0, sizeof(fftStruct->FFT_outFilteredR));
    memcpy(fftStruct->FilterBufferin1, firCoeffs, FIR_SIZE);

    fftStruct->FFTloopcnt = 0;
	arm_rfft_fast_f32(&FFT_left, fftStruct->FilterBufferin1, fftStruct->FilterBufferout1, 0);

}

void Cmsis_RealFFT(FFT_TypeDef *fftStruct, float *input, float *output, uint32_t offset)
{

fftStruct->inputptr = input;
fftStruct->outputptr = output;


	for(uint16_t i = 0; i<AUDIO_BLOCK_SIZE; i++){

		//*(FFTin_BufferR+(i+AUDIO_BLOCK_SIZE)*(FFTloopcnt)) = *(input+((2*i)+offset));
		*(fftStruct->FFTin_BufferR+i+(AUDIO_BLOCK_SIZE)*(fftStruct->FFTloopcnt)) = *(fftStruct->inputptr+(2*i+offset));
		*(fftStruct->FFTin_BufferL+i+(AUDIO_BLOCK_SIZE)*(fftStruct->FFTloopcnt)) = *(fftStruct->inputptr+(2*i+1)+offset);


	}

	if((fftStruct->FFTloopcnt)*(AUDIO_BLOCK_SIZE) >= FFT_SIZE-(AUDIO_BLOCK_SIZE))
	{

				arm_rfft_fast_f32(&FFT_right, fftStruct->FFTin_BufferR, fftStruct-> FFTout_BufferR, 0);
				arm_rfft_fast_f32(&FFT_left, fftStruct->FFTin_BufferL, fftStruct->FFTout_BufferL, 0);


				//arm_cmplx_mult_cmplx_f32(FFTout_BufferR, FilterBufferout1, FFT_outFilteredR, FFT_SIZE/2);
				//arm_cmplx_mult_cmplx_f32(FFTout_BufferL, FilterBufferout1, FFT_outFilteredL, FFT_SIZE/2);



				for(uint16_t i = 0; i<FFT_SIZE; i++ )
			{
					fftStruct->FFTout_BufferL[i] *=1;//FilterBufferout[i];
					fftStruct->FFTout_BufferR[i] *=1;//FilterBufferout[i];
			}


				arm_rfft_fast_f32(&FFT_right, fftStruct->FFTout_BufferR, fftStruct->IFFT_outBufferR, 1);
			arm_rfft_fast_f32(&FFT_left, fftStruct->FFTout_BufferL ,fftStruct->IFFT_outBufferL, 1);
			//arm_rfft_fast_f32(&FFT_right, FFT_outFilteredR,IFFT_outBufferR, 1);
			//arm_rfft_fast_f32(&FFT_left, FFT_outFilteredL ,IFFT_outBufferL, 1);
#if 0
			int flag = 0;
//while(flag == 0)
			{
				char temp[9];

				for(int l = 0; l<FFT_SIZE; l++){
					sprintf(temp, "%.5f\n", fftStruct->FFTout_BufferR[l]);
				UART_Transmitt((uint8_t*)temp, strlen(temp));
				}
			while(1){
				int k = 0;
			}
			}
#endif




			//Overlap add
#if 0
			for (int i = 0; i < FIR_SIZE-1; i++){

				fftStruct->IFFT_outBufferR[i] += fftStruct->OLASaveBufferR[i];
				fftStruct->IFFT_outBufferL[i] += fftStruct->OLASaveBufferL[i];
				    	}

			for (int i = 0; i<FIR_SIZE-1; i++){

				fftStruct->OLASaveBufferR[i] = fftStruct->IFFT_outBufferL[FFT_SIZE-FIR_SIZE+i];

				}
#endif

}
	for(uint16_t i= 0; i<AUDIO_BLOCK_SIZE; i++)
	{
		*(fftStruct->outputptr+(2*i+offset)) =*(fftStruct->IFFT_outBufferR+i+(AUDIO_BLOCK_SIZE)*(fftStruct->FFTloopcnt));
		*(fftStruct->outputptr+(2*i+1)+offset) = *(fftStruct->IFFT_outBufferL+i+(AUDIO_BLOCK_SIZE)*(fftStruct->FFTloopcnt));
	}


	BSP_LED_Toggle(LED1);
	fftStruct->FFTloopcnt++;

	if((AUDIO_BLOCK_SIZE)*(fftStruct->FFTloopcnt) >= FFT_SIZE)
	{
		fftStruct->FFTloopcnt = 0;

	}




}







void initCMSISFIR(FIR_BuffersHandleTypeDef *FIR_Buffers ) // FIR coefficients must be stored in time-revered order!
{
	fir_filterL.numTaps = FIR_SIZE;
	fir_filterL.pState = bufferCMSISL;
	fir_filterL.pCoeffs = firCoeffs;
	fir_filterR.numTaps = FIR_SIZE;
	fir_filterR.pState = bufferCMSISR;
	fir_filterR.pCoeffs = firCoeffs;

	memset(FIR_Buffers->inputL, 0, sizeof(FIR_Buffers->inputL));
	memset(FIR_Buffers->inputR, 0, sizeof(FIR_Buffers->inputR));
	memset(FIR_Buffers->outputL, 0, sizeof(FIR_Buffers->outputL));
	memset(FIR_Buffers->outputL, 0, sizeof(FIR_Buffers->inputL));
}





void convolveLR(FIR_BuffersHandleTypeDef *FIR_Buffer ,float *input, float *output, uint32_t offset)
{
   uint16_t l;

     FIR_Buffer->inputptr = input;
     FIR_Buffer->outputptr = output;

    for(l=0; l<((AUDIO_BLOCK_SIZE/4)); l++)
    {
       *(FIR_Buffer->inputL+l)  = *(FIR_Buffer->inputptr+(2*l+offset));
       *(FIR_Buffer->inputR+l) = *(FIR_Buffer->inputptr+(2*l+1+offset));
    }

    //splitChannels(input, inputL, inputR, offset, 0);
    arm_fir_f32(&fir_filterL, FIR_Buffer->inputL, FIR_Buffer->outputL, AUDIO_BLOCK_SIZE/4);
    arm_fir_f32(&fir_filterR, FIR_Buffer->inputR, FIR_Buffer->outputR, AUDIO_BLOCK_SIZE/4);
    //floatPointBlockFir(inputL, (AUDIO_BLOCK_SIZE/4), firCoeffs, FIR_SIZE, outputL, delayLineL, &indexL);
    //floatPointBlockFir(inputR, (AUDIO_BLOCK_SIZE/4), firCoeffs, FIR_SIZE, outputR, delayLineR, &indexR);


    for(l=0; l<((AUDIO_BLOCK_SIZE/4)); l++)
    {
    	*(FIR_Buffer->outputptr+(2*l)+offset) = *(FIR_Buffer->outputL+l);
    	*(FIR_Buffer->outputptr+(2*l)+1+offset) = *(FIR_Buffer->outputR+l);
    }
}



static void floatPointBlockFir(float *x, uint16_t blkSize,
                        float *h, uint16_t order,
                        float *y, float *w, uint16_t *index)
{
    uint16_t i, j, k;
    float sum;
    float *c;

    k = *index;
    for(j=0; j<blkSize; j++)
    {
        w[k] = *x++;
        c = h;
        sum = 0;
        for(i=0; i<order; i++)
        {
            sum += *c++ * w[k++];
            k %= order;
        }
        *y++ = sum;
        k = (order + k - 1)%order;
    }
    *index = k;
}



void IntToFloat(int16_t *input, float *output, uint32_t size, uint32_t offset)
{
	arm_q15_to_float((input+offset), (output+offset), size);

	/*
	uint32_t i;
	for(i=0; i<size; i++)
	{
		//output[i+offset] = (float)input[i+offset];
		*(offset + output++) = (float)(*(offset + input++));
	}
	*/
}

void FloatToInt(float *input, int16_t *output, uint32_t size, uint32_t offset)
{
	arm_float_to_q15((input+offset), (output+offset), size);

	/*
	uint32_t i;
	for(i=0; i<size; i++)
	{
		//output[i+offset] = (int16_t)input[i+offset];
		*(offset + output++) = (int16_t)(*(offset + input++));
	}
	*/
}




void muteRight(float *input, uint32_t offset)
{
	uint32_t i;
	for(i=0; i<(AUDIO_BLOCK_SIZE)/2; i++)
	{
		input[(2*i)+1+offset] = 0*input[(2*i)+1+offset];
	}
}

void muteLeft(float *input, uint32_t offset)
{
	uint32_t i;
	for(i=0; i<(AUDIO_BLOCK_SIZE)/2; i++)
	{
		input[(2*i)+offset] = 0*input[(2*i)+offset];
	}
}







void convolve(float *input, float *output, uint32_t offset)
{
	//float *coeffs = firCoeffs;
	float accR; //Accumulator for right channel
	float accL; //Accumulator for left channel
	uint32_t i,j,k,l;

	splitChannels(input, bufferL, bufferR, offset, FIR_SIZE);

	/*
	for(l=0; l<((AUDIO_BLOCK_SIZE/4)); l++)
	{
		bufferL[l+FIR_SIZE] = input[(2*l)+offset];
		bufferR[l+FIR_SIZE] = input[(2*l)+1+offset];
	}
	*/
	for(i=0; i<((AUDIO_BLOCK_SIZE/4)); i++)
	{
		//Convolve right channel
		accR = 0;
		for(j=0; j<FIR_SIZE; j++)
		{
			//accR += firCoeffs[FIR_SIZE-j-1]*bufferR[j];
			accR = fmaf(bufferR[j], firCoeffs[FIR_SIZE-j-1], accR);
		}

		for(k=0; k<BUFFER_SIZE-1; k++)
		{
			bufferR[k] = bufferR[k+1];
		}

		//Convolve left channel
		accL = 0;
		for(j=0; j<FIR_SIZE; j++)
		{
			//accL += firCoeffs[FIR_SIZE-j-1]*bufferL[j];
			accL = fmaf(bufferL[j], firCoeffs[FIR_SIZE-j-1], accL);
		}

		for(k=0; k<BUFFER_SIZE-1; k++)
		{
			bufferL[k] = bufferL[k+1];
		}
		output[(2*i)+offset] = accL;
		output[(2*i)+1+offset] = accR;
	}
}



static void splitChannels(float *input, float *chL, float *chR, uint16_t offset, uint16_t bufferOffset)
{
	uint32_t i;
	float *chLptr = chL + FIR_SIZE;
	float *chRptr = chR + FIR_SIZE;
	float *inputptr = input;
	for(i=0; i<AUDIO_BLOCK_SIZE/4; i++)
	{
		//chL[i] = input[(2*i)+offset];
		//chR[i] = input[(2*i)+1+offset];
		*(bufferOffset+chLptr++) = *(inputptr + (2*i) + offset);
		*(bufferOffset+chRptr++) = *(inputptr + (2*i) + 1 + offset);
	}
}

