/************************************************************************/
/*																		*/
/*	demo.c	--	Zybo DMA Demo				 						*/
/*																		*/
/************************************************************************/
/*	Author: Sam Lowe											*/
/*	Copyright 2015, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*		This file contains code for running a demonstration of the		*/
/*		DMA audio inputs and outputs on the Zybo.					*/
/*																		*/
/*																		*/
/************************************************************************/
/*  Notes:																*/
/*																		*/
/*		- The DMA max burst size needs to be set to 16 or less			*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/* 																		*/
/*		9/6/2016(SamL): Created										*/
/*																		*/
/************************************************************************/


#include "demo.h"




#include "audio/audio.h"
#include "dma/dma.h"
#include "intc/intc.h"
#include "userio/userio.h"
#include "iic/iic.h"
#include "ff.h"

/***************************** Include Files *********************************/

#include "C:\Users\Christopher\Documents\VivadoProjects\Zybo-DMA\proj\DMA.sdk\dma_bsp\ps7_cortexa9_0\include\xaxidma.h"
#include "C:\Users\Christopher\Documents\VivadoProjects\Zybo-DMA\proj\DMA.sdk\dma_bsp\ps7_cortexa9_0\include\xparameters.h"
#include "C:\Users\Christopher\Documents\VivadoProjects\Zybo-DMA\proj\DMA.sdk\dma_bsp\ps7_cortexa9_0\include\xil_exception.h"
#include "C:\Users\Christopher\Documents\VivadoProjects\Zybo-DMA\proj\DMA.sdk\dma_bsp\ps7_cortexa9_0\include\xdebug.h"
#include "C:\Users\Christopher\Documents\VivadoProjects\Zybo-DMA\proj\DMA.sdk\dma_bsp\ps7_cortexa9_0\include\xiic.h"
#include "C:\Users\Christopher\Documents\VivadoProjects\Zybo-DMA\proj\DMA.sdk\dma_bsp\ps7_cortexa9_0\include\xaxidma.h"
#include "C:\Users\Christopher\Documents\VivadoProjects\Zybo-DMA\proj\DMA.sdk\dma_bsp\ps7_cortexa9_0\include\xtime_l.h"


#ifdef XPAR_INTC_0_DEVICE_ID
 #include "xintc.h"
 #include "microblaze_sleep.h"
#else
 #include "C:\Users\Christopher\Documents\VivadoProjects\Zybo-DMA\proj\DMA.sdk\dma_bsp\ps7_cortexa9_0\include\xscugic.h"
#include "C:\Users\Christopher\Documents\VivadoProjects\Zybo-DMA\proj\DMA.sdk\dma_bsp\ps7_cortexa9_0\include\sleep.h"
#include "C:\Users\Christopher\Documents\VivadoProjects\Zybo-DMA\proj\DMA.sdk\dma_bsp\ps7_cortexa9_0\include\xil_cache.h"
#endif

/************************** Constant Definitions *****************************/

/*
 * Device hardware build related constants.
 */

// Audio constants
// Number of seconds to record/playback
#define NR_SEC_TO_REC_PLAY		5

// ADC/DAC sampling rate in Hz
//#define AUDIO_SAMPLING_RATE		1000
#define AUDIO_SAMPLING_RATE	  96000

// Number of samples to record/playback
#define NR_AUDIO_SAMPLES		(NR_SEC_TO_REC_PLAY*AUDIO_SAMPLING_RATE)

/* Timeout loop counter for reset
 */
#define RESET_TIMEOUT_COUNTER	10000

#define TEST_START_VALUE	0x0


/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/
#if (!defined(DEBUG))
extern void xil_printf(const char *format, ...);
#endif


/************************** Variable Definitions *****************************/
/*
 * Device instance definitions
 */

static XIic sIic;
static XAxiDma sAxiDma;		/* Instance of the XAxiDma */

static XGpio sUserIO;

#ifdef XPAR_INTC_0_DEVICE_ID
 static XIntc sIntc;
#else
 static XScuGic sIntc;
#endif

//
// Interrupt vector table
#ifdef XPAR_INTC_0_DEVICE_ID
const ivt_t ivt[] = {
	//IIC
	{XPAR_AXI_INTC_0_AXI_IIC_0_IIC2INTC_IRPT_INTR, (XInterruptHandler)XIic_InterruptHandler, &sIic},
	//DMA Stream to MemoryMap Interrupt handler
	{XPAR_AXI_INTC_0_AXI_DMA_0_S2MM_INTROUT_INTR, (XInterruptHandler)fnS2MMInterruptHandler, &sAxiDma},
	//DMA MemoryMap to Stream Interrupt handler
	{XPAR_AXI_INTC_0_AXI_DMA_0_MM2S_INTROUT_INTR, (XInterruptHandler)fnMM2SInterruptHandler, &sAxiDma},
	//User I/O (buttons, switches, LEDs)
	{XPAR_AXI_INTC_0_AXI_GPIO_0_IP2INTC_IRPT_INTR, (XInterruptHandler)fnUserIOIsr, &sUserIO}
};
#else
const ivt_t ivt[] = {
	//IIC
	{XPAR_FABRIC_AXI_IIC_0_IIC2INTC_IRPT_INTR, (Xil_ExceptionHandler)XIic_InterruptHandler, &sIic},
	//DMA Stream to MemoryMap Interrupt handler
	{XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR, (Xil_ExceptionHandler)fnS2MMInterruptHandler, &sAxiDma},
	//DMA MemoryMap to Stream Interrupt handler
	{XPAR_FABRIC_AXI_DMA_0_MM2S_INTROUT_INTR, (Xil_ExceptionHandler)fnMM2SInterruptHandler, &sAxiDma},
	//User I/O (buttons, switches, LEDs)
	{XPAR_FABRIC_AXI_GPIO_0_IP2INTC_IRPT_INTR, (Xil_ExceptionHandler)fnUserIOIsr, &sUserIO}
};
#endif

// SD card stuff
static FATFS FS_instance;
static FRESULT result;
static char* Path = "0:/";


/*****************************************************************************/
/**
*
* Main function
*
* This function is the main entry of the interrupt test. It does the following:
*	Initialize the interrupt controller
*	Initialize the IIC controller
*	Initialize the User I/O driver
*	Initialize the DMA engine
*	Initialize the Audio I2S controller
*	Enable the interrupts
*	Wait for a button event then start selected task
*	Wait for task to complete
*
* @param	None
*
* @return
*		- XST_SUCCESS if example finishes successfully
*		- XST_FAILURE if example fails.
*
* @note		None.
*
******************************************************************************/
int main(void)
{
	int Status;

	Demo.u8Verbose = 0;

	//Xil_DCacheDisable();

	xil_printf("\r\n--- Entering main() --- \r\n");


	//
	//Initialize the interrupt controller

	Status = fnInitInterruptController(&sIntc);
	if(Status != XST_SUCCESS) {
		xil_printf("Error initializing interrupts");
		return XST_FAILURE;
	}


	// Initialize IIC controller
	Status = fnInitIic(&sIic);
	if(Status != XST_SUCCESS) {
		xil_printf("Error initializing I2C controller");
		return XST_FAILURE;
	}

    // Initialize User I/O driver
    Status = fnInitUserIO(&sUserIO);
    if(Status != XST_SUCCESS) {
    	xil_printf("User I/O ERROR");
    	return XST_FAILURE;
    }


	//Initialize DMA
	Status = fnConfigDma(&sAxiDma);
	if(Status != XST_SUCCESS) {
		xil_printf("DMA configuration ERROR");
		return XST_FAILURE;
	}


	//Initialize Audio I2S
	Status = fnInitAudio();
	if(Status != XST_SUCCESS) {
		xil_printf("Audio initializing ERROR");
		return XST_FAILURE;
	}

	{
		XTime  tStart, tEnd;

		XTime_GetTime(&tStart);
		do {
			XTime_GetTime(&tEnd);
		}
		while((tEnd-tStart)/(COUNTS_PER_SECOND/10) < 20);
	}
	//Initialize Audio I2S
	Status = fnInitAudio();
	if(Status != XST_SUCCESS) {
		xil_printf("Audio initializing ERROR");
		return XST_FAILURE;
	}


	// Enable all interrupts in our interrupt vector table
	// Make sure all driver instances using interrupts are initialized first
	fnEnableInterrupts(&sIntc, &ivt[0], sizeof(ivt)/sizeof(ivt[0]));



	recording* recordings;
	recordings = (recording*) malloc(sizeof(recording)*16);

	for (int i=0; i < 16; i++)
	{
		recordings[i] = (recording) {.arr = {0}};
	}

	// Mount the SD card and load recordings from files
	result = f_mount(&FS_instance, Path, 1);
	xil_printf("Loading recordings from file..\r\n");

	for (int i = 0; i < 16; i++)
	{
		loadFromFile(i, (u32)&recordings[i]);
	}

	xil_printf("Done..\r\n");

	// Initializing switches & setting direction
	XGpio dip;
	XGpio_Initialize(&dip, XPAR_SWITCHES_DEVICE_ID);
	XGpio_SetDataDirection(&dip, 1, 0xffffffff);

	int dip_check = XGpio_DiscreteRead(&dip, 1);

	u32 addr = (u32) &recordings[dip_check];

	xil_printf("----------------------------------------------------------\r\n");
	xil_printf("Zybo DMA Audio Demo\r\n");
	xil_printf("----------------------------------------------------------\r\n");

    //main loop

    while(1) {

		// Checking the DMA S2MM event flag
		if (Demo.fDmaS2MMEvent)
		{
			xil_printf("\r\nRecording Done...");


			// Disable Stream function to send data (S2MM)
			Xil_Out32(I2S_STREAM_CONTROL_REG, 0x00000000);
			Xil_Out32(I2S_TRANSFER_CONTROL_REG, 0x00000000);

			Xil_DCacheInvalidateRange(addr, 5*NR_AUDIO_SAMPLES);
			//microblaze_invalidate_dcache();
			// Reset S2MM event and record flag
			Demo.fDmaS2MMEvent = 0;
			Demo.fAudioRecord = 0;

			writeToFile(dip_check, addr);
		}

		// Checking the DMA MM2S event flag
		if (Demo.fDmaMM2SEvent)
		{
			xil_printf("\r\nPlayback Done...");

			// Disable Stream function to send data (S2MM)
			Xil_Out32(I2S_STREAM_CONTROL_REG, 0x00000000);
			Xil_Out32(I2S_TRANSFER_CONTROL_REG, 0x00000000);
			//Flush cache
//					//microblaze_flush_dcache();
			Xil_DCacheFlushRange(addr, 5*NR_AUDIO_SAMPLES);
			//Reset MM2S event and playback flag
			Demo.fDmaMM2SEvent = 0;
			Demo.fAudioPlayback = 0;
		}

		// Checking the DMA Error event flag
		if (Demo.fDmaError)
		{
			xil_printf("\r\nDma Error...");
			xil_printf("\r\nDma Reset...");


			Demo.fDmaError = 0;
			Demo.fAudioPlayback = 0;
			Demo.fAudioRecord = 0;
		}

		// Checking the btn change event
		if(Demo.fUserIOEvent) {

			switch(Demo.chBtn) {
				case 'u':
					if (!Demo.fAudioRecord && !Demo.fAudioPlayback)
					{
						dip_check = XGpio_DiscreteRead(&dip, 1);
						addr = (u32) &recordings[dip_check];

						xil_printf("\r\nStart Recording...\r\n");

						fnSetMicInput();

						fnAudioRecord(sAxiDma,NR_AUDIO_SAMPLES, addr);
						Demo.fAudioRecord = 1;
					}
					else
					{
						if (Demo.fAudioRecord)
						{
							xil_printf("\r\nStill Recording...\r\n");
						}
						else
						{
							xil_printf("\r\nStill Playing back...\r\n");
						}
					}
					break;
				case 'd':
					if (!Demo.fAudioRecord && !Demo.fAudioPlayback)
					{
						dip_check = XGpio_DiscreteRead(&dip, 1);

						addr = (u32) &recordings[dip_check];
						xil_printf("\r\nStart Playback...\r\n");
						fnSetHpOutput();
						fnAudioPlay(sAxiDma,NR_AUDIO_SAMPLES, addr);
						Demo.fAudioPlayback = 1;

					}
					else
					{
						if (Demo.fAudioRecord)
						{
							xil_printf("\r\nStill Recording...\r\n");
						}
						else
						{
							xil_printf("\r\nStill Playing back...\r\n");
						}
					}
					break;
				case 'r':
					if (!Demo.fAudioRecord && !Demo.fAudioPlayback)
					{
						dip_check = XGpio_DiscreteRead(&dip, 1);
						addr = (u32) &recordings[dip_check];

						xil_printf("\r\nStart Recording...\r\n");
						fnSetLineInput();
						fnAudioRecord(sAxiDma,NR_AUDIO_SAMPLES, addr);
						Demo.fAudioRecord = 1;
					}
					else
					{
						if (Demo.fAudioRecord)
						{
							xil_printf("\r\nStill Recording...\r\n");
						}
						else
						{
							xil_printf("\r\nStill Playing back...\r\n");
						}
					}
					break;
				case 'l':
					if (!Demo.fAudioRecord && !Demo.fAudioPlayback)
					{
						dip_check = XGpio_DiscreteRead(&dip, 1);

						addr = (u32) &recordings[dip_check];
						xil_printf("\r\nStart Playback...");
						fnSetLineOutput();
						fnAudioPlay(sAxiDma,NR_AUDIO_SAMPLES, addr);
						Demo.fAudioPlayback = 1;
					}
					else
					{
						if (Demo.fAudioRecord)
						{
							xil_printf("\r\nStill Recording...\r\n");
						}
						else
						{
							xil_printf("\r\nStill Playing back...\r\n");
						}
					}
					break;
				default:
					break;
			}

			// Reset the user I/O flag
			Demo.chBtn = 0;
			Demo.fUserIOEvent = 0;


		}

    }

	xil_printf("\r\n--- Exiting main() --- \r\n");


	return XST_SUCCESS;

}


void writeToFile(int recordingIndex, u32 addr)
{
	FIL file1;
	char FileName[32];
	char* FileNamePtr;

	unsigned int bytesWr;
	uint recordingSize = 2097152;

	sprintf(FileName, "%d.txt", recordingIndex);
	FileNamePtr = (char *)FileName;
	result = f_open(&file1, FileNamePtr, FA_CREATE_ALWAYS | FA_WRITE);

	result = f_write(&file1, (const void*)addr, recordingSize, &bytesWr);

	result = f_close(&file1);
}

void loadFromFile(int recordingIndex, u32 addr)
{
	unsigned int bytesR;
	uint recordingSize = 2097152;

	FIL file1;
	char FileName[32];
	char* FileNamePtr;

	sprintf(FileName, "%d.txt", recordingIndex);
	FileNamePtr = (char *)FileName;
	result = f_open(&file1, FileNamePtr, FA_READ);

	result = f_read(&file1, (void*)addr, recordingSize, &bytesR);

	result = f_close(&file1);
}





