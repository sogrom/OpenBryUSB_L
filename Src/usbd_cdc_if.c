/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          :
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Defines
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  1024

/* USER CODE END PRIVATE_DEFINES */
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_Variables
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Send Data over USB CDC are stored in this buffer       */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */
static int8_t CDC_Init_FS     (void);
static int8_t CDC_DeInit_FS   (void);
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/*
 * these are the support routines needed to use the ST USB device
 * support for input and output.  For output it is fairly straightforward
 * and you can use the routine provided in usbd_cdc_if.c.  For receive,
 * we need to put a redirect into the CDC_Receive_FS() routine to call
 * our routine here called USB_Receive_FS() which takes any characters in
 * the USB receive buffer and transfers them to a FIFO here which emulates
 * the DMA FIFO when a UART is being used.
 *
 * After that is taken care of, this routine returns to CDC_Receive_FS()
 * where the USB receive process is reinitialized and started so that the
 * next buffer will be received
 *
 * Also, the two other mods to
 * usbd_cdc_if.c are 1. add #include "usb.h" to find this routine, and
 * 2. increase the input buffer size #define APP_RX_DATA_SIZE to 512
 * from 4 to ensure input buffer does not overflow while other stuff goes
 * on in case lots of data is being sent fast on the link
 */




#define RxBufferSize 1024

char RxBuffer[RxBufferSize];
volatile uint readIndex = RxBufferSize;
volatile uint RxFifoIndex = 0;

//volatile uint8_t RxFIFO[512];
//volatile uint16_t FIFOput=0, FIFOget=0;


void dummy(void);

void dummy(void){
     volatile static int i;

     for (i=0;i<50;i++);

     i++;
}





/*
 * the _write() routine is used by stdio output calls like printf().  It repeatedly
 * tries to send a string until it gets back an OK from the HAL call. I have tried
 * to make this work with interrupt and DMA service and neither works it seems.
 * Not sure why.
 */
int _write(int FD, char * outstr, int len){

     while(CDC_Transmit_FS((uint8_t*)outstr,len) == USBD_BUSY); //start new transmit
 // for some reason printf() needs this recovery time
    //dummy();
     return len;
}

/*
 * the _read() routine returns any characters that have been placed into
 * the RxBuffer by the USB receive FIFO process which emulates the UART
 * DMA circular buffer
 *
 * The UART receiver is set up with DMA in circular buffer mode.  This means that
 * it will continue receiving characters forever regardless of whether any are
 * taken out.  It will just write over the previous buffer if it overflows
 *
 * To know whether there are any characters in the buffer we look at the counter
 * register in the DMA channel assigned to the UART Rx buffer.  This count starts
 * at the size of the transfer and is decremented after a transfer to memory is done.
 *
 * We maintain a mirror DMA count register value readIndex.  If they are the same no data is
 * available.  If they are different, the DMA has decremented its counter
 * so we transfer data until they are the same or the rx buffer is full.  We
 * wrap our down counter the same way the DMA does in circular mode.
 */

int _read(int fd, char *instring, uint count){
     uint32_t bytesread = 0;
//     extern USBD_CDC_ItfTypeDef  USBD_Interface_fops_FS;

     if(count > bytesread){
          while(readIndex == (RxBufferSize - RxFifoIndex)){

          }
          {
               while((count > bytesread) & (readIndex !=(RxBufferSize - RxFifoIndex ))){
                    instring[bytesread] = RxBuffer[RxBufferSize - readIndex];
                    if(readIndex == (0))
                         readIndex = RxBufferSize;
                    else readIndex--;
                    bytesread++;
               }
          }
     }
     return (int)bytesread;
}


/*
 *  the kbhit() routine is used to check if there is a character or more available
 *  in the receive buffer.  If the buffer is empty it returns FALSE, and if there
 *  is at least one character it returns TRUE.  It is useful to check for
 *  activity before stalling a routine waiting for keyboard input if other work
 *  can be done.  It does not take a character from the buffer and does not affect
 *  any state
 */
int kbhit(void){
     if(readIndex == (RxBufferSize - RxFifoIndex))
          return 0;
     else return 1;
     return 0;
}

/*
 *
 */




/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */ 
  
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = 
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,  
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CDC_Init_FS
  *         Initializes the CDC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{ 
  /* USER CODE BEGIN 3 */ 
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */ 
}

/**
  * @brief  CDC_DeInit_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */ 
  return (USBD_OK);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CDC_Control_FS
  *         Manage the CDC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  /* USER CODE BEGIN 5 */
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
 
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
 
    break;

  case CDC_SET_COMM_FEATURE:
 
    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */ 
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:   
	
    break;

  case CDC_GET_LINE_CODING:     

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:
 
    break;    
    
  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */

/*
 * This routine is called by CDC_Receive_FS() which is a callback
 * from the underlying USB Device servicing code which gets called
 * when there is data received on the USB endpoint for the device.
 * This routine therefore acts like a circular buffer DMA for data
 * received on the USB port similar to the behavior of the circulat
 * buffer implemented on the UART receive DMA when a UART is used
 * for console I/O
 */

static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len)
{
     volatile uint32_t counter = 0;

     while(counter < *Len){
          RxBuffer[RxFifoIndex ] = pbuf[counter];
          counter++, RxFifoIndex++;
          if(RxFifoIndex  == RxBufferSize)
               RxFifoIndex  = 0;
          }

  return (USBD_OK);
}

//static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
//{
//  /* USER CODE BEGIN 6 */
//  int i;
//  int pos;
//
//  NVIC_DisableIRQ(USB_LP_IRQn);
//  pos = ring_rx_pos + ring_rx_len;
//  NVIC_EnableIRQ(USB_LP_IRQn);
//
//  for(i=0; i<*Len; i++)
//    ring_rx_buf[(pos+i) & (_GETC_BUF_SIZE - 1)] = Buf[i];
//
//  ring_rx_len += *Len;
//
//  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
//
//  rx_set_next();
//
//  return (USBD_OK);
//  /* USER CODE END 6 */
//}

/**
  * @brief  CDC_Transmit_FS
  *         Data send over USB IN endpoint are sent over CDC interface 
  *         through this function.           
  *         @note
  *         
  *                 
  * @param  Buf: Buffer of data to be send
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */ 
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */ 
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

