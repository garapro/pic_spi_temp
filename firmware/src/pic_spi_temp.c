/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pic_spi_temp.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "pic_spi_temp.h"

#include <stdio.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

PIC_SPI_TEMP_DATA pic_spi_tempData;
/* Static buffers, suitable for DMA transfer */
#define PIC_SPI_TEMP_MAKE_BUFFER_DMA_READY  __attribute__((coherent)) __attribute__((aligned(16)))

#define PIC_SPI_TEMP_FORMAT             "Temp : %f\r\n"

static uint8_t PIC_SPI_TEMP_MAKE_BUFFER_DMA_READY writeBuffer[PIC_SPI_TEMP_USB_CDC_COM_PORT_SINGLE_WRITE_BUFFER_SIZE];
static uint8_t writeString[] = "Hello World\r\n";

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************


/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE PIC_SPI_TEMP_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void * pData,
    uintptr_t userData
)
{
    PIC_SPI_TEMP_DATA * appDataObject;
    appDataObject = (PIC_SPI_TEMP_DATA *)userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent.  */
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:
            /* This means that the host has sent some data*/
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            /*** ADD ***/
            memset( writeBuffer, 0x00, sizeof(writeBuffer));
            /*** ADD ***/
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void PIC_SPI_TEMP_USBDeviceEventHandler ( USB_DEVICE_EVENT event, void * eventData, uintptr_t context )
{
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch ( event )
    {
        case USB_DEVICE_EVENT_SOF:
            break;

        case USB_DEVICE_EVENT_RESET:

            pic_spi_tempData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData;
            if ( configuredEventData->configurationValue == 1)
            {
                /* Register the CDC Device application event handler here.
                 * Note how the pic_spi_tempData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, PIC_SPI_TEMP_USBDeviceCDCEventHandler, (uintptr_t)&pic_spi_tempData);

                /* Mark that the device is now configured */
                pic_spi_tempData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(pic_spi_tempData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(pic_spi_tempData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

/* TODO:  Add any necessary callback functions.
*/
/*** ADD ***/
static void PIC_SPI_TEMP_callbackTimer( uintptr_t context, uint32_t currTick )
{
    pic_spi_tempData.spiState = pic_spi_tempData.spiState_Next;
    return;
}

void PIC_SPI_TEMP_SPIEventHandler( DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void * context ){
    
    switch(event){
        case DRV_SPI_BUFFER_EVENT_COMPLETE:
        {
            pic_spi_tempData.spiState = pic_spi_tempData.spiState_Next;
            break;
        }
        case DRV_SPI_BUFFER_EVENT_ERROR:
        {
            pic_spi_tempData.spiState = SPI_STATE_ERROR;
            break;
        }
    }
}
/*** ADD ***/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    static void USB_TX_Task (void)
    
   Remarks:
    Feeds the USB write function. 
*/
static void USB_TX_Task (void)
{
    /*** ADD ***/
    uint8_t size;
    /*** ADD ***/
    
    if(!pic_spi_tempData.isConfigured)
    {
        pic_spi_tempData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
    }
    else
    {
        /* Schedule a write if data is pending 
         */
        /*** ADD ***/
        //if ((pic_spi_tempData.writeLen > 0)/* && (pic_spi_tempData.writeTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID)*/)
        //{
        //    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
        //                         &pic_spi_tempData.writeTransferHandle,
        //                         writeBuffer, 
        //                         pic_spi_tempData.writeLen,
        //                         USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
        //}
        size = strlen(writeBuffer);
        if ((size > 0) && (pic_spi_tempData.writeTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID)){
            USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                                 &pic_spi_tempData.writeTransferHandle,
                                 writeBuffer, 
                                 size,
                                 USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
        }
        /*** ADD ***/
    }
}


/* TODO:  Add any necessary local functions.
*/
/*** ADD ***/
static void PIC_SPI_Tasks( void ){
    
    uint16_t tmpData;
    float temp;
    
    switch(pic_spi_tempData.spiState){
        case SPI_STATE_INIT:
        {
            pic_spi_tempData.spiState = SPI_STATE_ID;
            CSOff();
            break;
        }
        case SPI_STATE_ID:
        {
            pic_spi_tempData.spiState = SPI_STATE_WAIT;
            pic_spi_tempData.spiState_Next = SPI_STATE_PERIODTIMER;
            // ID取得
            pic_spi_tempData.writeBuf[0] = 0x58;
            DRV_SPI_BufferAddWriteRead2(
                    pic_spi_tempData.spiHandle,
                    pic_spi_tempData.writeBuf,
                    1,
                    pic_spi_tempData.readBuf,
                    2,
                    PIC_SPI_TEMP_SPIEventHandler,
                    NULL,
                    &pic_spi_tempData.spiBufferHandle
                    );
            if( pic_spi_tempData.spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID ){
                pic_spi_tempData.spiState = SPI_STATE_ID;
            }
            break;
        }
        case SPI_STATE_PERIODTIMER:
        {
            pic_spi_tempData.spiState = SPI_STATE_WAIT;
            pic_spi_tempData.spiState_Next = SPI_STATE_ONESHOT;
            
            pic_spi_tempData.timerHandle = SYS_TMR_ObjectCreate(1000, 0, PIC_SPI_TEMP_callbackTimer, SYS_TMR_FLAG_SINGLE | SYS_TMR_FLAG_AUTO_DELETE );
            if( pic_spi_tempData.timerHandle == SYS_TMR_HANDLE_INVALID){
                pic_spi_tempData.spiState = SPI_STATE_PERIODTIMER;
            }
            break;
        }
        case SPI_STATE_ONESHOT:
        {
            pic_spi_tempData.spiState = SPI_STATE_WAIT;
            pic_spi_tempData.spiState_Next = SPI_STATE_TEMPTIMER;
            // Config設定
            pic_spi_tempData.writeBuf[0] = 0x08;
            pic_spi_tempData.writeBuf[1] = 0x50;
            DRV_SPI_BufferAddWrite2(
                    pic_spi_tempData.spiHandle,
                    pic_spi_tempData.writeBuf,
                    2,
                    PIC_SPI_TEMP_SPIEventHandler,
                    NULL,
                    &pic_spi_tempData.spiBufferHandle
                    );
            if( pic_spi_tempData.spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID ){
                pic_spi_tempData.spiState = SPI_STATE_ID;
            }
            break;
        }
        case SPI_STATE_TEMPTIMER:
        {
            pic_spi_tempData.spiState = SPI_STATE_WAIT;
            pic_spi_tempData.spiState_Next = SPI_STATE_DATA;
            
            pic_spi_tempData.timerHandle = SYS_TMR_ObjectCreate(240, 0, PIC_SPI_TEMP_callbackTimer, SYS_TMR_FLAG_SINGLE | SYS_TMR_FLAG_AUTO_DELETE );
            if( pic_spi_tempData.timerHandle == SYS_TMR_HANDLE_INVALID){
                pic_spi_tempData.spiState = SPI_STATE_TEMPTIMER;
            }
            break;
        }
        case SPI_STATE_DATA:
        {
            pic_spi_tempData.spiState = SPI_STATE_WAIT;
            pic_spi_tempData.spiState_Next = SPI_STATE_WRITE_BUFFER;
            
            pic_spi_tempData.writeBuf[0] = 0x50;
            
            // 温度取得
            DRV_SPI_BufferAddWriteRead2(
                    pic_spi_tempData.spiHandle,
                    pic_spi_tempData.writeBuf,
                    1,
                    pic_spi_tempData.readBuf,
                    3,
                    PIC_SPI_TEMP_SPIEventHandler,
                    NULL,
                    &pic_spi_tempData.spiBufferHandle
                    );
            if( pic_spi_tempData.spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID ){
                pic_spi_tempData.spiState = SPI_STATE_DATA;
            }
            break;
        }
        case SPI_STATE_WRITE_BUFFER:
        {
            // 温度取得
            tmpData = (((uint16_t)(pic_spi_tempData.readBuf[1])<<8) & 0xFF00);
            tmpData = tmpData | pic_spi_tempData.readBuf[2];
            
            if( tmpData >= 32768 ){
                temp = (float)((( tmpData & 0x7FF8 ) >> 3 ) & 0x0FFF )/16 * -1;
            } else {
                temp = (float)((( tmpData & 0x7FF8 ) >> 3 ) & 0x0FFF )/16;
            }
            // USB送信バッファ書き込み
            // 書込み後、USB_TX_TASKS で送信する
            memset( writeBuffer, 0x00, sizeof(writeBuffer) );
            sprintf( writeBuffer, PIC_SPI_TEMP_FORMAT, temp );
            pic_spi_tempData.spiState = SPI_STATE_PERIODTIMER;
            break;
        }
    }
}
/*** ADD***/
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PIC_SPI_TEMP_Initialize ( void )

  Remarks:
    See prototype in pic_spi_temp.h.
 */

void PIC_SPI_TEMP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    pic_spi_tempData.state = PIC_SPI_TEMP_STATE_INIT;


    /* Device Layer Handle  */
    pic_spi_tempData.deviceHandle = USB_DEVICE_HANDLE_INVALID ;

    /* Device configured status */
    pic_spi_tempData.isConfigured = false;

    /* Initial get line coding state */
    pic_spi_tempData.getLineCodingData.dwDTERate   = 9600;
    pic_spi_tempData.getLineCodingData.bParityType =  0;
    pic_spi_tempData.getLineCodingData.bParityType = 0;
    pic_spi_tempData.getLineCodingData.bDataBits   = 8;


    /* Write Transfer Handle */
    pic_spi_tempData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
    
    /*Initialize the write data */
    pic_spi_tempData.writeLen = sizeof(writeString);
	memcpy(writeBuffer, writeString, pic_spi_tempData.writeLen);
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    /*** ADD ***/
    pic_spi_tempData.spiState = SPI_STATE_INIT;
    pic_spi_tempData.spiState_Next = SPI_STATE_INIT;
    
    pic_spi_tempData.spiHandle = DRV_HANDLE_INVALID;
    pic_spi_tempData.spiBufferHandle = DRV_SPI_BUFFER_HANDLE_INVALID;
    
    memset( pic_spi_tempData.writeBuf, 0x00, sizeof(pic_spi_tempData.writeBuf));
    memset( pic_spi_tempData.readBuf, 0x00, sizeof(pic_spi_tempData.readBuf));
    
    pic_spi_tempData.temp = 0;
    
    pic_spi_tempData.timerHandle = SYS_TMR_HANDLE_INVALID;
    /*** ADD ***/
}


/******************************************************************************
  Function:
    void PIC_SPI_TEMP_Tasks ( void )

  Remarks:
    See prototype in pic_spi_temp.h.
 */

void PIC_SPI_TEMP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( pic_spi_tempData.state )
    {
        /* Application's initial state. */
        case PIC_SPI_TEMP_STATE_INIT:
        {
            bool appInitialized = true;
       

            /* Open the device layer */
            if (pic_spi_tempData.deviceHandle == USB_DEVICE_HANDLE_INVALID)
            {
                pic_spi_tempData.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                                               DRV_IO_INTENT_READWRITE );
                appInitialized &= ( USB_DEVICE_HANDLE_INVALID != pic_spi_tempData.deviceHandle );
            }
            
            /*** ADD ***/
            if (pic_spi_tempData.spiHandle == DRV_HANDLE_INVALID)
            {
                pic_spi_tempData.spiHandle = DRV_SPI_Open( DRV_SPI_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING );
                appInitialized &= ( DRV_HANDLE_INVALID != pic_spi_tempData.spiHandle );
            }
            /*** ADD ***/
        
            if (appInitialized)
            {

                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(pic_spi_tempData.deviceHandle,
                                           PIC_SPI_TEMP_USBDeviceEventHandler, 0);
            
                pic_spi_tempData.state = PIC_SPI_TEMP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case PIC_SPI_TEMP_STATE_SERVICE_TASKS:
        {
            USB_TX_Task();
            
            /*** ADD ***/
            PIC_SPI_Tasks();
            /*** ADD ***/
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
