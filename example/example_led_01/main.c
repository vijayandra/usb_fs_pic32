#include "options.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "USB/usb.h"
#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"

#include "HardwareProfile.h"
#include "pin_selection.h"

/** I N C L U D E S **********************************************************/

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"

#include "HardwareProfile.h"
#include "global_var.h"
#include "commandlist.h"

#define VERSION_INFO "IWScope 2.0.0 "
/** V A R I A B L E S ********************************************************/
#if defined(__18CXX)
#pragma udata
#endif

/* PIC32 configuration. */
/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = ON            // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
#pragma config DEBUG    = OFF           // Debugger Disabled for Starter Kit

//#elif defined(__32MX460F512L__) || defined(__32MX795F512L__)
#if defined(__32MX795F512L__)
#pragma config FSRSSEL = PRIORITY_7
#endif

#pragma GCC push_options
#pragma GCC optimize ("O0")

char USB_In_Buffer[64]  __attribute__((aligned(4)));
char USB_Out_Buffer[64] __attribute__((aligned(4)));

BOOL stringPrinted;
volatile BOOL buttonPressed;
volatile BYTE buttonCount;
static portBASE_TYPE xHigherPriorityTaskWoken=pdFALSE;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
static void USBTask_RTOS(void);
static void SuperVisor(void);
static void vTransferTask(void);
unsigned char ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);


/******************************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *****************************************************************************/
xSemaphoreHandle sem_SoftReset;
xSemaphoreHandle sem_Sync;
xSemaphoreHandle sem_Serial;
xSemaphoreHandle sem_transfer;
BYTE numBytesRead;
unsigned short tx_tmp_len;
unsigned short _tmp_Var;

static void vCMDTask(void *pvParameters)
{
    unsigned char ch=0;
    for(;;)
    {
        if(xSemaphoreTake(xCmdx, portMAX_DELAY) == pdTRUE)
        {
            emb_printf("Data filled\r\n");
            //fill_scope_data(0);
        }
    }
}

static void Sequencer_RTOS(void)
{
    const TickType_t xTransferCompleteDelay = pdMS_TO_TICKS(7500UL );
    EventBits_t uxBits;
    unsigned short l;

    for( ;; )
    {
        uxBits=xEventGroupWaitBits(xCDCEventBits,
                                   0xffff,                  /* The bit to wait for. */
                                   pdTRUE,                  /* Clear the bit before exiting the function. */
                                   pdFALSE,                 /* Only need to wait for one bit anyway. */
                                   xTransferCompleteDelay );/* The maximum time to wait for the event. */

#if 1
        if((uxBits & BIT_00)!= 0)
        {

        }
        else
        {
        }

#else
        ///////I2C1
        if((uxBits & cmdI2C1TX)!= 0)
        {
            emb_printf("message received\r\n");
        }
        ///////I2C2
        if((uxBits & cmdI2C2TX)!= 0)
        {

        }
        if((uxBits & cmdI2C2RX)!= 0)
        {

        }
        ///////INCAP
        if((uxBits & cmdINCAP)!= 0)
        {

        }
        ///////OUTCAP
        if((uxBits & cmdOUTCAP)!= 0)
        {

        }
        ///////UART1
        if((uxBits & cmdUART1TX)!= 0)
        {

        }
        if((uxBits & cmdUART1RX)!= 0)
        {

        }
        if((uxBits & cmdUART2TX)!= 0)
        {

        }
        if((uxBits & cmdUART2RX)!= 0)
        {

        }
        ///////GPIO
        if((uxBits & cmdGPIO_IN)!= 0)
        {

        }
        if((uxBits & cmdGPIO_OUT)!= 0)
        {

        }
        ///////DAC
        if((uxBits & cmdDAC)!= 0)
        {

        }
        ///////ADC
        if((uxBits & cmdADC)!= 0)
        {

        }
        else
        {
        }
#endif
    }
}

#if 0
static void HyperTerm(void)
{
    unsigned short j;

    xSemaphoreTake(sem_Serial, portMAX_DELAY);

    for( ;; )
    {
        if(xSemaphoreTake(sem_Serial, portMAX_DELAY) == pdTRUE)
        {
            //emb_printf("Filling ended");
            emb_printf(__FUNCTION__);
            if(cfrm_scope.tfrm.sco.activeChannel == 0x01)
            {
                for(j=0; j<cfrm_scope.tfrm.sco.numPoint; j++)
                {
                    emb_printf("%d\n",BANK_01A[j]);
                    vTaskDelay(10);
                }
            }
            else if(cfrm_scope.tfrm.sco.activeChannel == 0x01)
            {
                for(j=0; j<cfrm_scope.tfrm.sco.numPoint; j++)
                {
                    emb_printf("%d\n",BANK_01A[j]);
                    vTaskDelay(10);
                }
            }
            //else if(cfrm_scope.tfrm.sco.activeChannel == 0x03)
            else
            {
                for(j=0; j<(cfrm_scope.tfrm.sco.numPoint/2); j+=2)
                {
                    emb_printf("%d,%d\n",BANK_01A[j],BANK_01A[j+1]);
                    vTaskDelay(10);
                }
            }
        }
    }
}

static void vTransferTask(void)
{
    int l=0;
    for(;;)
    {
        xSemaphoreTake(sem_transfer,portMAX_DELAY);

        if((__mem_trnf_act) && circ_buffer_free_space(&c_Tx_Pool)>=64)
        {
            part_data[0]=__mem_addr;
            part_data[1]=2*((__mem_len<=28)?__mem_len:28)+4;

            if(__mem_bank)
            {
                memcpy(&part_data[2],&BANK_01A[__mem_addr],part_data[1]-4);
            }
            else
            {
                memcpy(&part_data[2],&BANK_01A[__mem_addr],part_data[1]-4);
            }

            //Enc_Frame_Buffer(&c_Tx,kPartialData,0,part_data,part_data[1]);

            circ_buffer_write(&c_Tx_Pool,c_Tx.dbuff,c_Tx.lenl);
            //xSemaphoreGive(sem_Sync);
            //emb_printf("bank=%d addr=%d len=%d\r\n",__mem_bank,__mem_addr,__mem_len);

            if(__mem_len<=28)
            {
                __mem_len      = 0;
                __mem_addr     = 0;
                __mem_trnf_act = 0;
            }
            else
            {
                __mem_len     -= 28;
                __mem_addr    += 28;
            }
        }

        do
        {
            l=pullFrame(&c_Rx_Pool,&_c_Rx);
            if(l) ext_on_arrival();
        }
        while(l);
    }
}
#endif

#define INIT_SEM(X)          X = NULL;  vSemaphoreCreateBinary(X); xSemaphoreTake(X,portMAX_DELAY);

int main(void)
{
    InitializeSystem();
    INTEnableSystemMultiVectoredInt();


    mInitSPI_CS_DAC();
    mInitSPI_CS_ADC();
    SpiConfigAsMaster();
    mInit_P3_GPIO_OUTPUT();

    SYSTEMConfigPerformance( configCPU_CLOCK_HZ );
//mOSCSetPBDIV( OSC_PB_DIV_2);

    vSemaphoreCreateBinary(sem_Sync);
    vSemaphoreCreateBinary(sem_SoftReset);
    vSemaphoreCreateBinary(sem_Serial);

    vSemaphoreCreateBinary(xTxSem);
    vSemaphoreCreateBinary(xRxSem);
    vSemaphoreCreateBinary(xCmdx);
    vSemaphoreCreateBinary(sem_transfer);

    xSemaphoreTake(sem_SoftReset, portMAX_DELAY);

    xStartUARTTask();
//vSetupTimer1(1000);
    xCDCEventBits = xEventGroupCreate();

    circ_buffer_init(&c_Rx_Pool,crx_buff,BUFFER_LEN);
    circ_buffer_init(&c_Tx_Pool,ctx_buff,BUFFER_LEN);


    OpenTimer2(T2_ON | T2_PS_1_1,600);
    OpenOC4(OC_ON | OC_TIMER_MODE16 | T2_SOURCE_INT | OC_CONTINUE_PULSE | OC_LOW_HIGH,600,400);
    OpenOC2(OC_ON | OC_TIMER_MODE16 | T2_SOURCE_INT | OC_CONTINUE_PULSE | OC_LOW_HIGH,600,100);

#if 0
    OpenTimer3(T3_ON | T3_PS_1_1,0xffff);
    OpenCapture4(IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_FEDGE_RISE | IC_ON );
    mIC4SetIntPriority(1);
    mIC4IntEnable(1);
#endif


    xTaskCreate(USBTask_RTOS,  "USB",(8*configMINIMAL_STACK_SIZE), ( void * ) 0, 2, NULL );
    xTaskCreate(SuperVisor,    "SUP",(configMINIMAL_STACK_SIZE),   ( void * ) 0, 1, NULL );
    //xTaskCreate(HyperTerm,"htp",(configMINIMAL_STACK_SIZE), ( void * ) 0, 1, NULL );
    //xTaskCreate(vCMDTask,"CMDX",(4*configMINIMAL_STACK_SIZE), ( void * ) 0, 3, NULL );
    vTaskStartScheduler();

}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif
#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;    // See HardwareProfile.h
#endif

    mInitSPI_CS_DAC();
    mInitSPI_CS_ADC();
    SpiConfigAsMaster();


    USBDeviceInit();    //usb_device.c.  Initializes USB module SFRs and firmware
    //variables to known states.
}//end InitializeSystem




/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:
 *
 *****************************************************************************/
void UserInit(void)
{
    //Initialize all of the debouncing variables
    buttonCount = 0;
    buttonPressed = FALSE;
    stringPrinted = TRUE;

    //Initialize all of the LED pins
    mInitAllLEDs();

    //Initialize the pushbuttons
    mInitAllSwitches();
}//end UserInit

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void)
{
    static WORD led_count=0;

    if(led_count == 0)led_count = 10000U;
    led_count--;

#define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
#define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
#define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
#define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(USBSuspendControl == 1)
    {
        if(led_count==0)
        {
            mLED_1_Toggle();
            if(mGetLED_1())
            {
                mLED_2_On();
            }
            else
            {
                mLED_2_Off();
            }
        }//end if
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            mLED_Both_Off();
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            mLED_Both_On();
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            mLED_Only_1_On();
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            mLED_Only_2_On();
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
                mLED_1_Toggle();
                if(mGetLED_1())
                {
                    mLED_2_Off();
                }
                else
                {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA* each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

// Note *: The "usb_20.pdf" specs indicate 500uA or 2.5mA, depending upon device classification. However,
// the USB-IF has officially issued an ECN (engineering change notice) changing this to 2.5mA for all
// devices.  Make sure to re-download the latest specifications to get all of the newest ECNs.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
    //Example power saving code.  Insert appropriate code here for the desired
    //application behavior.  If the microcontroller will be put to sleep, a
    //process similar to that shown below may be used:

    //ConfigureIOPinsForLowPower();
    //SaveStateOfAllInterruptEnableBits();
    //DisableAllInterruptEnableBits();
    //EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();    //should enable at least USBActivityIF as a wake source
    //Sleep();
    //RestoreStateOfAllPreviouslySavedInterruptEnableBits();    //Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
    //RestoreIOPinsToNormal();                                    //Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

    //IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
    //cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
    //things to not work as intended.


#if defined(__C30__) || defined __XC16__
    USBSleepOnSuspend();
#endif
}

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *                    suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *                    mode, the host may wake the device back up by sending non-
 *                    idle state signalling.
 *
 *                    This call back is invoked when a wakeup from USB suspend
 *                    is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
    // If clock switching or other power savings measures were taken when
    // executing the USBCBSuspend() function, now would be a good time to
    // switch back to normal full power run mode conditions.  The host allows
    // 10+ milliseconds of wakeup time, after which the device must be
    // fully back to normal, and capable of receiving and processing USB
    // packets.  In order to do this, the USB module must receive proper
    // clocking (IE: 48MHz clock must be available to SIE for full speed USB
    // operation).
    // Make sure the selected oscillator settings are consistent with USB
    // operation before returning from this function.
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.

    //This is reverse logic since the pushbutton is active low
    if(buttonPressed == sw2)
    {
        if(buttonCount != 0)
        {
            buttonCount--;
        }
        else
        {
            //This is reverse logic since the pushbutton is active low
            buttonPressed = !sw2;

            //Wait 100ms before the next press can be generated
            buttonCount = 100;
        }
    }
    else
    {
        if(buttonCount != 0)
        {
            buttonCount--;
        }
    }
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

    // Typically, user firmware does not need to do anything special
    // if a USB error occurs.  For example, if the host sends an OUT
    // packet to your device, but the packet gets corrupted (ex:
    // because of a bad connection, or the user unplugs the
    // USB cable during the transmission) this will typically set
    // one or more USB error interrupt flags.  Nothing specific
    // needs to be done however, since the SIE will automatically
    // send a "NAK" packet to the host.  In response to this, the
    // host will normally retry to send the packet again, and no
    // data loss occurs.  The system will typically recover
    // automatically, without the need for application firmware
    // intervention.

    // Nevertheless, this callback function is provided, such as
    // for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 *                     firmware must process the request and respond
 *                    appropriately to fulfill the request.  Some of
 *                    the SETUP packets will be for standard
 *                    USB "chapter 9" (as in, fulfilling chapter 9 of
 *                    the official USB specifications) requests, while
 *                    others may be specific to the USB device class
 *                    that is being implemented.  For example, a HID
 *                    class device needs to be able to respond to
 *                    "GET REPORT" type of requests.  This
 *                    is not a standard USB chapter 9 request, and
 *                    therefore not handled by usb_device.c.  Instead
 *                    this request should be handled by class specific
 *                    firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *                    called when a SETUP, bRequest: SET_DESCRIPTOR request
 *                    arrives.  Typically SET_DESCRIPTOR requests are
 *                    not used in most applications, and it is
 *                    optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 *                     SET_CONFIGURATION (wValue not = 0) request.  This
 *                    callback function should initialize the endpoints
 *                    for the device's usage according to the current
 *                    configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //Enable the CDC data endpoints
    CDCInitEP();
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 *                     peripheral devices to wake up a host PC (such
 *                    as if it is in a low power suspend to RAM state).
 *                    This can be a very useful feature in some
 *                    USB applications, such as an Infrared remote
 *                    control    receiver.  If a user presses the "power"
 *                    button on a remote control, it is nice that the
 *                    IR receiver can detect this signalling, and then
 *                    send a USB "command" to the PC to wake up.
 *
 *                    The USBCBSendResume() "callback" function is used
 *                    to send this special USB signalling which wakes
 *                    up the PC.  This function may be called by
 *                    application firmware to wake up the PC.  This
 *                    function will only be able to wake up the host if
 *                  all of the below are true:
 *
 *                    1.  The USB driver used on the host PC supports
 *                        the remote wakeup capability.
 *                    2.  The USB configuration descriptor indicates
 *                        the device is remote wakeup capable in the
 *                        bmAttributes field.
 *                    3.  The USB host PC is currently sleeping,
 *                        and has previously sent your device a SET
 *                        FEATURE setup packet which "armed" the
 *                        remote wakeup capability.
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior,
 *                  as a USB device that has not been armed to perform remote
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *
 *                    This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex:
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup.
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;

    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager
    //properties page for the USB device, power management tab, the
    //"Allow this device to bring the computer out of standby." checkbox
    //should be checked).
    if(USBGetRemoteWakeupStatus() == TRUE)
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE;  //So we don't execute this code again,
            //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;
            do
            {
                delay_count--;
            }
            while(delay_count);

            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }
            while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


/*******************************************************************
 * Function:        void USBCBEP0DataReceived(void)
 *
 * PreCondition:    ENABLE_EP0_DATA_RECEIVED_CALLBACK must be
 *                  defined already (in usb_config.h)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called whenever a EP0 data
 *                  packet is received.  This gives the user (and
 *                  thus the various class examples a way to get
 *                  data that is received via the control endpoint.
 *                  This function needs to be used in conjunction
 *                  with the USBCBCheckOtherReq() function since
 *                  the USBCBCheckOtherReq() function is the apps
 *                  method for getting the initial control transfer
 *                  before the data arrives.
 *
 * Note:            None
 *******************************************************************/
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        int event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           int event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size)
{
    switch( event )
    {
    case EVENT_TRANSFER:
        //Add application specific callback task or callback function here if desired.
        break;
    case EVENT_SOF:
        USBCB_SOF_Handler();
        break;
    case EVENT_SUSPEND:
        USBCBSuspend();
        break;
    case EVENT_RESUME:
        USBCBWakeFromSuspend();
        break;
    case EVENT_CONFIGURED:
        USBCBInitEP();
        break;
    case EVENT_SET_DESCRIPTOR:
        USBCBStdSetDscHandler();
        break;
    case EVENT_EP0_REQUEST:
        USBCBCheckOtherReq();
        break;
    case EVENT_BUS_ERROR:
        USBCBErrorHandler();
        break;
    case EVENT_TRANSFER_TERMINATED:
        //Add application specific callback task or callback function here if desired.
        //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
        //FEATURE (endpoint halt) request on an application endpoint which was
        //previously armed (UOWN was = 1).  Here would be a good place to:
        //1.  Determine which endpoint the transaction that just got terminated was
        //      on, by checking the handle value in the *pdata.
        //2.  Re-arm the endpoint if desired (typically would be the case for OUT
        //      endpoints).
        break;
    default:
        break;
    }
    return TRUE;
}

void vApplicationIdleHook(void)
{
    //emb_printf("T=%d\r\n",(unsigned short)(ReadADC10(8 * ((~ReadActiveBufferADC10() & 0x01)))));
    //mAD1ClearIntFlag();
    UART_flush_pending();
}

static void fillArray(void)
{
    int i;
#if 0
    for(i=0; i<MAX_SIG_POINT; i++)
    {
        _scope.adcBuffA[i]=2*i+1;
        _scope.adcBuffB[i]=2*i+2;//MAX_SIG_POINT-i;
    }
#endif
    //_scope.sco_Elem=1000;
    //fill_scope_data(0);

    _signal.activeSigChannel=3;
    _signal.sigElem   = 1000;

    _signal.sig_freq=200;
    _signal.sigA_min=0.3f;
    _signal.sigA_max=4.4f;
    _signal.sigA_type=0;

    _signal.sigB_min=0.0f;
    _signal.sigB_max=5.0f;
    _signal.sigB_type=1;

    sSignal_Gen(0,_signal.sigA_type,_signal.sigA_min ,_signal.sigA_max);
    sSignal_Gen(1,_signal.sigB_type,_signal.sigB_min ,_signal.sigB_max);
    Start_Signal_Transfer_DMA_Direct(maxSigPoints,1000,1);

#define config1     ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
#define config2     ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_ON
#define config3     ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15
#define configscan  SKIP_SCAN_ALL
#define configport  ENABLE_AN4_ANA | ENABLE_AN2_ANA

    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN4 |  ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN2);
    OpenADC10( config1, config2, config3, configport, configscan );
    EnableADC10(); // Enable the ADC

}

static void SuperVisor(void)
{
    if(xSemaphoreTake(sem_SoftReset, portMAX_DELAY) == pdTRUE)
    {
        emb_printf("ABout to perform soft reset\r\n");
        vTaskDelay(3000);
        emb_printf("<<<<<<<<<< soft reset\r\n");
        vTaskDelay(500);
        SoftReset();
    }
}

static void USBTask_RTOS(void)
{
    unsigned char _tmp_Var=0;
    int l=0;

    USBDeviceAttach();
    fillArray();

    emb_printf("--------------------------------------\r\n");
    emb_printf("----IWSCope Oscilloscope Ver %d.%d.%d----\r\n",VERSION_MAJOR ,VERSION_MINOR ,VERSION_BUGFX);
    emb_printf("- Compiled on ");
    emb_printf(__DATE__);
    emb_printf(" ");
    emb_printf(__TIME__);
    emb_printf(" ");
    emb_printf("--\r\n");
    emb_printf("-------------------------------------\r\n");

    for( ;; )
    {
        if(!((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)))
        {
            ////////////////// Start hereUSB
            if(xSemaphoreTake(sem_Sync, portMAX_DELAY) == pdTRUE)
            {
                // Handle Incomming Data and place it in Q for later processing
                if(USBUSARTIsTxTrfReady())
                {
                    numBytesRead = getsUSBUSART(USB_Out_Buffer,64);
                    circ_buffer_write(&c_Rx_Pool,USB_Out_Buffer,numBytesRead);
                    xSemaphoreGive(sem_transfer);
                }

                // Pull completed packet, process it
                l=pullFrame(&c_Rx_Pool,&_c_Rx);
                if(l)
                {
                    ext_on_arrival();
                }

                if(_tmp_Var==0)
                {
                    circ_buffer_read(&c_Tx_Pool,USB_In_Buffer,64,&_tmp_Var);
                }

                if(_tmp_Var)
                {
                    if(putUSBUSART(USB_In_Buffer,_tmp_Var)== CDC_TX_READY)
                    {
                        _tmp_Var=0;
                        xSemaphoreGive(sem_transfer);
                    }
                }

                //while ( !mAD1GetIntFlag() )
#if 0
                if(_scope.bank_Rdy)
                {
                    _scope.bank_Rdy=0;
                    emb_printf("serial data filled in\r\n");
                    emb_printf("0x%04x 0x%04x\r\n" ,_scope.BANK_01A[0] ,_scope.BANK_01B[0]);
                    emb_printf("0x%04x 0x%04x\r\n" ,_scope.BANK_01A[1] ,_scope.BANK_01B[1]);
                    emb_printf("0x%04x 0x%04x\r\n" ,_scope.BANK_01A[2] ,_scope.BANK_01B[2]);
                    emb_printf("0x%04x 0x%04x\r\n" ,_scope.BANK_01A[3] ,_scope.BANK_01B[3]);
                    emb_printf("0x%04x 0x%04x\r\n" ,_scope.BANK_01A[4] ,_scope.BANK_01B[4]);
                    xSemaphoreGive(xCmdx);
                }
#endif
                CDCTxService();
                //ext_periodic_call();

            }
        }
    }
}

#pragma GCC pop_options
/** EOF main.c *************************************************/

