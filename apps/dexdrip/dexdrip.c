/** XDRIP GSM Bridge:


  == Description ==
  The app uses the radio_queue libray to receive packets.  It does not
  transmit any packets.

  Received packets are forwarded via a SIM800 GSM modem to a remote web service.

  This allows for reception in to xDrip remotely using GPRS wide area networking

  GSM modem facilities added by jamorham

  based on wixel-xdrip and xbridge2

  Web service url takes cgi get parameters:

  rr = current ms clock of wixel
  lv = raw value
  lf = filtered value
  ts = ms since packet received

  populate the defines or my_transmitter_id.h file with your local web service
  gprs apn etc

  == Parameters ==
radio_channel: See description in radio_link.h.
*/


/** Dependencies **************************************************************/
//#define DEBUG 
#include <cc2511_map.h>
#include <board.h>
#include <random.h>
#include <time.h>
#include <usb.h>
#include <usb_com.h>
#include <radio_registers.h>
#include <radio_queue.h>
#include <gpio.h>
#include <uart1.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <adc.h>

/* Function prototypes ********************************************************/

void nicersleep (int secs);
int gsm_do_sequence ();
void gsm_interactive ();
void gsm_uplink ();
void usb_printf (const char *format, ...);
void gsm_wake_serial ();
int gsm_sleep_mode ();

// gsm sleep mode controlled by dtr pin

#define GSM_USE_DTR_PIN

void setADCInputs ();




//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  //
//                           SET THESE VARIABLES TO MEET YOUR NEEDS                                 //
//                                   1 = TRUE       0 = FALSE                                       //
//                                                                                                  //

#ifndef CUSTOM_TRANSMITTER_ID
#warning "Using built-in transmitter id and defines from dexdrip.c"

static XDATA const char transmitter_id[] = "ABCDE";                                               //

#define my_webservice_url	"my.example.com/receiver.cgi"
#define my_webservice_reply     "!ACK"
#define my_user_agent 		"xDrip"
#define my_gprs_apn		"apn.check.your.carrier.info"

#else
// get user specific configuration from an external untracked file
#include "my_transmitter_id.h"
#endif



//                                                                                                  //
static volatile BIT only_listen_for_my_transmitter = 1;	//
// 1 is recommended                                                                                 //
//                                                                                                  //
static volatile BIT status_lights = 1;	//
// if status_lights = 1; the yellow light flashes while actively scanning                           //
// if a light is flashing for more than 10 minutes straight, it may not be picking up your dex      //
//                                                                                                  //
static volatile BIT allow_alternate_usb_protocol = 1;
// if set to 1 and plugged in to USB then protocol output is suitable for dexterity and similar

static volatile BIT use_gsm = 1;
//                                                                                                  //
//..................................................................................................//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  //
//                 Advanced Options, dont change unless you know what you are doing                 //
//                                   1 = TRUE       0 = FALSE                                       //
//                                                                                                  //
//                                                                                                  //
static volatile uint8 wake_earlier_for_next_miss = 20;	//
// if a packet is missed, wake this many seconds earlier to try and get the next one                //
// shorter means better bettery life but more likely to miss multiple packets in a row              //
//                                                                                                  //
static volatile uint8 misses_until_failure = 2;	//
// after how many missed packets should we just start a nonstop scan?                               //
// a high value is better for conserving batter life if you go out of wixel range a lot             //
// but it could also mean missing packets for MUCH longer periods of time                           //
// a value of zero is best if you dont care at all about battery life                               //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////



static XDATA volatile int start_channel = 0;
uint32 XDATA asciiToDexcomSrc (char *addr);
uint32 XDATA getSrcValue (char srcVal);
volatile uint32 dex_tx_id;
#define NUM_CHANNELS        (4)
static int8 fOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
static XDATA int8 defaultfOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
static uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
static uint32 waitTimes[NUM_CHANNELS] = { 13500, 500, 500, 500 };
//Now lets try to crank down the channel 1 wait time, if we can 5000 works but it wont catch channel 4 ever
static uint32 delayedWaitTimes[NUM_CHANNELS] = { 0, 700, 700, 700 };
static uint32 catch_offsets[NUM_CHANNELS] = { 0, 0, 0, 0 };

static uint8 last_catch_channel = 0;
BIT needsTimingCalibration = 1;
BIT usbEnabled = 1;
static volatile BIT send_to_uart = 1;	// when doing usb printf - this is not a setting it is dynamic var
static uint8 save_IEN0;
static uint8 save_IEN1;
static uint8 save_IEN2;
unsigned char XDATA PM2_BUF[7] = { 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x04 };
unsigned char XDATA dmaDesc[8] =
{ 0x00, 0x00, 0xDF, 0xBE, 0x00, 0x07, 0x20, 0x42 };

static volatile BIT usb_connected = 0;

static uint32 XDATA gsm_delay = 0;	// how long we spent uplinking
static uint32 XDATA lastraw = 0;
static uint32 XDATA lastfiltered = 0;
static uint32 XDATA lasttime = 0;
static uint32 XDATA general_timer = 0;
volatile uint8 sequential_missed_packets = 0;

typedef struct _Dexcom_packet
{
    uint8 len;
    uint32 dest_addr;
    uint32 src_addr;
    uint8 port;
    uint8 device_info;
    uint8 txId;
    uint16 raw;
    uint16 filtered;
    uint8 battery;
    uint8 unknown;
    uint8 checksum;
    int8 RSSI;
    uint8 LQI;
} Dexcom_packet;

void
sleepInit (void)
{
    WORIRQ |= (1 << 4);
}

ISR (ST, 1)
{
    IRCON &= 0x7F;
    WORIRQ &= 0xFE;
    SLEEP &= 0xFC;
}

void
switchToRCOSC (void)
{
    SLEEP &= ~0x04;
    while (!(SLEEP & 0x20));
    CLKCON = (CLKCON & ~0x07) | 0x40 | 0x01;
    while (!(CLKCON & 0x40));
    SLEEP |= 0x04;
}

void
uartEnable ()
{
    U1UCR &= ~0x40;		//CTS/RTS Off // always off!
    U1CSR |= 0x40;		// Recevier enable
    delayMs (100);
}

void
uartDisable ()
{
    delayMs (100);
    U1UCR &= ~0x40;		//CTS/RTS Off
    U1CSR &= ~0x40;		// Recevier disable
}

void
blink_yellow_led ()
{
    if (status_lights)
    {
        LED_YELLOW (((getMs () / 250) % 2));	//Blink quarter seconds
    }
}

void
blink_red_led ()
{
    if (status_lights)
    {
        LED_RED (((getMs () / 250) % 2));	//Blink 1/4 seconds
    }
}

int8
getPacketRSSI (Dexcom_packet * p)
{
    return (p->RSSI / 2) - 73;
}

uint8
getPacketPassedChecksum (Dexcom_packet * p)
{
    return ((p->LQI & 0x80) == 0x80) ? 1 : 0;
}

uint8
bit_reverse_byte (uint8 in)
{
    uint8 XDATA bRet = 0;
    if (in & 0x01)
        bRet |= 0x80;
    if (in & 0x02)
        bRet |= 0x40;
    if (in & 0x04)
        bRet |= 0x20;
    if (in & 0x08)
        bRet |= 0x10;
    if (in & 0x10)
        bRet |= 0x08;
    if (in & 0x20)
        bRet |= 0x04;
    if (in & 0x40)
        bRet |= 0x02;
    if (in & 0x80)
        bRet |= 0x01;
    return bRet;
}

uint8
min8 (uint8 a, uint8 b)
{
    if (a < b)
        return a;
    return b;
}

void
bit_reverse_bytes (uint8 * buf, uint8 nLen)
{
    uint8 XDATA i = 0;
    for (; i < nLen; i++)
    {
        buf[i] = bit_reverse_byte (buf[i]);
    }
}

uint32
dex_num_decoder (uint16 usShortFloat)
{
    uint16 XDATA usReversed = usShortFloat;
    uint8 XDATA usExponent = 0;
    uint32 XDATA usMantissa = 0;
    bit_reverse_bytes ((uint8 *) & usReversed, 2);
    usExponent = ((usReversed & 0xE000) >> 13);
    usMantissa = (usReversed & 0x1FFF);
    return usMantissa << usExponent;
}

char XDATA SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
                                'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
                                'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y'
                              };

void
dexcom_src_to_ascii (uint32 src, char addr[6])
{
    addr[0] = SrcNameTable[(src >> 20) & 0x1F];
    addr[1] = SrcNameTable[(src >> 15) & 0x1F];
    addr[2] = SrcNameTable[(src >> 10) & 0x1F];
    addr[3] = SrcNameTable[(src >> 5) & 0x1F];
    addr[4] = SrcNameTable[(src >> 0) & 0x1F];
    addr[5] = 0;
}

void
doServices ()
{
    if (usbPowerPresent ())
    {
        boardService ();
        usbComService ();
    }
}

void
initUart1 ()
{
    uart1Init ();
    uart1SetBaudRate (9600);
}


// you can uncomment this if you want a glowing yellow LED when a terminal program is connected
// to the USB.  I got sick of it.
// LineStateChangeCallback - sets the yellow LED to the state of DTR on the USB, whenever it changes.
void
LineStateChangeCallback (uint8 state)
{
    //LED_YELLOW(state & ACM_CONTROL_LINE_DTR);
    usb_connected = state & ACM_CONTROL_LINE_DTR;
}

uint32
asciiToDexcomSrc (char addr[6])
{
    uint32 XDATA src = 0;
    src |= (getSrcValue (addr[0]) << 20);
    src |= (getSrcValue (addr[1]) << 15);
    src |= (getSrcValue (addr[2]) << 10);
    src |= (getSrcValue (addr[3]) << 5);
    src |= getSrcValue (addr[4]);
    return src;
}

uint32
getSrcValue (char srcVal)
{
    uint8 i = 0;
    for (i = 0; i < 32; i++)
    {
        if (SrcNameTable[i] == srcVal)
            break;
    }
    return i & 0xFF;
}

void
print_packet (Dexcom_packet * pPkt)
{

    lasttime = getMs ();
    lastraw = dex_num_decoder (pPkt->raw);
    lastfiltered = dex_num_decoder (pPkt->filtered) * 2;

    uartEnable ();

    if ((allow_alternate_usb_protocol == 0) || !usbPowerPresent ())
    {

        // Classic 3 field protocol for serial/bluetooth only
        if (!use_gsm)
            printf ("%lu %hhu %d", dex_num_decoder (pPkt->raw), pPkt->battery,
                    adcConvertToMillivolts (adcRead (0)));

    }
    else
    {

        // Protocol suitable for dexterity android application or python script when running in USB mode
        usb_printf ("%lu %lu %lu %hhu %d %hhu %d \r\n", pPkt->src_addr,
                    dex_num_decoder (pPkt->raw),
                    dex_num_decoder (pPkt->filtered) * 2, pPkt->battery,
                    getPacketRSSI (pPkt), pPkt->txId,
                    adcConvertToMillivolts (adcRead (0)));
    }

    gsm_delay = getMs ();
    if (use_gsm)
        gsm_uplink ();		// send data via gsm / gprs
    gsm_delay = (getMs () - gsm_delay) / 1000;


    uartDisable ();



}


///////// JAMORHAM GPRS / GSM MODEM FEATURE /////////////////


typedef struct _Serial_cmd
{
    const char *command;
    const char *response;
    const int timeout;
} Serial_cmd;


const Serial_cmd XDATA gsm_cmd[] =
{
    {"ATZ", "OK", 10},
    {"AT+CFUN=0", "", 10},
    {"AT+CSCLK=1", "OK", 2},
    {"AT+CFUN=1", "Call Ready", 30},
    {"AT+SAPBR=0,1", "", 3},
    {"AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2},
    {"AT+SAPBR=3,1,\"APN\",\"" my_gprs_apn "\"", "OK", 2},
    {"AT+SAPBR=1,1", "OK", 30},

    {"AT+HTTPTERM", "", 2},
    {"AT+HTTPINIT", "", 10},
    {"AT+HTTPPARA=\"CID\",1", "OK", 2},
    {
        "AT+HTTPPARA=\"URL\",\"" my_webservice_url "?%s\"", "OK",
        2
    },
    {"AT+HTTPPARA=\"UA\",\"" my_user_agent "\"", "OK", 2},
    {"AT+HTTPACTION=0", "+HTTPACTION: 0,200,", 60},
    {"AT+HTTPREAD", my_webservice_reply , 2},
    {"AT+HTTPTERM", "OK", 2},

// end of sequence marker
    {"END", "END", 255}
};

int
gsm_get_response (const char *response, int timeout)
__reentrant
{
    char b;
    int ptr = 0;
    int len = strlen (response);
    uint32 timeout_time = getMs () + ((uint32) timeout * 1000);

    while ((ptr < len) && (getMs () < timeout_time))
    {
        if (uart1RxAvailable ())
        {
            b = uart1RxReceiveByte ();
            if (usb_connected)
                usbComTxSendByte (b);

            if (response[ptr] == b)
            {
                ptr++;
            }
            else
            {
                ptr = 0;
            }
        }
        else
        {
            doServices ();
            delayMicroseconds (250);
        }

    }

    if ((len > 0) && (ptr < len))
    {
        usb_printf ("Timedout on %s\r\n", response);
        return 0;
    }
    else
    {
#ifdef DEBUG
        usb_printf ("Matched %s\r\n", response);
#endif
        return 1;
    }				// did we get a match
}				// function end

void
usb_printf (const char *format, ...)
{
    va_list argx;
    if (usb_connected)
    {
        send_to_uart = 0;
        va_start (argx, format);
        vprintf (format, argx);
        send_to_uart = 1;
    }
}

int
gsm_send_command (const char *command, const char *response, int timeout)
{

    char b;
    char XDATA buffer[256];
    char XDATA param[256] = "";


    sprintf (param, "rr=%lu&lv=%lu&lf=%lu&ts=%lu", getMs (), lastraw,
             lastfiltered, (getMs () - lasttime));

// read in any buffered data before we send the command

    while (uart1RxAvailable ())
    {
        b = uart1RxReceiveByte ();
        if (usb_connected)
            usbComTxSendByte (b);
    }

    sprintf (buffer, command, param);	// fill in string

    usb_printf ("GSM send command: %s / %s / %d\r\n", buffer, response,
                timeout);

    printf (buffer);
    printf ("\r\n");		// enter key
    return gsm_get_response (response, timeout);
}


/////////////////////////////


void
makeAllOutputs ()
{
    int XDATA i;
    for (i = 1; i < 16; i++)
    {
        // in the future, this should be set to only the channels being used for output, and add the one for input
        if (i != 13)
            setDigitalOutput (i, LOW);	// skip P1_3 for dtr line
    }
}

void
makeAllOutputsLow ()
{
    int XDATA i;
    for (i = 0; i < 16; i++)
    {
        if (i != 13)
            setDigitalOutput (i, LOW);	// skip P1_3 for dtr line
    }
}

void
reset_offsets ()
{
    int i;
    for (i = 0; i < 4; i++)
    {
        fOffset[i] = defaultfOffset[i];
    }
}

void
killWithWatchdog ()
{
    WDCTL = (WDCTL & ~0x03) | 0x00;
    WDCTL = (WDCTL & ~0x04) | 0x08;
}

void
goToSleep (int32 seconds)
__reentrant
{

    if (use_gsm)
	{
	LED_YELLOW(0); // shut off any indicator led
    seconds = seconds - gsm_delay;
    usb_printf ("Delay adjusted for gsm by %ld\r\n", gsm_delay);
    gsm_delay = 0;
	}

    if (seconds < 1)
        return;

    adcSetMillivoltCalibration (adcReadVddMillivolts ());
    makeAllOutputsLow ();

    if (!needsTimingCalibration)
    {
        if (!usbPowerPresent ())
        {
            unsigned char temp;
            unsigned char storedDescHigh, storedDescLow;
            BIT storedDma0Armed;
            unsigned char storedIEN0, storedIEN1, storedIEN2;

            uint8 savedPICTL = PICTL;
            BIT savedP0IE = P0IE;
            uint8 savedP0SEL = P0SEL;
            uint8 savedP0DIR = P0DIR;
            uint8 savedP1SEL = P1SEL;
            uint8 savedP1DIR = P1DIR;

            sleepInit ();

            disableUsbPullup ();
            usbDeviceState = USB_STATE_DETACHED;
            usbEnabled = 0;
            SLEEP &= ~(1 << 7);

            WORCTRL |= 0x03;	// 2^5 periods
            switchToRCOSC ();

            storedDescHigh = DMA0CFGH;
            storedDescLow = DMA0CFGL;
            storedDma0Armed = DMAARM & 0x01;
            DMAARM |= 0x81;
            dmaDesc[0] = ((unsigned int) &PM2_BUF) >> 8;
            dmaDesc[1] = (unsigned int) &PM2_BUF;

            DMA0CFGH = ((unsigned int) &dmaDesc) >> 8;
            DMA0CFGL = (unsigned int) &dmaDesc;
            DMAARM = 0x01;

            // save enabled interrupts
            storedIEN0 = IEN0;
            storedIEN1 = IEN1;
            storedIEN2 = IEN2;

            //enable sleep timer interrupt
            IEN0 |= 0xA0;

            //disable all interrupts except the sleep timer
            IEN0 &= 0xA0;
            IEN1 &= ~0x3F;
            IEN2 &= ~0x3F;

            WORCTRL |= 0x04;	// Reset
            temp = WORTIME0;
            while (temp == WORTIME0)
            {
            };
            WOREVT1 = seconds >> 8;
            WOREVT0 = seconds;

            temp = WORTIME0;
            while (temp == WORTIME0)
            {
            };

            MEMCTR |= 0x02;
            SLEEP = 0x06;
            __asm nop __endasm;
            __asm nop __endasm;
            __asm nop __endasm;
            if (SLEEP & 0x03)
            {
                __asm mov 0xD7, #0x01 __endasm;
                __asm nop __endasm;
                __asm orl 0x87, #0x01 __endasm;
                __asm nop __endasm;
            }
            IEN0 = storedIEN0;
            IEN1 = storedIEN1;
            IEN2 = storedIEN2;
            DMA0CFGH = storedDescHigh;
            DMA0CFGL = storedDescLow;
            if (storedDma0Armed)
            {
                DMAARM |= 0x01;
            }
            // Switch back to high speed
            boardClockInit ();

            PICTL = savedPICTL;
            P0IE = savedP0IE;
            P0SEL = savedP0SEL;
            P0DIR = savedP0DIR;
            P1SEL = savedP1SEL;
            P1DIR = savedP1DIR;
            USBPOW = 1;
            USBCIE = 0b0111;
        }
        else
        {
            uint32 start_waiting = getMs ();
            if (!usbEnabled)
            {
                usbDeviceState = USB_STATE_POWERED;
                enableUsbPullup ();
                usbEnabled = 1;
            }
            delayMs (100);
            while ((getMs () - start_waiting) < (seconds * 1000))
            {
                doServices ();
            }
        }
    }
    makeAllOutputs ();
}


void
putchar (char c)
{
    if (send_to_uart)
        uart1TxSendByte (c);
    if (usbPowerPresent ())
    {
        usbComTxSendByte (c);
    }
}

void
swap_channel (uint8 channel, uint8 newFSCTRL0)
{
    do
    {
        RFST = 4;			//SIDLE
    }
    while (MARCSTATE != 0x01);

    FSCTRL0 = newFSCTRL0;
    CHANNR = channel;
    RFST = 2;			//RX
}

void
strobe_radio (int radio_chan)
{
    radioMacInit ();
    MCSM1 = 0;
    radioMacStrobe ();
    swap_channel (nChannels[radio_chan], fOffset[radio_chan]);
}

int
WaitForPacket (uint16 milliseconds, Dexcom_packet * pkt, uint8 channel)
{
    uint32 start = getMs ();
    uint8 *packet = 0;
    uint32 i = 0;
    //uint32 seven_minutes = 420000;
    uint32 XDATA six_minutes = 360000;
//#define six_minutes 360000
    int nRet = 0;
    swap_channel (nChannels[channel], fOffset[channel]);

    while (!milliseconds || (getMs () - start) < milliseconds)
    {
        doServices ();
        blink_yellow_led ();
        i++;
        if (!(i % 40000))
        {
            strobe_radio (channel);
        }
        if ((getMs () - start) > six_minutes)
        {
            killWithWatchdog ();
            delayMs (2000);
        }
        if (packet = radioQueueRxCurrentPacket ())
        {
            uint8 len = packet[0];
            fOffset[channel] += FREQEST;
            memcpy (pkt, packet, min8 (len + 2, sizeof (Dexcom_packet)));
            if (radioCrcPassed ())
            {
                if (pkt->src_addr == dex_tx_id || dex_tx_id == 0
                        || only_listen_for_my_transmitter == 0)
                {
                    pkt->txId -= channel;
                    radioQueueRxDoneWithPacket ();
                    LED_YELLOW (0);
                    last_catch_channel = channel;
                    return 1;
                }
                else
                {
                    radioQueueRxDoneWithPacket ();
                }
            }
            else
            {
                radioQueueRxDoneWithPacket ();
                LED_YELLOW (0);
                return 0;
            }
        }
    }
    LED_YELLOW (0);
    return nRet;
}

uint32
delayFor (int wait_chan)
{
    if (needsTimingCalibration)
    {
        return delayedWaitTimes[wait_chan];
    }
    //return waitTimes[wait_chan];
    if (!wait_chan && sequential_missed_packets)
    {
        return waitTimes[wait_chan] +
               (sequential_missed_packets * wake_earlier_for_next_miss * 2 * 1000);
    }
    else
    {
        return waitTimes[wait_chan];
    }
}

BIT
get_packet (Dexcom_packet * pPkt)
{
    int nChannel = 0;
    for (nChannel = start_channel; nChannel < NUM_CHANNELS; nChannel++)
    {
        switch (WaitForPacket (delayFor (nChannel), pPkt, nChannel))
        {
        case 1:
            needsTimingCalibration = 0;
            sequential_missed_packets = 0;
            return 1;
        case 0:
            continue;
        }
    }
    needsTimingCalibration = 1;
//    killWithWatchdog();
//   delayMs(2000);
    sequential_missed_packets++;
    if (sequential_missed_packets > misses_until_failure)
    {
        sequential_missed_packets = 0;
        needsTimingCalibration = 1;
    }
    reset_offsets ();
    last_catch_channel = 0;
    return 0;
}

void
setADCInputs ()
{
    P0INP = 0;			//set pull resistors on pins 0_0 - 0_5 to low
}

void flashingError(int XDATA howlong,int XDATA flashspeed)
{
            general_timer=getMs()+howlong;
            while (getMs()<general_timer)
            {
                LED_RED (!(((getMs () / flashspeed)) % 2) & 1);
                LED_YELLOW ((((getMs () / flashspeed)) % 2) & 1);
                doServices();
            }
                LED_RED(0); // lights off after error
                LED_YELLOW(0);
}

void
configBt ()
{
    uartEnable ();
    if (use_gsm)
    {
        if  (!gsm_sleep_mode ())
        {
            // problem with init of gsm module
		flashingError(20000,400);
        }
#ifdef DEBUG
        gsm_interactive(); // for debug
#endif
    }
    else
    {
        printf ("AT+NAMExDrip");
    }

    uartDisable ();
}

int
gsm_sleep_mode ()
{
    int XDATA result = 1;
    gsm_wake_serial ();
    usb_printf ("Sleeping GSM\r\n");
#ifdef GSM_USE_DTR_PIN
    gsm_send_command ("AT+CSCLK=1", "OK", 2);
#else
    gsm_send_command ("ATZ", "OK", 2);
#endif
    if (!gsm_send_command ("AT+CFUN=0", "OK", 10)) result = 0; // pass error
    delayMs (2000);
#ifdef GSM_USE_DTR_PIN
    setDigitalOutput (13, HIGH);	// P1_3 set high to sleep
#endif
    return result;
}

void
gsm_wake_serial ()
__reentrant
{
    int i;
    usb_printf ("Waking GSM\r\n");
#ifdef GSM_USE_DTR_PIN
    setDigitalOutput (13, LOW);	// P1_3 set low to wake
#endif
    for (i = 0; i < 10; i++)
    {
        uart1TxSendByte (' ');
    }
    delayMs (200);

    if (!gsm_send_command ("AT", "OK", 2))
        if (!gsm_send_command ("AT", "OK", 2))
          if (!gsm_send_command ("AT", "OK", 2))
	   {
	    flashingError(20000,200); // double speed flash means could not wake
	   }
}

void
gsm_uplink ()
{
    int i = 0;
    int result;
    while ((i < 5) && (i > -1))
    {
        usb_printf ("Attempting to uplink data: try: %d\r\n", i);
        result = gsm_do_sequence ();
        i++;
        if (result)
        {
            i = -10;
        }
        else
        {
            delayMs (3000);
        }
    }
    if (i == -10)
    {
        usb_printf ("UPLINK SUCCESS!\r\n");
    }
    else
    {
        usb_printf ("UPLINK FAILED!\r\n");

    }
}

#ifdef DEBUG

void
gsm_interactive ()
__reentrant
{
    usb_printf ("Entering interactive mode\r\n");
    while (1)
    {

        char b;


        while (uart1RxAvailable ())
        {
            b = uart1RxReceiveByte ();
            if (usb_connected)
                usbComTxSendByte (b);	// send as debug output
        }
        while (usbComRxAvailable ())
        {
            b = usbComRxReceiveByte ();
            if (b == '¬')
                gsm_do_sequence ();
            if (b == '`')
            {
                usb_printf ("exit interactive\r\n");
                return;
            }
            uart1TxSendByte (b);
        }

        doServices ();
        delayMs (10);
    }				// main while

}
#endif

int
gsm_do_sequence ()
{
    int i = 0;
    int result = 1;

    LED_GREEN (1);
    LED_RED (0);
    usb_printf ("Doing GSM sequence\r\n");
    gsm_wake_serial ();
    while (result)
    {
        if (gsm_cmd[i].timeout == 255)
            break;			// exit loop when on last cmd
        LED_RED (1);
        result =
            gsm_send_command (gsm_cmd[i].command, gsm_cmd[i].response,
                              gsm_cmd[i].timeout);
        LED_RED (0);
        if (result)
            delayMs (250);
        i++;
    }
    if (result)
    {
        usb_printf ("GSM Sequence SUCCESS\r\n");
        LED_RED (0);
        LED_GREEN (0);
	LED_YELLOW (1);
        gsm_sleep_mode ();
    }
    else
    {
        usb_printf ("GSM Sequence FAILURE\r\n");
        gsm_sleep_mode ();
        LED_RED (1);
        LED_GREEN (0);
    }
    return result;
}

#ifdef DEBUG
void
nicersleep (int secs)
__reentrant
{
    int i = 0;
    int j = 0;
    while (i < secs)
    {
        doServices ();
        for (j = 0; j < 10; j++)
        {
            delayMs (100);
            blink_red_led ();
        }
        i++;
    }
    LED_RED (0);
}
#endif


void
main ()
{
    systemInit ();
    LED_YELLOW(1);		// YELLOW LED STARTS DURING POWER ON
    initUart1 ();
    P1DIR |= 0x08;		// RTS
    sleepInit ();


    LED_GREEN (1);		// Green shows when connected usb 
    //usbComRequestLineStateChangeNotification(LineStateChangeCallback);

#ifdef DEBUG
    usb_connected = 1;
#else
    usb_connected = usbPowerPresent ();
#endif


    makeAllOutputs ();
    setADCInputs ();

    delayMs (1000);
    dex_tx_id = asciiToDexcomSrc (transmitter_id);
    delayMs (1000);

    radioQueueInit ();
    radioQueueAllowCrcErrors = 1;
    MCSM1 = 0;


    // from xbridge3.c jstevensonsog

    MCSM0 &= 0x34;		// calibrate every fourth transition to and from IDLE.
    MCSM1 = 0x00;			// after RX go to idle, we don't transmit
    //MCSM2 = 0x08;
    MCSM2 = 0x17;			// terminate receiving on drop of carrier, but keep it up when packet quality is good.


    configBt ();

    while (1)
    {
        Dexcom_packet Pkt;
        memset (&Pkt, 0, sizeof (Dexcom_packet));
        //   boardService();

        if (get_packet (&Pkt))
        {
            print_packet (&Pkt);
        }

        RFST = 4;			// SIDLE = 4
        delayMs (100);

        radioMacSleep ();

        if (usbPowerPresent ())
        {
            sequential_missed_packets++;
        }
        if (sequential_missed_packets > 0)
        {
            int first_square =
                sequential_missed_packets * sequential_missed_packets *
                wake_earlier_for_next_miss;
            int second_square =
                (sequential_missed_packets - 1) * (sequential_missed_packets -
                                                   1) *
                wake_earlier_for_next_miss;
            int sleep_time = (268 - first_square + second_square);
            goToSleep (sleep_time);
        }
        else
        {
            goToSleep (283);
        }
        radioMacResume ();

        MCSM1 = 0;
        radioMacStrobe ();
    }
}

