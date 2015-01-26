/** DEXDRIP Translator:
  == Description ==
  The app uses the radio_queue libray to receive packets.  It does not
  transmit any packets.

  The output from this app takes the following format:
  RAWREADING TRANSMITTERBATTERY WIXELBATTERY

  The green LED indicates that data was just sent

  PLEASE BE SURE TO SET YOUR TRANSMITTER ID BELOW

  == Parameters ==
radio_channel: See description in radio_link.h.
*/


/** Dependencies **************************************************************/
/*#define DEBUG*/
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
#include <string.h>
#include <ctype.h>
#include <adc.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//..................SET THESE VARIABLES TO MEET YOUR NEEDS..........................................//
static XDATA const char transmitter_id[] = "ABCDE";                                                 //
static volatile BIT only_listen_for_my_transmitter = 0;                                             //
static volatile BIT status_lights = 1;                                                              //
static volatile BIT is_BLE = 1;                                                                     //
//..................................................................................................//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

static XDATA volatile int start_channel = 0;
uint32 XDATA asciiToDexcomSrc(char *addr);
uint32 XDATA getSrcValue(char srcVal);
volatile uint32 dex_tx_id;
#define NUM_CHANNELS        (4)
static int8 fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static XDATA int8 defaultfOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
static uint32 waitTimes[NUM_CHANNELS] = { 2500, 100, 100, 2000 };
//Now lets try to crank down the channel 1 wait time, if we can 5000 works but it wont catch channel 4 ever
static uint32 delayedWaitTimes[NUM_CHANNELS] = { 0, 500, 500, 500 };
BIT usb = 1;
BIT needsTimingCalibration = 1;

typedef struct _Dexcom_packet {
    uint8   len;
    uint32  dest_addr;
    uint32  src_addr;
    uint8   port;
    uint8   device_info;
    uint8   txId;
    uint16  raw;
    uint16  filtered;
    uint8   battery;
    uint8   unknown;
    uint8   checksum;
    int8    RSSI;
    uint8   LQI;
} Dexcom_packet;

void uartEnable() {
    LED_GREEN(1);
    U1UCR |= 0x40; //CTS/RTS ON
    delayMs(1000);
}

void uartDisable() {
    delayMs(1000);
    U1UCR &= ~0x40; //CTS/RTS Off
    U1CSR &= ~0x40; // Recevier disable
    LED_GREEN(0);
}

void blink_yellow_led() {
    if(status_lights) {
        LED_YELLOW(((getMs()/1000) % 2));
    }
}

void blink_red_led() {
    if(status_lights) {
        LED_RED(((getMs()/1000) % 2));
    }
}

int8 getPacketRSSI(Dexcom_packet* p) {
    return (p->RSSI/2)-73;
}

uint8 getPacketPassedChecksum(Dexcom_packet* p) {
    return ((p->LQI & 0x80)==0x80) ? 1:0;
}

uint8 bit_reverse_byte(uint8 in) {
    uint8 XDATA bRet = 0;
    if(in & 0x01)
        bRet |= 0x80;
    if(in & 0x02)
        bRet |= 0x40;
    if(in & 0x04)
        bRet |= 0x20;
    if(in & 0x08)
        bRet |= 0x10;
    if(in & 0x10)
        bRet |= 0x08;
    if(in & 0x20)
        bRet |= 0x04;
    if(in & 0x40)
        bRet |= 0x02;
    if(in & 0x80)
        bRet |= 0x01;
    return bRet;
}

uint8 min8(uint8 a, uint8 b) {
    if(a < b) return a;
    return b;
}

void bit_reverse_bytes(uint8* buf, uint8 nLen) {
    uint8 XDATA i = 0;
    for(; i < nLen; i++) {
        buf[i] = bit_reverse_byte(buf[i]);
    }
}

uint32 dex_num_decoder(uint16 usShortFloat) {
    uint16 XDATA usReversed = usShortFloat;
    uint8 XDATA usExponent = 0;
    uint32 XDATA usMantissa = 0;
    bit_reverse_bytes((uint8*)&usReversed, 2);
    usExponent = ((usReversed & 0xE000) >> 13);
    usMantissa = (usReversed & 0x1FFF);
    return usMantissa << usExponent;
}

char XDATA SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
                          '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
                          'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
                          'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y' };

void dexcom_src_to_ascii(uint32 src, char addr[6]) {
    addr[0] = SrcNameTable[(src >> 20) & 0x1F];
    addr[1] = SrcNameTable[(src >> 15) & 0x1F];
    addr[2] = SrcNameTable[(src >> 10) & 0x1F];
    addr[3] = SrcNameTable[(src >> 5) & 0x1F];
    addr[4] = SrcNameTable[(src >> 0) & 0x1F];
    addr[5] = 0;
}

void doServices() {
    if(usbPowerPresent()) {
        boardService();
        usbComService();
    }
}

void initUart1() {
    uart1Init();
    uart1SetBaudRate(9600);
}

uint32 asciiToDexcomSrc(char addr[6]) {
    uint32 XDATA src = 0;
    src |= (getSrcValue(addr[0]) << 20);
    src |= (getSrcValue(addr[1]) << 15);
    src |= (getSrcValue(addr[2]) << 10);
    src |= (getSrcValue(addr[3]) << 5);
    src |= getSrcValue(addr[4]);
    return src;
}

uint32 getSrcValue(char srcVal) {
    uint8 i = 0;
    for(i = 0; i < 32; i++) {
        if (SrcNameTable[i]==srcVal) break;
    }
    return i & 0xFF;
}

void print_packet(Dexcom_packet* pPkt) {
    char params[30];
    int x;
    memset(params, 0, sizeof(params));
    sprintf(params, "%lu %hhu %d", dex_num_decoder(pPkt->raw), pPkt->battery, adcConvertToMillivolts(adcRead(0)));

    /* strlen doesn't seem to work */
    for (x = 0; x < 30; x++)
        if (params[x] == 0)
            break;

    uartEnable();
    if (is_BLE)
        printf("%s", params);
    else
        printf("%d %s", x, params);
    uartDisable();
}

void makeAllOutputs() {
    int XDATA i;
    for (i=6; i < 16; i++) { // in the future, this should be set to only the channels being used for output, and add the one for input
        setDigitalOutput(i, LOW);
    }
}

void rest_offsets() {
    int i;
    for(i=0; i<4; i++) {
        fOffset[i] = defaultfOffset[i];
    }
}

ISR (ST, 0) {
    IRCON &= ~0x80;
    SLEEP &= ~0x02;
    IEN0 &= ~0x20;
    WORIRQ &= ~0x11;
    WORCTRL &= ~0x03;
    if(usbPowerPresent()) {
         usbPoll();
    }
}

void goToSleep (uint16 seconds) {
    unsigned char temp;

    if(!usbPowerPresent()) {
        if(usb) {
            usb = 0;
            needsTimingCalibration = 1;
        }
        if(needsTimingCalibration) {
            seconds = 1;
        }

        adcSetMillivoltCalibration(adcReadVddMillivolts());
        IEN0 |= 0x20; // Enable global ST interrupt [IEN0.STIE]
        WORIRQ |= 0x10; // enable sleep timer interrupt [EVENT0_MASK]

        /*SLEEP |= 0x02;                  // SLEEP.MODE = PM2*/
        SLEEP |= 0x01;                  // SLEEP.MODE = PM2


        disableUsbPullup();
        usbDeviceState = USB_STATE_DETACHED;

        WORCTRL |= 0x04;  // Reset
        temp = WORTIME0;
        while (temp == WORTIME0) {};
        temp = WORTIME0;
        while (temp == WORTIME0) {};
        WORCTRL |= 0x03; // 2^5 periods
        WOREVT1 = (seconds >> 8);
        WOREVT0 = (seconds & 0xff);
        PCON |= 0x01; // PCON.IDLE = 1;
    } else {
        if(!usb) {
            usb = 1;
        }
        usbDeviceState = USB_STATE_POWERED;
        enableUsbPullup();
        needsTimingCalibration = 1;
    }
}

void putchar(char c) {
    uart1TxSendByte(c);
    if (usbPowerPresent())
        usbComTxSendByte(c);
}

void swap_channel(uint8 channel, uint8 newFSCTRL0) {
    do {
        RFST = 4;   //SIDLE
    } while (MARCSTATE != 0x01);

    FSCTRL0 = newFSCTRL0;
    CHANNR = channel;
    RFST = 2;   //RX
}

void strobe_radio(int radio_chan) {
    LED_RED(1);
    radioMacInit();
    MCSM1 = 0;
    radioMacStrobe();
    swap_channel(nChannels[radio_chan], fOffset[radio_chan]);
    LED_RED(0);
}

int WaitForPacket(uint16 milliseconds, Dexcom_packet* pkt, uint8 channel) {
    uint32 start = getMs();
    uint8 * packet = 0;
    uint32 i = 0;
    int nRet = 0;
    swap_channel(nChannels[channel], fOffset[channel]);

    while (!milliseconds || (getMs() - start) < milliseconds) {
        i++;
        if(!(i % 100000)) {
            strobe_radio(channel);
        }
        doServices();
        blink_yellow_led();
        if (packet = radioQueueRxCurrentPacket()) {
            uint8 len = packet[0];
            /*fOffset[channel] += FREQEST;*/
            memcpy(pkt, packet, min8(len+2, sizeof(Dexcom_packet)));
            if(radioCrcPassed()) {
                if(pkt->src_addr == dex_tx_id || dex_tx_id == 0 || only_listen_for_my_transmitter == 0) {
                    pkt->txId -= channel;
                    nRet = 1;
                }
            }
            radioQueueRxDoneWithPacket();
            LED_YELLOW(0);
            return nRet;
        }
    }
    LED_YELLOW(0);
    return nRet;
}

void set_lights(int chan_catch){
    if(status_lights) {
        switch(chan_catch) {
        case 1:
            LED_RED(1);
        case 2:
            LED_YELLOW(1);
        case 3:
            LED_RED(1);
            LED_YELLOW(1);
        case 5:
            LED_GREEN(0);
            LED_RED(0);
            LED_YELLOW(0);
        }
    }
}

uint32 delayFor(int wait_chan) {
    if(needsTimingCalibration) {
        return delayedWaitTimes[wait_chan];
    }
    return waitTimes[wait_chan];
}

BIT get_packet(Dexcom_packet* pPkt) {
    int nChannel = 0;
    set_lights(5);
    for(nChannel = start_channel; nChannel < NUM_CHANNELS; nChannel++) {
        switch(WaitForPacket(delayFor(nChannel), pPkt, nChannel)) {
        case 1:
            set_lights(nChannel);
            needsTimingCalibration = 0;
            return 1;
        case 0:
            continue;
        }
    }
    needsTimingCalibration = 1;
    /*rest_offsets();*/
    return 0;
}

void setADCInputs() {
    P0INP=0; //set pull resistors on pins 0_0 - 0_5 to low
}

void configBt() {
    uartEnable();
    printf("AT+NAMEDexDrip");
    uartDisable();
}

void main() {
    systemInit();
    initUart1();
    P1DIR |= 0x08; // RTS

    makeAllOutputs();
    setADCInputs();

    delayMs(1000);
    if (is_BLE)
        configBt();
    dex_tx_id= asciiToDexcomSrc(transmitter_id);
    delayMs(1000);

    radioQueueInit();
    radioQueueAllowCrcErrors = 1;
    MCSM1 = 0;

    while(1) {
        Dexcom_packet Pkt;
        memset(&Pkt, 0, sizeof(Dexcom_packet));
        boardService();
        if(get_packet(&Pkt)) {
            print_packet(&Pkt);
        }

        RFST = 4;
        delayMs(100);
        doServices();
        goToSleep(260); // Reduce this until we are just on the cusp of missing on the first channels
        //265 seemed a little too long still
        //261 seemed a little too short
        //263 seemed pretty good, first packet loss was after three hours, then missed a few of them before getting into a good groove
        //Currently attempting to up the wait times on the channels and dial the sleep down to 261
        //Now trying to drop the wait times a bit to make sure we can still hit channel 4 if we miss channel 1
        USBPOW = 1;
        USBCIE = 0b0111;
        radioMacInit();
        MCSM1 = 0;
        radioMacStrobe();
    }
}
