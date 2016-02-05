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
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  //
//                           SET THESE VARIABLES TO MEET YOUR NEEDS                                 //
//                                   1 = TRUE       0 = FALSE                                       //
//                                                                                                  //
  static XDATA const char transmitter_id[] = "ABCDE";                                               //
//                                                                                                  //
  static volatile BIT only_listen_for_my_transmitter = 1;                                           //
// 1 is recommended                                                                                 //
//                                                                                                  //
  static volatile BIT status_lights = 1;                                                            //
// if status_lights = 1; the yellow light flashes while actively scanning                           //
// if a light is flashing for more than 10 minutes straight, it may not be picking up your dex      //
//                                                                                                  //
  static volatile BIT allow_alternate_usb_protocol = 0;
// if set to 1 and plugged in to USB then protocol output is suitable for dexterity and similar     //
//                                                                                                  //
//..................................................................................................//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  //
//                 Advanced Options, dont change unless you know what you are doing                 //
//                                                                                                  //
//                                                                                                  //
  static volatile uint8 wake_earlier_for_next_miss = 20;                                            //
// if a packet is missed, wake this many seconds earlier to try and get the next one                //
// shorter means better bettery life but more likely to miss multiple packets in a row              //
//                                                                                                  //
  static volatile uint8 misses_until_failure = 2;                                                   //
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
uint32 XDATA asciiToDexcomSrc(char *addr);
uint32 XDATA getSrcValue(char srcVal);
volatile uint32 dex_tx_id;
#define NUM_CHANNELS        (4)
static int8 fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static XDATA int8 defaultfOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
static uint32 waitTimes[NUM_CHANNELS] = { 13500, 500, 500, 500 };
//Now lets try to crank down the channel 1 wait time, if we can 5000 works but it wont catch channel 4 ever
static uint32 delayedWaitTimes[NUM_CHANNELS] = { 0, 700, 700, 700 };
static uint32 catch_offsets[NUM_CHANNELS] = { 0, 0, 0, 0 };
static uint8 last_catch_channel = 0;
BIT needsTimingCalibration = 1;
BIT usbEnabled = 1;
static uint8 save_IEN0;
static uint8 save_IEN1;
static uint8 save_IEN2;
unsigned char XDATA PM2_BUF[7] = {0x06,0x06,0x06,0x06,0x06,0x06,0x04};
unsigned char XDATA dmaDesc[8] = {0x00,0x00,0xDF,0xBE,0x00,0x07,0x20,0x42};
volatile uint8 sequential_missed_packets = 0;

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

void sleepInit(void) {
   WORIRQ  |= (1<<4);
}

ISR(ST, 1) {
    IRCON &= 0x7F;
    WORIRQ &= 0xFE;
    SLEEP &= 0xFC;
}

void switchToRCOSC(void) {
    SLEEP &= ~0x04;
    while ( ! (SLEEP & 0x20) );
    CLKCON = (CLKCON & ~0x07) | 0x40 | 0x01;
    while ( !(CLKCON & 0x40) );
    SLEEP |= 0x04;
}

void uartEnable() {
    U1UCR |= 0x40; //CTS/RTS ON
    delayMs(100);
}

void uartDisable() {
    delayMs(100);
    U1UCR &= ~0x40; //CTS/RTS Off
    U1CSR &= ~0x40; // Recevier disable
}

void blink_yellow_led() {
    if(status_lights) {
        LED_YELLOW(((getMs()/250) % 2));//Blink quarter seconds
    }
}

void blink_red_led() {
    if(status_lights) {
        LED_RED(((getMs()/500) % 2));//Blink half seconds
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
    uartEnable();
    if((allow_alternate_usb_protocol==0)||!usbPowerPresent()) {
      // Classic 3 field protocol for serial/bluetooth only
      printf("%lu %hhu %d", dex_num_decoder(pPkt->raw), pPkt->battery, adcConvertToMillivolts(adcRead(0)));
    } else {
      // Protocol suitable for dexterity android application or python script when running in USB mode
      printf("%lu %lu %lu %hhu %d %hhu %d \r\n", pPkt->src_addr,dex_num_decoder(pPkt->raw),dex_num_decoder(pPkt->filtered)*2, pPkt->battery, getPacketRSSI(pPkt),pPkt->txId,adcConvertToMillivolts(adcRead(0)));
    }
    uartDisable();
}

void makeAllOutputs() {
    int XDATA i;
    for (i=1; i < 16; i++) { // in the future, this should be set to only the channels being used for output, and add the one for input
        setDigitalOutput(i, LOW);
    }
}
void makeAllOutputsLow() {
    int XDATA i;
    for (i=0; i < 16; i++) {
        setDigitalOutput(i, LOW);
    }
}

void reset_offsets() {
    int i;
    for(i=0; i<4; i++) {
        fOffset[i] = defaultfOffset[i];
    }
}

void killWithWatchdog() {
    WDCTL = (WDCTL & ~0x03) | 0x00;
    WDCTL = (WDCTL & ~0x04) | 0x08;
}

void goToSleep (int seconds) {
    adcSetMillivoltCalibration(adcReadVddMillivolts());
    makeAllOutputsLow();

    if(!needsTimingCalibration) {
        if(!usbPowerPresent()){
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

            sleepInit();

            disableUsbPullup();
            usbDeviceState = USB_STATE_DETACHED;
            usbEnabled = 0;
            SLEEP &= ~(1<<7);

            WORCTRL |= 0x03; // 2^5 periods
            switchToRCOSC();

            storedDescHigh = DMA0CFGH;
            storedDescLow = DMA0CFGL;
            storedDma0Armed = DMAARM & 0x01;
            DMAARM |= 0x81;
            dmaDesc[0] = ((unsigned int)& PM2_BUF) >> 8;
            dmaDesc[1] = (unsigned int)& PM2_BUF;

            DMA0CFGH = ((unsigned int)&dmaDesc) >> 8;
            DMA0CFGL = (unsigned int)&dmaDesc;
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

            WORCTRL |= 0x04;  // Reset
            temp = WORTIME0;
            while(temp == WORTIME0) {};
            WOREVT1 = seconds >> 8;
            WOREVT0 = seconds;

            temp = WORTIME0;
            while(temp == WORTIME0) {};

            MEMCTR |= 0x02;
            SLEEP = 0x06;
            __asm nop __endasm;
            __asm nop __endasm;
            __asm nop __endasm;
            if(SLEEP & 0x03){
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
            if(storedDma0Armed){
                DMAARM |= 0x01;
            }
            // Switch back to high speed
            boardClockInit();

            PICTL = savedPICTL;
            P0IE = savedP0IE;
            P0SEL = savedP0SEL;
            P0DIR = savedP0DIR;
            P1SEL = savedP1SEL;
            P1DIR = savedP1DIR;
            USBPOW = 1;
            USBCIE = 0b0111;
        } else {
            uint32 start_waiting = getMs();
            if(!usbEnabled) {
                usbDeviceState = USB_STATE_POWERED;
                enableUsbPullup();
                usbEnabled = 1;
            }
            delayMs(100);
            while((getMs() - start_waiting) < (seconds * 1000)) {
                doServices();
            }
        }
    }
    makeAllOutputs();
}

void putchar(char c) {
    uart1TxSendByte(c);
    if (usbPowerPresent()) {
        usbComTxSendByte(c);
    }
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
    radioMacInit();
    MCSM1 = 0;
    radioMacStrobe();
    swap_channel(nChannels[radio_chan], fOffset[radio_chan]);
}

int WaitForPacket(uint16 milliseconds, Dexcom_packet* pkt, uint8 channel) {
    uint32 start = getMs();
    uint8 * packet = 0;
    uint32 i = 0;
    uint32 six_minutes = 360000;
    int nRet = 0;
    swap_channel(nChannels[channel], fOffset[channel]);

    while (!milliseconds || (getMs() - start) < milliseconds) {
        doServices();
        blink_yellow_led();
        i++;
        if(!(i % 40000)) {
            strobe_radio(channel);
        }
        if(getMs() - start > six_minutes) {
            killWithWatchdog();
            delayMs(2000);
        }
        if (packet = radioQueueRxCurrentPacket()) {
            uint8 len = packet[0];
            fOffset[channel] += FREQEST;
            memcpy(pkt, packet, min8(len+2, sizeof(Dexcom_packet)));
            if(radioCrcPassed()) {
                if(pkt->src_addr == dex_tx_id || dex_tx_id == 0 || only_listen_for_my_transmitter == 0) {
                    pkt->txId -= channel;
                    radioQueueRxDoneWithPacket();
                    LED_YELLOW(0);
                    last_catch_channel = channel;
                    return 1;
                } else {
                    radioQueueRxDoneWithPacket();
                }
            } else {
                radioQueueRxDoneWithPacket();
                LED_YELLOW(0);
                return 0;
            }
        }
    }
    LED_YELLOW(0);
    return nRet;
}

uint32 delayFor(int wait_chan) {
    if(needsTimingCalibration) {
        return delayedWaitTimes[wait_chan];
    }
    if(!wait_chan && sequential_missed_packets) {
        return waitTimes[wait_chan] + (sequential_missed_packets * wake_earlier_for_next_miss * 2 * 1000);
    } else {
        return waitTimes[wait_chan];
    }
}

BIT get_packet(Dexcom_packet* pPkt) {
    int nChannel = 0;
    for(nChannel = start_channel; nChannel < NUM_CHANNELS; nChannel++) {
        switch(WaitForPacket(delayFor(nChannel), pPkt, nChannel)) {
        case 1:
            needsTimingCalibration = 0;
            sequential_missed_packets = 0;
            return 1;
        case 0:
            continue;
        }
    }
    sequential_missed_packets ++;
    if(sequential_missed_packets > misses_until_failure) {
        sequential_missed_packets = 0;
        needsTimingCalibration = 1;
    }
    reset_offsets();
    last_catch_channel = 0;
    return 0;
}

void setADCInputs() {
    P0INP=0; //set pull resistors on pins 0_0 - 0_5 to low
}

void configBt() {
    uartEnable();
    printf("AT+NAMExDrip");
    uartDisable();
}

void main() {
    systemInit();
    initUart1();
    P1DIR |= 0x08; // RTS
    sleepInit();

    makeAllOutputs();
    setADCInputs();

    delayMs(1000);
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

        radioMacSleep();
        if(usbPowerPresent()){
            sequential_missed_packets++;
        }
        if(sequential_missed_packets > 0) {
            int first_square = sequential_missed_packets * sequential_missed_packets * wake_earlier_for_next_miss;
            int second_square = (sequential_missed_packets - 1) * (sequential_missed_packets - 1) * wake_earlier_for_next_miss;
            int sleep_time = (268 - first_square + second_square);
            goToSleep(sleep_time);
        } else {
            goToSleep(283);
        }
        radioMacResume();
        MCSM1 = 0;
        radioMacStrobe();
    }
}
