/** DEXDRIP Translator:
  == Description ==
  The app uses the radio_queue libray to receive packets.  It does not
  transmit any packets.

  The output from this app takes the following format:
  RAWREADING FILTEREDREADING TRANSMITTERBATTERY

  The green LED indicates that data was just sent

  PLEASE BE SURE TO SET YOUR TRANSMITTER ID BELOW

  Also, if you dont want to use usb, set it to zero and set close usb to 1

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

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//..................SET THESE VARIABLES TO MEET YOUR NEEDS..........................................//
//                           0 = false, 1 = true                                                    //
static volatile BIT usbEnabled = 1;                                                                 //
static const char XDATA transmitter_id[] = "ABCDE";                                                       //
static volatile BIT do_close_usb = 1;                                                               //
static volatile BIT only_listen_for_my_transmitter = 0;                                             //
//..................................................................................................//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

extern uint8 channel_number = 0;
static volatile uint8 XDATA start_channel = 0;
extern volatile BIT channel_select = 0;
uint32 asciiToDexcomSrc(char *addr);
uint32 getSrcValue(char srcVal);
volatile uint32 dex_tx_id;
#define NUM_CHANNELS        (4)
static uint8 XDATA fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static uint8 XDATA nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
static uint32 XDATA five_minutes = (60000 * 5);
uint32 XDATA initial_wait = 0;
uint32 XDATA second_wait = 0;
uint32 XDATA first_wait_fourth_channel = 0;
uint32 XDATA delay_offset = 0;
uint16 XDATA wait_before_known_miss;
uint16 XDATA fixed_wait_adder;


uint8 lookup[16] = {
    0x0, 0x8, 0x4, 0xC,
    0x2, 0xA, 0x6, 0xE,
    0x1, 0x9, 0x5, 0xD,
    0x3, 0xB, 0x7, 0xF };

uint8 flip( uint8 n ) {
    return (lookup[n&0x0F] << 4) | lookup[n>>4];
}

uint32 dDecode(uint8 offset,uint8 XDATA * pkt)  {
    uint32 rawValue = 0;
    rawValue = (flip (pkt[offset+1]) & 0x1F)<< 8;
    rawValue |= flip (pkt[offset]);
    rawValue = rawValue << ((flip (pkt[offset+1]) & 0xE0)>>5 );
    return rawValue;
}

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

int8 getPacketRSSI(Dexcom_packet* p) {
    return (p->RSSI/2)-73;
}

uint8 getPacketPassedChecksum(Dexcom_packet* p) {
    return ((p->LQI & 0x80)==0x80) ? 1:0;
}

uint8 bit_reverse_byte(uint8 in) {
    uint8 bRet = 0;
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
    uint8 i = 0;
    for(; i < nLen; i++) {
        buf[i] = bit_reverse_byte(buf[i]);
    }
}

uint32 dex_num_decoder(uint16 usShortFloat) {
    uint16 usReversed = usShortFloat;
    uint8 usExponent = 0;
    uint32 usMantissa = 0;
    bit_reverse_bytes((uint8*)&usReversed, 2);
    usExponent = ((usReversed & 0xE000) >> 13);
    usMantissa = (usReversed & 0x1FFF);
    return usMantissa << usExponent;
}

char SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
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

uint32 asciiToDexcomSrc(char addr[6]) {
    uint32 src = 0;
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
    printf("%lu %lu %hhu", dex_num_decoder(pPkt->raw), 2 * dex_num_decoder(pPkt->filtered), pPkt->battery);
    uartDisable();
}

void makeAllOutputs() {
    uint8 i;
    for (i=0; i < 16; i++) {
        setDigitalOutput(i, LOW);
    }
}

ISR (ST, 0) {
    IRCON &= ~0x80;
    SLEEP &= ~0x02;
    IEN0 &= ~0x20;
    WORIRQ &= ~0x11;
    WORCTRL &= ~0x03;
    if(do_close_usb && usbEnabled) {
         usbPoll();
    }
}

void uartEnable() {
    U1UCR |= 0x40; //CTS/RTS ON
    delayMs(500);
}

void uartDisable() {
    LED_GREEN(1);
    delayMs(500);
    U1UCR &= ~0x40; //CTS/RTS Off
    LED_GREEN(0);
    U1CSR &= ~0x40; // Recevier disable
}

void goToSleep (uint16 seconds) {
    unsigned char temp;

    if(!usbEnabled) {
        IEN0 |= 0x20; // Enable global ST interrupt [IEN0.STIE]
        WORIRQ |= 0x10; // enable sleep timer interrupt [EVENT0_MASK]

        /*SLEEP |= 0x02;                  // SLEEP.MODE = PM2*/
        SLEEP |= 0x01;                  // SLEEP.MODE = PM2

        if(do_close_usb)
        {
            SLEEP &= ~(1<<7);
            disableUsbPullup();
            usbDeviceState = USB_STATE_DETACHED;
        }

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
        uint32 start = getMs();
        uint32 end = getMs();
        while(((end-start)/1000)<seconds) {
            end = getMs();
            delayMs(100);
            doServices();
        }
    }
}

void putchar(char c) {
    uart1TxSendByte(c);
    if (usbEnabled)
        usbComTxSendByte(c);
}

char nibbleToAscii(uint8 nibble) {
    nibble &= 0xF;
    if (nibble <= 0x9){ return '0' + nibble; }
    else{ return 'A' + (nibble - 0xA); }
}


void toBytes(uint32 n,uint8 bytes[] ) {
    bytes[0] = (n >> 24) & 0xFF;
    bytes[1] = (n >> 16) & 0xFF;
    bytes[2] = (n >> 8) & 0xFF;
    bytes[3] = n & 0xFF;
}

void printBytes(uint8 bytes[]) {
    uint8 j;
    for(j = 0; j < 4; j++) {
        putchar(nibbleToAscii(bytes[j] >> 4));
        putchar(nibbleToAscii(bytes[j]));
        if(j<4) putchar('-');
    }
}
void swap_channel(uint8 channel, uint8 newFSCTRL0)
{
    do
    {
        RFST = 4;   //SIDLE
    } while (MARCSTATE != 0x01);

    FSCTRL0 = newFSCTRL0;
    CHANNR = channel;
    RFST = 2;   //RX
}

uint8 WaitForPacket(uint16 milliseconds, Dexcom_packet* pkt, uint8 XDATA channel) {
    uint32 XDATA start = getMs();
    uint8 XDATA * packet = 0;
    uint8 XDATA nRet = 0;
    static uint8 XDATA lastpktxid = 64;
    uint8 XDATA txid = 0;
    swap_channel(nChannels[channel], fOffset[channel]);

    while (!milliseconds || (getMs() - start) < milliseconds) {
        doServices();
        if (packet = radioQueueRxCurrentPacket()) {
            uint8 len = packet[0];

            if(radioCrcPassed()) {
                fOffset[channel] += FREQEST;
                memcpy(pkt, packet, min8(len+2, sizeof(Dexcom_packet)));

                if(pkt->src_addr == dex_tx_id || dex_tx_id == 0 || only_listen_for_my_transmitter == 0) {
                    pkt->txId -= channel;
                    txid = (pkt->txId & 0xFC) >> 2;

                    if(txid != lastpktxid) {
                        nRet = 1;
                        lastpktxid = txid;
                    }
                }
            }
            radioQueueRxDoneWithPacket();
            return nRet;
        }
    }
    return nRet;
}

uint8 get_packet(Dexcom_packet* pPkt, uint32 XDATA channel_wait) {
    uint8 XDATA nChannel = 0;
    uint32 XDATA delay = channel_wait + 100;

    if(WaitForPacket(delay, pPkt, nChannel) == 1) {
        delay_offset = 0;
        return 1;
    } else {
        uint32 search_cutoff = getMs() + wait_before_known_miss - 100;
        while(getMs() < search_cutoff) {
            nChannel = ((nChannel % 3) + 1);
            delay = 25;

            if(WaitForPacket(delay, pPkt, nChannel) == 1) {
                delay_offset = (nChannel * ((wait_before_known_miss - 100) / 3) );
                return 0;
            }
        }
        return 0;
    }
}

uint8 get_packet_fixed_channel(Dexcom_packet* pPkt, uint8 XDATA nChannel) {
    while(1) {
        switch(WaitForPacket(25, pPkt, nChannel)) {
        case 1:
            return 1;
        case 0:
            continue;
        }
    }
    return 0;
}

uint8 SetRFParam(unsigned char XDATA* addr, uint8 val) {
    *addr = val;
    return 1;
}

void doServices()
{
    if(usbEnabled) {
        boardService();
        usbComService();
    }
}

void initUart1() {
    uart1Init();
    uart1SetBaudRate(9600);
}

void configBt() {
    uartEnable();
    printf("AT+NAMEDexDrip3");
    uartDisable();
}

void rest(uint16 XDATA rest_time) {
    RFST = 4;
    delayMs(80);
    doServices();
    goToSleep(rest_time + (delay_offset / 1000));
    USBPOW = 1;
    USBCIE = 0b0111;
    radioMacInit();
    MCSM1 = 0;
    radioMacStrobe();
}

void timing_setup() {
    /*7 pass timing setup*/
    /*first pass, wait on channel 1 for packets*/
        Dexcom_packet Pkt;
        memset(&Pkt, 0, sizeof(Dexcom_packet));
        boardService();
        LED_GREEN(1);
        LED_RED(1);
        while(!get_packet_fixed_channel(&Pkt, 0)) {}

        print_packet(&Pkt);
        LED_GREEN(0);
    //sleep default, then wait on channel 1
        while (initial_wait == 0 || initial_wait > five_minutes){
            RFST = 4;
            delayMs(80);
            doServices();
            goToSleep(100);
            USBPOW = 1;
            USBCIE = 0b0111;
            radioMacInit();
            MCSM1 = 0;
            radioMacStrobe();

            memset(&Pkt, 0, sizeof(Dexcom_packet));
            boardService();

            timeInit();
            LED_GREEN(1);
            while(!get_packet_fixed_channel(&Pkt, 0)) {}
            print_packet(&Pkt);
            LED_GREEN(0);
            initial_wait = getMs();
        }
        initial_wait = 100 + (initial_wait / 1000) - 3;

    //add to default sleep, then wait on channel 1
        while (second_wait == 0 || second_wait > five_minutes) {
            rest(initial_wait);
            memset(&Pkt, 0, sizeof(Dexcom_packet));
            boardService();
            timeInit();
            LED_GREEN(1);
            while(!get_packet_fixed_channel(&Pkt, 0)) {}
            print_packet(&Pkt);
            LED_GREEN(0);
            second_wait = getMs();
        }

    //repeat but listen on channel 4
        while (first_wait_fourth_channel == 0 || first_wait_fourth_channel > five_minutes) {
            rest(initial_wait);
            memset(&Pkt, 0, sizeof(Dexcom_packet));
            boardService();
            timeInit();
            LED_GREEN(1);
            while(!get_packet_fixed_channel(&Pkt, 3)) {}
            print_packet(&Pkt);
            LED_GREEN(0);
            first_wait_fourth_channel = getMs() + 100;
        }

    //Get back into timing for channel 1
        rest(initial_wait - 10);
        memset(&Pkt, 0, sizeof(Dexcom_packet));
        boardService();
        timeInit();
        LED_GREEN(1);
        while(!get_packet_fixed_channel(&Pkt, 0)) {}
        LED_GREEN(0);

    //set resleep values and default
        LED_RED(0);
        fixed_wait_adder = (initial_wait + second_wait) / 2;
        wait_before_known_miss = first_wait_fourth_channel - fixed_wait_adder;
}

void main() {
    systemInit();
    channel_select = 1;
    channel_number = 0;

    initUart1();
    P1DIR |= 0x08; // RTS
    makeAllOutputs();

    delayMs(4000);
    configBt();
    dex_tx_id= asciiToDexcomSrc(transmitter_id);
    delayMs(4000);

    radioQueueInit();
    radioQueueAllowCrcErrors = 1;
    MCSM1 = 0;

    timing_setup();

    while(1) {
        Dexcom_packet Pkt;
        timeInit();
        memset(&Pkt, 0, sizeof(Dexcom_packet));
        boardService();
        LED_YELLOW(1);
        if(!get_packet(&Pkt, fixed_wait_adder))
            continue;

        print_packet(&Pkt);

        if(getMs() > five_minutes) {
            timing_setup();
        }
        LED_YELLOW(0);
        LED_RED(0);
        rest(initial_wait);
    }
}
