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
//..................................................................................................//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

extern XDATA int32 channel_number = 0;
static XDATA volatile int start_channel = 0;
extern volatile BIT channel_select = 0;
uint32 XDATA asciiToDexcomSrc(char *addr);
uint32 XDATA getSrcValue(char srcVal);
volatile XDATA uint32 dex_tx_id;
#define NUM_CHANNELS        (4)
static XDATA int8 fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static XDATA uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };

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
    U1UCR |= 0x40; //CTS/RTS ON
    delayMs(1000);
}

void uartDisable() {
    LED_GREEN(1);
    delayMs(2000);
    U1UCR &= ~0x40; //CTS/RTS Off
    LED_GREEN(0);
    U1CSR &= ~0x40; // Recevier disable
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

void doServices()
{
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
    uint8 XDATA i = 0;
    for(i = 0; i < 32; i++) {
        if (SrcNameTable[i]==srcVal) break;
    }
    return i & 0xFF;
}
void print_packet(Dexcom_packet* pPkt) {
    adcSetMillivoltCalibration(adcReadVddMillivolts());
    uartEnable();
    printf("%lu %hhu %d", dex_num_decoder(pPkt->raw), pPkt->battery, adcConvertToMillivolts(adcRead(5)));
    uartDisable();
}

void makeAllOutputs() {
    int XDATA i;
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
    if(usbPowerPresent()) {
         usbPoll();
    }
}

void goToSleep (uint16 seconds) {
    unsigned char XDATA temp;

    if(!usbPowerPresent()) {
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
        uint32 start = getMs();
        uint32 end = getMs();
        usbDeviceState = USB_STATE_POWERED;
        enableUsbPullup();
        while(((end-start)/1000)<seconds) {
            end = getMs();
            /*LED_RED( ((getMs()/1000) % 2) == 0 );*/
            delayMs(100);
            doServices();
        }
    }
}

void putchar(char c) {
    uart1TxSendByte(c);
    if (usbPowerPresent())
        usbComTxSendByte(c);
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

int WaitForPacket(uint16 milliseconds, Dexcom_packet* pkt, uint8 channel) {
    uint32 start = getMs();
    uint8 XDATA * packet = 0;
    int XDATA nRet = 0;
    static uint8 XDATA lastpktxid = 64;
    uint8 txid = 0;
    if(channel >= NUM_CHANNELS) {
        return -1;
    }
    swap_channel(nChannels[channel], fOffset[channel]);

    while (!milliseconds || (getMs() - start) < milliseconds) {
        doServices();
        if (packet = radioQueueRxCurrentPacket()) {
            uint8 XDATA len = packet[0];

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

int get_packet(Dexcom_packet* pPkt) {
    int XDATA delay = 0;
    int XDATA nChannel = 0;
    for(nChannel = start_channel; nChannel < NUM_CHANNELS; nChannel++) {
        switch(WaitForPacket(delay, pPkt, nChannel)) {
        case 1:
            return 1;
        case 0:
            continue;
        case -1:
            return 0;
        }
        delay = 500;
    }
    return 0;
}


void configBt() {
    uartEnable();
    printf("AT+NAMEDexDrip2");
    uartDisable();
}

void main() {
    uint8 XDATA ch = 0;
    uint16 XDATA cnt = 0;
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

    while(1) {
        Dexcom_packet Pkt;
        memset(&Pkt, 0, sizeof(Dexcom_packet));
        boardService();
        if(!get_packet(&Pkt))
            continue;

        print_packet(&Pkt);

        RFST = 4;
        delayMs(80);
        doServices();
        goToSleep(270);
        USBPOW = 1;
        USBCIE = 0b0111;
        radioMacInit();
        MCSM1 = 0;
        radioMacStrobe();
    }
}
