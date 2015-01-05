/** DEXDRIP Translator:
  == Description ==
  The app uses the radio_queue libray to receive packets.  It does not
  transmit any packets.

  The output from this app takes the following format:
  RAWREADING TRANSMITTERBATTERY WIXELBATTERY

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
#include <adc.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//..................SET THESE VARIABLES TO MEET YOUR NEEDS..........................................//
//                           0 = false, 1 = true                                                    //
static const char XDATA dexcom_transmitter_id[] = "ABCDE";                                          //
static volatile BIT only_listen_for_my_transmitter = 1;                                             //
//..................................................................................................//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

uint32 asciiToDexcomSrc(char *addr);
uint32 getSrcValue(char srcVal);
volatile uint32 dex_tx_id;
#define NUM_CHANNELS        (4)
static uint8 XDATA fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static uint8 XDATA nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
static uint32 XDATA five_minutes = (60000 * 5);
uint32 XDATA initial_wait = 0;
uint32 XDATA second_wait = 0;
uint32 XDATA wait_fourth_channel = 0;
uint8 do_timing_setup = 1;
uint8 been_there_done_that = 0;
uint16 XDATA fixed_wait_adder;
uint32 start_time_main;
uint32 channel_drift = 0;
BIT usb_off = 0;

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
    delayMs(3000);
}

void uartDisable() {
    LED_GREEN(1);
    delayMs(3000);
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
    SLEEP &= ~0x01;                  // SLEEP.MODE = PM1
    /*SLEEP &= ~0x02;*/
    IEN0 &= ~0x20;
    WORIRQ &= ~0x11;
    WORCTRL &= ~0x03;
    if(usbPowerPresent()) {
         usbPoll();
    }
}

void doServices() {
    if(usbPowerPresent()) {
        boardService();
        usbComService();
    }
}

void goToSleep (uint16 seconds) {
    unsigned char XDATA temp;

    if(!usbPowerPresent()) {
        IEN0 |= 0x20; // Enable global ST interrupt [IEN0.STIE]
        WORIRQ |= 0x10; // enable sleep timer interrupt [EVENT0_MASK]

        if(!usb_off) {
            SLEEP &= ~(1<<7);
            disableUsbPullup();
            usbDeviceState = USB_STATE_DETACHED;
            do_timing_setup = 2;
            usb_off = 1;
        }

        /*SLEEP |= 0x02;                  // SLEEP.MODE = PM2*/
        SLEEP |= 0x01;                  // SLEEP.MODE = PM1


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
        uint32 start_sleep = getMs();
        uint32 end_sleep = getMs();
        if(usb_off) {
            enableUsbPullup();
            usbDeviceState = USB_STATE_ATTACHED;
            SLEEP |= (1<<7);
            do_timing_setup = 2;
            usb_off = 0;
        }

        while(((end_sleep - start_sleep) / 1000) < seconds) {
            end_sleep = getMs();
            delayMs(100);
            doServices();
        }
    }
}

void putchar(char c) {
    uart1TxSendByte(c);
    if(usbPowerPresent()) {
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

uint8 WaitForPacket(Dexcom_packet* pkt, uint8 channel) {
    uint8 XDATA * packet = 0;
    uint8 XDATA txid = 0;
    uint32 XDATA start_scan = getMs();

    while ((getMs() - start_scan) < 20) {
        doServices();
        if (packet = radioQueueRxCurrentPacket()) {
            uint8 XDATA len = packet[0];

            fOffset[channel] += FREQEST;
            memcpy(pkt, packet, min8(len+2, sizeof(Dexcom_packet)));

            if(pkt->src_addr == dex_tx_id || dex_tx_id == 0 || only_listen_for_my_transmitter == 0) {
                pkt->txId -= channel;
                txid = (pkt->txId & 0xFC) >> 2;

                radioQueueRxDoneWithPacket();
                return 1;
            }
        radioQueueRxDoneWithPacket();
        }
    }
    return 0;
}

uint8 get_packet_fixed_channel(Dexcom_packet* pPkt, uint8 XDATA nChannel) {
    swap_channel(nChannels[nChannel], fOffset[nChannel]);
    while(1) {
        if(WaitForPacket(pPkt, nChannel)) {
            return 1;
        }
    }
}

uint8 get_packet_fixed_channel_timed(Dexcom_packet* pPkt, uint8 XDATA nChannel) {
    if(WaitForPacket(pPkt, nChannel)) {
        return 1;
    }
    return 0;
}

uint8 get_packet(Dexcom_packet* pPkt) {
    uint32 start_time_packet = getMs();
    swap_channel(nChannels[0], fOffset[0]);
    while(getMs() - start_time_packet < fixed_wait_adder + 1400) {
        if(get_packet_fixed_channel_timed(pPkt, 0)) {
            return 1;
        }
    }
    start_time_packet = getMs();
    swap_channel(nChannels[3], fOffset[3]);
    while(getMs() - start_time_packet < 7000) {
        if(get_packet_fixed_channel_timed(pPkt, 3)) {
            channel_drift = 1;
            swap_channel(nChannels[0], fOffset[0]);
            return 1;
        }
    }
    swap_channel(nChannels[0], fOffset[0]);
    channel_drift = 1;
    return 1;
}


uint8 SetRFParam(unsigned char XDATA* addr, uint8 val) {
    *addr = val;
    return 1;
}

void configBt() {
    uartEnable();
    printf("AT+NAMEDexTEST");
    uartDisable();
}

void rest(uint32 rest_time) {
    LED_RED(1);
    if(rest_time < 10) {
        rest_time = 10;
    }
    RFST = 4;
    delayMs(4000);
    doServices();
    goToSleep(rest_time);
    USBPOW = 1;
    USBCIE = 0b0111;
    radioMacInit();
    MCSM1 = 0;
    radioMacStrobe();
    LED_RED(0);
}

void main() {
    systemInit();

    initUart1();
    P1DIR |= 0x08; // RTS
    makeAllOutputs();

    delayMs(4000);
    configBt();
    dex_tx_id= asciiToDexcomSrc(dexcom_transmitter_id);
    delayMs(4000);

    radioQueueInit();
    MCSM1 = 0;
    do_timing_setup = 1;

    while(1) {
        uint32 start_time = 0;
        Dexcom_packet Pkt;
        timeInit();
        // if do_timing_setup == 0, timings seem alright, just search for packets using what we know!
        // if do_timing_setup == 1, setup all our timings
        // if do_timing_setup == 2, start timings over again
        // if do_timing_setup == 3, try to reset wake time to address time drift

        if (do_timing_setup == 2 || do_timing_setup == 3) {
            do_timing_setup = 1;
        }

        if (do_timing_setup == 1) {
            rest(10);
            memset(&Pkt, 0, sizeof(Dexcom_packet));
            doServices();
            get_packet_fixed_channel(&Pkt, 0);

            print_packet(&Pkt);
        }

        if (do_timing_setup == 1) {
            rest(100);
            memset(&Pkt, 0, sizeof(Dexcom_packet));
            doServices();
            timeInit();
            start_time = getMs();
            get_packet_fixed_channel(&Pkt, 0);

            initial_wait = getMs() - start_time;
            print_packet(&Pkt);

            if(initial_wait > five_minutes) {
                do_timing_setup = 2;
            }
            initial_wait = 100 + (initial_wait / 1000) - 30;
        }

    //add to default sleep, then wait on channel 1
        if (do_timing_setup == 1) {
            rest(initial_wait);
            memset(&Pkt, 0, sizeof(Dexcom_packet));
            doServices();
            timeInit();
            start_time = getMs();
            get_packet_fixed_channel(&Pkt, 0);

            second_wait = getMs() - start_time;
            print_packet(&Pkt);

            if(second_wait > five_minutes) {
                do_timing_setup = 2;
            } else {
                do_timing_setup = 0;
                fixed_wait_adder = second_wait;
            }
        }

    // Alright, Heres the main loop
        while(do_timing_setup == 0) {
            rest(initial_wait);
            memset(&Pkt, 0, sizeof(Dexcom_packet));
            doServices();
            timeInit();
            start_time = getMs();
            if(get_packet(&Pkt)) {
                print_packet(&Pkt);
            }

            if(channel_drift == 1) {
                do_timing_setup = 3;
                been_there_done_that++;
                channel_drift = 0;
            }
            if(getMs() - start_time > five_minutes) {
                do_timing_setup = 3;
                been_there_done_that++;
            } else {
                been_there_done_that = 0;
            }
        }

        if (do_timing_setup == 3 && been_there_done_that == 0) {
            rest(100);
            memset(&Pkt, 0, sizeof(Dexcom_packet));
            doServices();
            timeInit();
            start_time = getMs();
            get_packet_fixed_channel(&Pkt, 0);
            print_packet(&Pkt);
            do_timing_setup = 0;
        }

        if (do_timing_setup == 3 && been_there_done_that > 1) {
            been_there_done_that = 0;
            do_timing_setup = 1;
        }
    }
}
