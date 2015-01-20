/** DEXDRIP Translator:
  == Description ==
  The app uses the radio_queue libray to receive packets.  It does not
  transmit any packets.

  The output from this app takes the following format:
  RAWREADING TRANSMITTERBATTERY WIXELBATTERY

  The green LED indicates that data was just sent

  PLEASE BE SURE TO SET YOUR TRANSMITTER ID BELOW

  Also, if you dont want to use usb, set it to zero and set close usb to 1

  if do_lights = 1, the lights indicate the following
     RED LIGHT = Sleeping
     GREEN LIGHT = Sending Packet
     YELLOW LIGHT = Scanning channel 1 in the main loop
     YELLOW AND RED LIGHT = Scanning channel 3 in the main loop
     NO LIGHT = Scanning for packets in order to find timings
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
static const char XDATA dexcom_transmitter_id[] = "66ENF";                                          //
static volatile BIT only_listen_for_my_transmitter = 1;                                             //
static volatile BIT do_lights = 1;                                                                  //
static volatile BIT adjust_offset = 0;                                                              //
//..................................................................................................//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Dont Change anything from here on in unless you know what your doing (Or just want to have funzies!
uint32 asciiToDexcomSrc(char *addr);
uint32 getSrcValue(char srcVal);
volatile uint32 dex_tx_id;
#define NUM_CHANNELS        (4)
static uint8 fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
static uint32 XDATA five_minutes = 300000;
// TIMING VARIABLES
uint32 initial_wait = 0;
uint32 first_wait = 0;
uint32 second_wait = 0;
uint32 third_wait = 0;
uint32 should_wait_first = 0;
uint32 should_wait_second = 0;
uint32 should_wait_third = 0;
uint32 should_wait_fourth = 0;
int32 channel_drift = 0;

// STATE VARIABLES
uint8 do_timing_setup = 1;
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

void green_on() {
    if(do_lights) { LED_GREEN(1); }
}
void green_off() {
    if(do_lights) { LED_GREEN(0); }
}
void red_on() {
    if(do_lights) { LED_RED(1); }
}
void red_off() {
    if(do_lights) { LED_RED(0); }
}
void yellow_on() {
    if(do_lights) { LED_YELLOW(1); }
}
void yellow_off() {
    if(do_lights) { LED_YELLOW(0); }
}

void uartEnable() {
    green_on();
    U1UCR |= 0x40; //CTS/RTS ON
    delayMs(2000);
}

void uartDisable() {
    delayMs(2000);
    U1UCR &= ~0x40; //CTS/RTS Off
    U1CSR &= ~0x40; // Recevier disable
    green_off();
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
    if(!usb_off) {
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
            disableUsbPullup();
            usbDeviceState = USB_STATE_DETACHED;
            SLEEP &= ~(1<<7);
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
            usbDeviceState = USB_STATE_POWERED;
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
    if(!usb_off) {
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

uint32 get_packet_fixed_channel_timed(Dexcom_packet* pkt, uint8 channel, uint32 wait_time) {
    uint32 start_time_packet = 0;
    uint8 * packet = 0;
    uint32 waited_for = 0;
    start_time_packet = getMs();
    swap_channel(nChannels[channel], fOffset[channel]);
    if(!wait_time) { wait_time = five_minutes; }

    while(getMs() - start_time_packet < wait_time) {
        if (packet = radioQueueRxCurrentPacket()) {
            uint8 len = packet[0];
            waited_for = getMs() - start_time_packet;

            memcpy(pkt, packet, min8(len+2, sizeof(Dexcom_packet)));
            pkt->txId -= channel;

            if(pkt->src_addr == dex_tx_id || dex_tx_id == 0 || only_listen_for_my_transmitter == 0) {
                if(adjust_offset) { fOffset[channel] += FREQEST; }
                radioQueueRxDoneWithPacket();
                return waited_for;
            }
            radioQueueRxDoneWithPacket();
        }
        doServices();
    }
    return 0;
}

uint32 get_packet_fixed_channel(Dexcom_packet* pPkt, uint8 nChannel) {
    return get_packet_fixed_channel_timed(pPkt, nChannel, (five_minutes + (20 * 1000)));
}

uint32 get_packet(Dexcom_packet* pPkt) {
    uint32 start_full_scan = getMs();
    yellow_on();

    if (get_packet_fixed_channel_timed(pPkt, 0, (should_wait_first - channel_drift))) {
        yellow_off();
        return start_full_scan - getMs();
    }
    if (get_packet_fixed_channel_timed(pPkt, 1, should_wait_second)) {
        yellow_off();
        return start_full_scan - getMs();
    }
    if (get_packet_fixed_channel_timed(pPkt, 2, should_wait_third)) {
        yellow_off();
        return start_full_scan - getMs();
    }
    if (get_packet_fixed_channel_timed(pPkt, 3, should_wait_fourth)) {
        yellow_off();
        return start_full_scan - getMs();
    }
    yellow_off();
    return 0;
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
    red_on();
    if(rest_time < 10) {
        rest_time = 10;
    } else if (rest_time > (5 * 60)) {
        rest_time = (5 * 60) - 30;
    }
    RFST = 4;
    delayMs(6000);
    goToSleep(rest_time);
    USBPOW = 1;
    USBCIE = 0b0111;
    radioMacInit();
    MCSM1 = 0;
    radioMacStrobe();
    red_off();
}

void main() {
    BIT entered_loop = 0;
    systemInit();

    initUart1();
    P1DIR |= 0x08; // RTS
    /*makeAllOutputs();*/

    delayMs(4000);
    configBt();
    dex_tx_id= asciiToDexcomSrc(dexcom_transmitter_id);
    delayMs(4000);

    radioQueueInit();
    MCSM1 = 0;
    do_timing_setup = 1;

    rest(10);
    while(1) {
        uint32 timer = 0;
        Dexcom_packet Pkt;
        boardClockInit();
        timeInit();

        if (do_timing_setup && entered_loop) { do_timing_setup = 1; }
        entered_loop = 1;
        // if do_timing_setup == 0, timings seem alright, just search for packets using what we know!
        // if do_timing_setup == 1, setup all our timings
        // if do_timing_setup == 2, start timings over again
        // if do_timing_setup == 3, try to reset wake time to address time drift

        if (do_timing_setup == 1) {
            rest(10);
            memset(&Pkt, 0, sizeof(Dexcom_packet));
            if (get_packet_fixed_channel(&Pkt, 0)) {
                print_packet(&Pkt);
            } else {
                do_timing_setup = 2;
            }
        }

        if (do_timing_setup == 1) {
            rest(150);
            memset(&Pkt, 0, sizeof(Dexcom_packet));
            if (timer = get_packet_fixed_channel(&Pkt, 0)) {
                initial_wait = 150 + (timer / 1000) - 25;
                print_packet(&Pkt);
            } else {
                do_timing_setup = 2;
            }
            timer = 0;
        }

    //add to default sleep, then wait on channel 1
        if (do_timing_setup == 1) {
            rest(initial_wait);
            memset(&Pkt, 0, sizeof(Dexcom_packet));

            if (timer = get_packet_fixed_channel(&Pkt, 0)) {
                first_wait = timer;
                print_packet(&Pkt);
            } else {
                do_timing_setup = 2;
            }
            timer = 0;
        }

    //add to default sleep, then wait on channel 2
        if (do_timing_setup == 1) {
            rest(initial_wait);
            memset(&Pkt, 0, sizeof(Dexcom_packet));

            if (timer = get_packet_fixed_channel(&Pkt, 1)) {
                second_wait = timer;
                print_packet(&Pkt);
            } else {
                do_timing_setup = 2;
            }
            timer = 0;
        }

    //add to default sleep, then wait on channel 3
        if (do_timing_setup == 1) {
            rest(initial_wait);
            memset(&Pkt, 0, sizeof(Dexcom_packet));

            if (timer = get_packet_fixed_channel(&Pkt, 2)) {
                third_wait = timer;
                print_packet(&Pkt);
            } else {
                do_timing_setup = 2;
            }
            timer = 0;
        }

    //add to default sleep, then waitV on channel 4
        if (do_timing_setup == 1) {
            rest(initial_wait);
            memset(&Pkt, 0, sizeof(Dexcom_packet));

            if (timer = get_packet_fixed_channel(&Pkt, 3)) {
                should_wait_first = (first_wait + second_wait) / 2;
                should_wait_second = ((second_wait + third_wait) / 2) - should_wait_first;
                should_wait_third = ((third_wait + timer) / 2) - should_wait_second;
                should_wait_fourth = 3000;
                channel_drift = 0;
                print_packet(&Pkt);
            } else {
                do_timing_setup = 2;
            }
            timer = 0;
        }

    //reset our timings to get back to channel 1
        if (do_timing_setup == 1) {
            rest(initial_wait);
            memset(&Pkt, 0, sizeof(Dexcom_packet));

            if (get_packet_fixed_channel(&Pkt, 0)) {
                print_packet(&Pkt);
                do_timing_setup = 0;
            } else {
                do_timing_setup = 2;
            }
        }

    // Alright, Heres the main loop
        while(!do_timing_setup) {
            timer = 0;
            rest(initial_wait);
            memset(&Pkt, 0, sizeof(Dexcom_packet));

            if(timer = get_packet(&Pkt)) {
                channel_drift = timer - first_wait;
                if(channel_drift < 0) { channel_drift = 0; }
                print_packet(&Pkt);
            } else {
                do_timing_setup = 2;
            }
        }
    }
}
