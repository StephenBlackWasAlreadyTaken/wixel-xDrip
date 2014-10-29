/** DEXDRIP Translator:
  == Description ==
  The app uses the radio_queue libray to receive packets.  It does not
  transmit any packets.

  The output from this app takes the following format:

  The red LED indicates activity on the radio channel (packets being received).
  Since every radio packet has a chance of being lost, there is no guarantee
  that this app will pick up all the packets being sent, and some of
  what it does pick up will be corrupted (indicated by a failed CRC check).

  == Parameters ==
radio_channel: See description in radio_link.h.
*/


/** Dependencies **************************************************************/
#include <wixel.h>
#include <usb.h>
#include <usb_com.h>
#include <radio_registers.h>
#include <radio_queue.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <uart1.h>
#include <gpio.h>

static volatile BIT do_sleep = 0;
static volatile BIT is_sleeping = 0;
static volatile BIT do_verbose = 1;
static volatile BIT do_binary = 1;
static volatile BIT usb_on = 0;
static volatile BIT bt_on = 1;
static volatile int watched_channel = 0;
static volatile BIT do_close_usb = 0;

// forward prototypes
int doServices(uint8 bWithProtocol);
#define USB_COMMAND_MAXLEN  (32)
#define NUM_CHANNELS        (4)

// frequency offsets for each channel - seed to 0.
static uint8 fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };

// store RF config in FLASH, and retrieve it from here to put it in proper location (also incidentally in flash).
// this allows persistent storage of RF params that will survive a restart of the wixel (although not a reload of wixel app obviously).
// TO-DO get this working with DMA - need to erase memory block first, then write this to it.

uint8 XDATA RF_Params[50];
uint16 getRFParamOffset(unsigned char XDATA* pAddr)
{
    unsigned char XDATA* p = 0;
    return pAddr - p;
}

uint8 checkRFParamAddr(uint16 addr)
{
    if(addr >= 0xDF20 && addr <= 0xDF22)
        return 0;
    if(addr >= 0xDF27 && addr <= 0xDF2D)
        return 0;
    if(addr >= 0xDF00 && addr <= 0xDF31)
        return 1;
    return 0;
}

void StoreRFParam(uint16 addr, uint8 val)
{
    if(checkRFParamAddr(addr))
        RF_Params[addr - 0xDF00] = val;
}

uint8 SetRFParam(uint16 addr, uint8 val)
{
    if(checkRFParamAddr(addr))
    {
        char XDATA *p = 0; p+=addr;
        *p = val;
        StoreRFParam(addr, val);
        return 1;
    }
    return 0;
}



uint8 GetRFParam(uint16 addr)
{
    char XDATA *p = 0; p+=addr;
    return *p;
}

void LoadRFParam(unsigned char XDATA* addr, uint8 default_val)
{
    uint16 addr_offset = getRFParamOffset(addr);
    if(checkRFParamAddr(addr_offset))
    {
        uint8 val = default_val;
        if(RF_Params[49] == 1)
            val = RF_Params[addr_offset - 0xDF00];

        SetRFParam(addr_offset, val);
    }
}

void dex_RadioSettings()
{
    // Transmit power: one of the highest settings, but not the highest.
    LoadRFParam(&PA_TABLE0, 0x00);
    LoadRFParam(&IOCFG0, 0x0E);
    LoadRFParam(&FREQ2, 0x65);
    LoadRFParam(&FREQ1, 0x0A);
    LoadRFParam(&FREQ0, 0xAA);
    LoadRFParam(&SYNC1, 0xD3);
    LoadRFParam(&SYNC0, 0x91);
    LoadRFParam(&ADDR, 0x00);
    LoadRFParam(&FSCTRL1, 0x0A);
    LoadRFParam(&FSCTRL0, 0x00);
    LoadRFParam(&MDMCFG4, 0x4B);
    LoadRFParam(&MDMCFG3, 0x11);
    LoadRFParam(&MDMCFG2, 0x73);
    LoadRFParam(&MDMCFG1, 0x03);
    LoadRFParam(&MDMCFG0, 0x55);
    LoadRFParam(&DEVIATN, 0x00);
    LoadRFParam(&FREND1, 0xB6);
    LoadRFParam(&FREND0, 0x10);
    LoadRFParam(&FOCCFG, 0x0A);
    LoadRFParam(&BSCFG, 0x6C);
    LoadRFParam(&AGCCTRL2, 0x44);
    LoadRFParam(&AGCCTRL1, 0x00);
    LoadRFParam(&AGCCTRL0, 0xB2);
    LoadRFParam(&FSCAL3, 0xA9);
    LoadRFParam(&FSCAL2, 0x0A);
    LoadRFParam(&FSCAL1, 0x20);
    LoadRFParam(&FSCAL0, 0x0D);
    LoadRFParam(&TEST2, 0x81);
    LoadRFParam(&TEST1, 0x35);
    LoadRFParam(&TEST0, 0x0B);
    LoadRFParam(&PKTCTRL1, 0x04);
    LoadRFParam(&PKTCTRL0, 0x05);
    RF_Params[49] = 1;
}

uint8 min8(uint8 a, uint8 b)
{
    if(a < b) return a;
    return b;
}

typedef struct _Dexcom_packet
{
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

void openUart() {
    uart1Init();
    uart1SetBaudRate(9600);
}

void uartEnable() {
    U1CSR |= 0x40; // Recevier enable
    P2 |= 0x02;
    P1 &= ~0x08;
}

void uartDisable() {
    while(uartTxPendingBytes()!=0)  { }
    P2 |= 0x02;
    P1 |= 0x08;
    U1CSR &= ~0x40; // Recevier disable
}

void configBt() {
    printf("AT+NAMEDexDrip");
}
/** Functions *****************************************************************/
/* the function that puts the system to sleep (PM2) and configures sleep timer to
   wake it again in 250 seconds.*/

void makeAllOutputs(BIT value)
{
    int i = 0;
    for (;i < 16; i++)
    {
        setDigitalOutput(i, value);
    }
}

// use power mode 1
#define SLEEP_MODE_USING (0x01)

// ISR for catching Sleep Timer interrupts
ISR (ST, 0) {
    IRCON &= ~0x80;     // clear IRCON.STIF
    SLEEP &= ~SLEEP_MODE_USING;     // clear SLEEP.MODE
    IEN0 &= ~0x20;      // clear IEN0.STIE
    WORIRQ &= ~0x11;    // clear Sleep timer EVENT0_MASK and EVENT0_FLAG
    WORCTRL &= ~0x03;   // Set timer resolution back to 1 period.

    if (usb_on && do_close_usb) {
        usbPoll();
    }
    is_sleeping = 0; 
}

void goToSleep (uint16 seconds) {
    unsigned char temp;
    // The wixel docs note that any input pins consume ~30uA
    IEN0 |= 0x20; // Enable global ST interrupt [IEN0.STIE]
    WORIRQ |= 0x10; // enable sleep timer interrupt [EVENT0_MASK]

    /* the sleep mode i've chosen is PM2. According to the CC251132 datasheet,
       typical power consumption from the SoC should be around 0.5uA */

    /*The SLEEP.MODE will be cleared to 00 by HW when power
      mode is entered, thus interrupts are enabled during power modes.
      All interrupts not to be used to wake up from power modes must
      be disabled before setting SLEEP.MODE!=00.*/

    // sleep power mode 2 is incompatible with USB - as USB registers lose state in this mode.

    SLEEP |= SLEEP_MODE_USING; // SLEEP.MODE = PM2

    if(do_close_usb)
    {
        // disable the USB module
        SLEEP &= ~(1<<7); // Disable the USB module (SLEEP.USB_EN = 0).
        disableUsbPullup();
        usbDeviceState = USB_STATE_DETACHED;
    }

    WORCTRL |= 0x04; // Reset
     /*Wait for 2x+ve edge on 32kHz clock*/
    temp = WORTIME0;
    while (temp == WORTIME0) {};
    temp = WORTIME0;
    while (temp == WORTIME0) {};
    WORCTRL |= 0x03; // 2^15 periods
    WOREVT1 = (seconds >> 8);
    WOREVT0 = (seconds & 0xff); //300=293 s
    PCON |= 0x01; // PCON.IDLE = 1;
    is_sleeping = 1;
}

void updateLeds()
{
    if (do_sleep)
    {
        if(is_sleeping)
        {
            LED_YELLOW((getMs()&0x00000F00) == 0x100);
        }
        else
        {
            LED_GREEN((getMs()&0x00000380) == 0x80);
        }
    }
}

// This is called by printf and printPacket.
void putchar(char c)
{
    usbComTxSendByte(c);
}

char nibbleToAscii(uint8 nibble)
{
    nibble &= 0xF;
    if (nibble <= 0x9){ return '0' + nibble; }
    else{ return 'A' + (nibble - 0xA); }
}

uint8 bit_reverse_byte(uint8 in)
{
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

void bit_reverse_bytes(uint8* buf, uint8 nLen)
{
    uint8 i = 0;
    for(; i < nLen; i++)
    {
        buf[i] = bit_reverse_byte(buf[i]);
    }
}

uint32 dex_num_decoder(uint16 usShortFloat)
{
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

void dexcom_src_to_ascii(uint32 src, char addr[6])
{
    addr[0] = SrcNameTable[(src >> 20) & 0x1F];
    addr[1] = SrcNameTable[(src >> 15) & 0x1F];
    addr[2] = SrcNameTable[(src >> 10) & 0x1F];
    addr[3] = SrcNameTable[(src >> 5) & 0x1F];
    addr[4] = SrcNameTable[(src >> 0) & 0x1F];
    addr[5] = 0;
}

typedef struct _RawRecord
{
    uint8   size;
    uint32  tickcount;
    uint8   src_addr[6];
    uint16  raw;
    uint16  filtered;
    uint8   battery;
    int8    RSSI;
    uint8   txid;
} RawRecord;

void print_packet(Dexcom_packet* pPkt)
{
    uint8 txid = (pPkt->txId & 0xFC) >> 2;
    XDATA RawRecord Record;
    Record.size = sizeof(Record);
    Record.tickcount = getMs();
    dexcom_src_to_ascii(pPkt->src_addr, Record.src_addr);
    Record.raw = dex_num_decoder(pPkt->raw);
    Record.filtered = 2 * dex_num_decoder(pPkt->filtered);
    Record.battery = pPkt->battery;
    Record.RSSI = getPacketRSSI(pPkt);
    Record.txid = txid;

    LED_YELLOW(1);
    LED_GREEN(1);

    if(usb_on) {
        while(usbComTxAvailable() < sizeof(Record))
            doServices(0);
        usbComTxSend((const uint8 XDATA*)&Record, sizeof(Record));
    }

    if(bt_on) }
        uartEnable();
        while(uart1TxAvailable() < sizeof(Record))
            delayMs(20);
        uart1TxSend((const uint8 XDATA*)&Record, sizeof(Record));
        uartDisable();
    }

    LED_YELLOW{(0);
    LED_GREEN(0);
}

typedef struct _usb_command
{
    uint8 usbCommandBuffer[USB_COMMAND_MAXLEN];
    uint8 nCurReadPos;
} t_usb_command;

uint8 init_usb_command(t_usb_command* pCmd)
{
    if(!pCmd)
        return 0;
    memset(pCmd->usbCommandBuffer, 0, USB_COMMAND_MAXLEN);
    pCmd->nCurReadPos = 0;
    return 0;
}

static t_usb_command usb_command;

int usb_command_is(char* command)
{
    uint8 len = strlen(command);
    if(len != usb_command.nCurReadPos)
        return 0;
    return memcmp(command, usb_command.usbCommandBuffer, len)==0;
}

uint8 Hex1ToUint4(char c)
{
    if(c >= '0' && c <= '9')
        return c-'0';
    if(c >= 'A' && c <= 'F')
        return c + 10 -'A';
    if(c >= 'a' && c <= 'f')
        return c + 10 -'a';
    return 0;
}

uint8 Hex2ToUint8(char* c)
{
    uint8 r = 0;
    r += Hex1ToUint4(c[0]); r <<=4;
    r += Hex1ToUint4(c[1]);
    return r;
}

uint16 Hex4ToUint16(char* c)
{
    uint16 r = 0;
    r += Hex1ToUint4(c[0]); r <<=4;
    r += Hex1ToUint4(c[1]); r <<=4;
    r += Hex1ToUint4(c[2]); r <<=4;
    r += Hex1ToUint4(c[3]);
    return r;
}

int doServices(uint8 bWithProtocol)
{
    boardService();
    updateLeds();
    usbComService();
    if(bWithProtocol)
        return usbControlProtocolService();
    return 1;
}

void swap_channel(uint8 channel, uint8 newFSCTRL0)
{
    do
    {
        RFST = 4;   //SIDLE
    } while (MARCSTATE != 0x01);
    // update this, since offset can change based on channel
    FSCTRL0 = newFSCTRL0;
    CHANNR = channel;
    RFST = 2;   //RX
}

// channel is the channel index = 0...3
int WaitForPacket(uint16 milliseconds, Dexcom_packet* pkt, uint8 channel)
{
    uint32 start = getMs();
    uint8 XDATA * packet = 0;
    int nRet = 0;
    if(channel >= NUM_CHANNELS)
        return -1;
    swap_channel(nChannels[channel], fOffset[channel]);
    while (!milliseconds || (getMs() - start) < milliseconds)
    {
        if (packet = radioQueueRxCurrentPacket())
        {
            uint8 len = packet[0];
            if(radioCrcPassed())
            {
                fOffset[channel] += FREQEST;
                // there's a packet!
                memcpy(pkt, packet, min8(len+2, sizeof(Dexcom_packet)));
                nRet = 1;
                pkt->txId -= channel;
            }
            else
            {
                if(usb_on) {
                    printf("[%lu] CRC failure channel %d(%d) RSSI %d %hhu bytes received\r\n", getMs(), channel, (int)CHANNR, (int)((int8)(RSSI))/2 - 73, len);
                }
            }
            radioQueueRxDoneWithPacket();
            return nRet;
        }
    }
    if(usb_on) {
        printf("[%lu] timed out waiting for packet on channel %d(%d)\r\n", getMs(), channel, (int)CHANNR);
        printf("If you see this several times, try a different channel\r\n");
    }
    return nRet;
}

int get_packet(Dexcom_packet* pPkt)
{
    int delay = 0;
    int retries_before_reporting = 0;
    for(retries_before_reporting; retries_before_reporting < 3; retries_before_reporting++)
    {
        switch(WaitForPacket(delay, pPkt, watched_channel))
        {
        case 1:                             // got a packet that passed CRC
            return 1;
        case 0:                             // timed out
            break;
        }
        // ok, no packet this time, set new delay and try to next channel
        delay = 600;
    }
    return 0;
}

void LineStateChangeCallback(uint8 state)
{
    LED_YELLOW(state & ACM_CONTROL_LINE_DTR);
}

extern void basicUsbInit();

void main()
{
    systemInit();
    delayMs(4000);

    if (usb_on) {
        usbInit();
        usbComRequestLineStateChangeNotification(LineStateChangeCallback);
        init_usb_command(&usb_command);
    }

    if (bt_on) {
        openUart();
        configBt();
    }

    setRadioRegistersInitFunc(dex_RadioSettings);
    radioQueueInit();
    radioQueueAllowCrcErrors = 1;


    if (do_sleep) {
        makeAllOutputs(LOW);
    }
    // these are reset in radioQueueInit and radioMacInit after our init func was already called

    MCSM1 = 0;          // after RX go to idle, we don't transmit
    while (1)
    {
        Dexcom_packet Pkt;
        memset(&Pkt, 0, sizeof(Dexcom_packet));
        if(!get_packet(&Pkt))
            continue;
        // ok, we got a packet
        print_packet(&Pkt);
        // can't safely sleep if we didn't get a packet!
        if (do_sleep)
        {
            uint8 savedPICTL = PICTL;
            BIT savedP0IE = P0IE;
            RFST = 4;   //SIDLE
            LED_RED(1);
            LED_YELLOW(1);
            LED_GREEN(1);
            delayMs(80);
            doServices(1);
            LED_RED(0);
            LED_YELLOW(0);
            LED_GREEN(0);
            goToSleep(280);   //~295 s

            PICTL = savedPICTL;
            P0IE = savedP0IE;
            // Enable suspend detection and disable any other weird features.
            USBPOW = 1;
            // Enable the USB common interrupts we care about: Reset, Resume, Suspend.
            // Without this, we USBCIF.SUSPENDIF will not get set (the datasheet is incomplete).
            USBCIE = 0b0111;
            // bootstrap radio again
            radioMacInit();
            MCSM1 = 0;
            radioMacStrobe();
        }
    }
}
