#include <radio_registers.h>
#include <cc2511_map.h>

void radioRegistersInit()
{
   

IOCFG0   =0x0E;
CHANNR   =0x00;
FSCTRL1  =0x0A;
FSCTRL0	 =0x00;
FREQ2    =0x65;
FREQ1    =0x0A;
FREQ0    =0xAA;
MDMCFG4  =0x4B;
MDMCFG3  =0x11;
MDMCFG2  =0x73;
MDMCFG1  =0x03;
MDMCFG0  =0x55;
DEVIATN  =0x00;
MCSM0    =0x14;
FOCCFG   =0x0A;

FSCAL3   =0xA9;
FSCAL2   =0x0A;
FSCAL1   =0x20;
FSCAL0   =0x0D;

TEST2    =0x81;
TEST1    =0x35;
TEST0    =0x0B;

PA_TABLE0=0x00;

SYNC1	 =0xD3;
SYNC0	 =0x91;

ADDR	 =0x00;

FREND1	 =0xB6;
FREND0	 =0x10;

BSCFG	 =0x6C;

AGCCTRL2 =0x44;

AGCCTRL1 =0x00;
//AGCCTRL1 =0x70;


AGCCTRL0 =0xB2;
//AGCCTRL0 =0x95;

PKTCTRL1 =0x04;
PKTCTRL0 =0x05;

}

BIT radioCrcPassed()
{
    return (LQI & 0x80) ? 1 : 0;
}

uint8 radioLqi()
{
    return LQI & 0x7F;
}

int8 radioRssi()
{
    return ((int8)RSSI)/2 - RSSI_OFFSET;
}
