#ifndef __DEXDRIP_PACKET_H__
#define __DEXDRIP_PACKET_H__

enum dexdrip_packet_type {
    DATA_PACKET = 1,
};

/*
 * Keep this alligned and multiple of 4
 */
#define DATA_PACKET_VERSION 1

typedef struct dexdrip_data_packet {
    uint8 version;
    uint8 dexcom_battery;
    uint16 dexdrip_battery;
    uint32 raw;
} dexdrip_data_packet_t;

typedef struct dexdrip_binary_packet {
    uint8 packet_type;
    uint8 len; /* payload length */
    uint8 XDATA *payload;
} dexdrip_binary_packet_t;

#endif
