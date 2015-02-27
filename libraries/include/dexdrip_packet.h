#ifndef __DEXDRIP_PACKET_H__
#define __DEXDRIP_PACKET_H__

enum dexdrip_packet_type {
    DATA_PACKET = 1,
};

typedef struct dexdrip_data_packet {
    uint32 raw;
    int16  dexdrip_battery;
    uint8  dexcom_battery;
    uint8  padding[1];
} dexdrip_data_packet_t;

typedef struct dexdrip_binary_packet {
    uint8 packet_type;
    uint8 len; /* payload length */
    uint8 XDATA *payload;
} dexdrip_binary_packet_t;

#endif
