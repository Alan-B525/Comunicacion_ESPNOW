#pragma once

#include <stdint.h>

#define TDMA_MAX_NODES 10
#define TDMA_MAX_SAMPLES_PER_PACKET 8
#define TDMA_SYNC_VERSION 1
#define TDMA_DATA_HEADER_SIZE 10

enum PacketType : uint8_t {
    PKT_SYNC = 1,
    PKT_DATA = 2,
    PKT_ACK = 3
};

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t version;
    uint8_t node_count;
    uint8_t reserved;
    uint16_t cycle_us;
    uint16_t slot_us;
    uint16_t guard_us;
    uint32_t epoch_us;
    uint32_t sync_id;
} SyncPacket;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t sender_id;
    uint16_t packet_seq;
    uint32_t base_ts_us;
    uint8_t sample_count;
    uint8_t reserved;
    uint16_t samples[TDMA_MAX_SAMPLES_PER_PACKET];
} DataPacket;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t sender_id;
    uint16_t packet_seq;
    uint32_t receive_ts_us;
} AckPacket;
