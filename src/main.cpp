#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <tdma_protocol.h>
#define RX_LED 8
#define WIFI_CHANNEL 1
#define SYNC_SEND_INTERVAL_MS 200
#define STATS_INTERVAL_MS 5000
#define SERIAL_BAUD 921600

static const uint16_t TDMA_SLOT_US = 1200;
static const uint16_t TDMA_GUARD_US = 200;

static const uint8_t broadcast_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static uint32_t last_sync_send = 0;
static uint32_t last_stats_time = 0;
static uint32_t sync_counter = 0;
static bool blink_led = false;

static uint16_t last_packet_seq[TDMA_MAX_NODES] = {0};
static uint32_t packet_received_count[TDMA_MAX_NODES] = {0};
static uint32_t sample_received_count[TDMA_MAX_NODES] = {0};
static uint32_t packet_lost_count[TDMA_MAX_NODES] = {0};
static uint32_t invalid_packet_count[TDMA_MAX_NODES] = {0};
static uint32_t sync_sent_count = 0;

void printBaseInfo() {
    Serial.println("--- ESP32 C3 Base Station ESP-NOW TDMA ---");
    Serial.print("MAC Address de Base Station: ");
    Serial.println(WiFi.macAddress());
    Serial.print("Canal ESP-NOW: ");
    Serial.println(WIFI_CHANNEL);
    Serial.print("Ciclos TDMA: ");
    Serial.print(TDMA_MAX_NODES);
    Serial.println(" nodos\n");
    Serial.println("Formato de datos esperados: PKT_DATA con muestras agrupadas");
    Serial.println("-------------------------------------------");
}

void printStats() {
    Serial.println("STATS_BEGIN");
    Serial.print("Sync enviados: ");
    Serial.println(sync_sent_count);
    for (uint8_t i = 0; i < TDMA_MAX_NODES; i++) {
        Serial.print("Nodo ");
        Serial.print(i + 1);
        Serial.print(" - Paquetes: ");
        Serial.print(packet_received_count[i]);
        Serial.print(" | Muestras: ");
        Serial.print(sample_received_count[i]);
        Serial.print(" | Perdidos: ");
        Serial.print(packet_lost_count[i]);
        Serial.print(" | Errores: ");
        Serial.println(invalid_packet_count[i]);
    }
    Serial.println("STATS_END");
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.print("Error enviando SYNC: ");
        Serial.println(status);
    }
}

static bool addBroadcastPeer() {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcast_mac, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    return esp_now_add_peer(&peerInfo) == ESP_OK;
}

static void sendSync() {
    SyncPacket packet = {};
    packet.type = PKT_SYNC;
    packet.version = TDMA_SYNC_VERSION;
    packet.node_count = TDMA_MAX_NODES;
    packet.slot_us = TDMA_SLOT_US;
    packet.guard_us = TDMA_GUARD_US;
    packet.cycle_us = packet.slot_us * packet.node_count;
    packet.epoch_us = micros();
    packet.sync_id = sync_counter++;

    esp_err_t result = esp_now_send(broadcast_mac, reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
    if (result == ESP_OK) {
        sync_sent_count++;
    } else {
        Serial.print("Error enviando SYNC: ");
        Serial.println(result);
    }
}

static void handleDataPacket(const DataPacket &packet, uint32_t rx_us) {
    if (packet.sender_id < 1 || packet.sender_id > TDMA_MAX_NODES) {
        return;
    }
    uint8_t idx = packet.sender_id - 1;
    if (packet.sample_count == 0 || packet.sample_count > TDMA_MAX_SAMPLES_PER_PACKET) {
        invalid_packet_count[idx]++;
        return;
    }

    packet_received_count[idx]++;
    sample_received_count[idx] += packet.sample_count;

    if (last_packet_seq[idx] != 0) {
        uint16_t delta = static_cast<uint16_t>(packet.packet_seq - last_packet_seq[idx]);
        if (delta > 1) {
            uint32_t missed = static_cast<uint32_t>(delta - 1);
            packet_lost_count[idx] += missed;
            Serial.print("LOSS,ID=");
            Serial.print(packet.sender_id);
            Serial.print(",MISSED=");
            Serial.print(missed);
            Serial.print(",PREV_SEQ=");
            Serial.print(last_packet_seq[idx]);
            Serial.print(",CURR_SEQ=");
            Serial.print(packet.packet_seq);
            Serial.print(",RX_US=");
            Serial.println(rx_us);
        }
    }
    last_packet_seq[idx] = packet.packet_seq;

    Serial.print("DATA,ID=");
    Serial.print(packet.sender_id);
    Serial.print(",SEQ=");
    Serial.print(packet.packet_seq);
    Serial.print(",SAMPLES=");
    Serial.print(packet.sample_count);
    Serial.print(",BASE_US=");
    Serial.print(packet.base_ts_us);
    Serial.print(",RX_US=");
    Serial.print(rx_us);
    Serial.print(",VALUES=");
    for (uint8_t i = 0; i < packet.sample_count; i++) {
        if (i > 0) Serial.print(",");
        Serial.print(packet.samples[i]);
    }
    Serial.println();

    blink_led = true;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len < 1) {
        return;
    }
    uint8_t packet_type = incomingData[0];
    if (packet_type != PKT_DATA) {
        return;
    }

    if (len < TDMA_DATA_HEADER_SIZE + sizeof(uint16_t)) {
        return;
    }

    DataPacket packet = {};
    memcpy(&packet, incomingData, min(len, (int)sizeof(packet)));
    size_t expected_len = TDMA_DATA_HEADER_SIZE + packet.sample_count * sizeof(uint16_t);
    if (packet.sample_count == 0 || packet.sample_count > TDMA_MAX_SAMPLES_PER_PACKET || len != expected_len) {
        if (packet.sender_id >= 1 && packet.sender_id <= TDMA_MAX_NODES) {
            invalid_packet_count[packet.sender_id - 1]++;
        }
        return;
    }

    handleDataPacket(packet, micros());
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);

    pinMode(RX_LED, OUTPUT);
    digitalWrite(RX_LED, LOW);

    WiFi.mode(WIFI_STA);
    delay(100);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error inicializando ESP-NOW");
        while (true) {
            delay(1000);
        }
    }

    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(onDataSent);

    if (!addBroadcastPeer()) {
        Serial.println("Error agregando peer broadcast");
    }

    printBaseInfo();
    last_stats_time = millis();
    last_sync_send = millis();
    sendSync();
}

void loop() {
    if (blink_led) {
        digitalWrite(RX_LED, HIGH);
        delay(50);
        digitalWrite(RX_LED, LOW);
        blink_led = false;
    }

    uint32_t now = millis();
    if ((int32_t)(now - last_sync_send) >= 0) {
        if (now - last_sync_send >= SYNC_SEND_INTERVAL_MS) {
            sendSync();
            last_sync_send = now;
        }
    }
    // Stats deshabilitados para reducir output en pantalla
    // if ((int32_t)(now - last_stats_time) >= 0) {
    //     if (now - last_stats_time >= STATS_INTERVAL_MS) {
    //         printStats();
    //         last_stats_time = now;
    //     }
    // }
}

