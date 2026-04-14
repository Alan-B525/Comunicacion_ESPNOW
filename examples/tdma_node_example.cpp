#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "../include/tdma_protocol.h"

#define WIFI_CHANNEL 1
#define NODE_ID 1 // Cambia a 1, 2 o 3 para cada nodo
#define BASESTATION_MAC {0xb8, 0xf8, 0x62, 0x04, 0x6d, 0xdc} // Reemplaza con la MAC de la basestation

static uint8_t basestation_mac[6] = BASESTATION_MAC;
static uint32_t tdma_epoch_us = 0;
static uint16_t tdma_cycle_us = 0;
static uint16_t tdma_slot_us = 0;
static uint16_t tdma_guard_us = 0;
static bool tdma_ready = false;
static uint32_t next_sample_us = 0;
static uint16_t sample_buffer[TDMA_MAX_SAMPLES_PER_PACKET] = {0};
static uint8_t buffered_samples = 0;
static uint16_t packet_seq = 1;
static uint32_t last_sent_cycle = UINT32_MAX;

static inline uint16_t readSensorValue() {
    int raw = analogRead(A0);
    if (raw < 0) raw = 0;
    if (raw > 4095) raw = 4095;
    return (uint16_t)((raw * 10000) / 4095);
}

static bool isMySlot(uint32_t now_us) {
    if (!tdma_ready || tdma_cycle_us == 0) {
        return false;
    }
    uint32_t offset = ((uint32_t)(NODE_ID - 1) * tdma_slot_us);
    uint32_t phase = (now_us - tdma_epoch_us) % tdma_cycle_us;
    uint32_t slot_start = offset + tdma_guard_us / 2;
    uint32_t slot_end = offset + tdma_slot_us - tdma_guard_us / 2;
    return (phase >= slot_start) && (phase < slot_end);
}

static uint32_t currentSlotCycle(uint32_t now_us) {
    if (!tdma_ready || tdma_cycle_us == 0) {
        return UINT32_MAX;
    }
    return (now_us - tdma_epoch_us) / tdma_cycle_us;
}

static void sendDataPacket() {
    if (buffered_samples == 0) {
        return;
    }

    DataPacket packet;
    memset(&packet, 0, sizeof(packet));
    packet.type = PKT_DATA;
    packet.sender_id = NODE_ID;
    packet.packet_seq = packet_seq++;
    packet.base_ts_us = micros();
    packet.sample_count = buffered_samples;
    for (uint8_t i = 0; i < buffered_samples; i++) {
        packet.samples[i] = sample_buffer[i];
    }

    size_t length = TDMA_DATA_HEADER_SIZE + buffered_samples * sizeof(uint16_t);
    esp_err_t result = esp_now_send(basestation_mac, reinterpret_cast<uint8_t*>(&packet), length);
    if (result == ESP_OK) {
        Serial.print("NODE: enviado paquete ");
        Serial.print(packet.packet_seq);
        Serial.print(" con ");
        Serial.print(buffered_samples);
        Serial.println(" muestras");
        buffered_samples = 0;
    } else {
        Serial.print("NODE: error al enviar paquete ");
        Serial.println(result);
    }
}

static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("NODE: ESP-NOW send status = ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "ERROR");
}

static void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len != sizeof(SyncPacket)) {
        return;
    }
    SyncPacket packet;
    memcpy(&packet, incomingData, sizeof(packet));
    if (packet.type != PKT_SYNC || packet.version != TDMA_SYNC_VERSION) {
        return;
    }
    tdma_epoch_us = packet.epoch_us;
    tdma_cycle_us = packet.cycle_us;
    tdma_slot_us = packet.slot_us;
    tdma_guard_us = packet.guard_us;
    tdma_ready = true;
    Serial.print("NODE: sync recibido, ciclo= ");
    Serial.print(tdma_cycle_us);
    Serial.print(" us, slot= ");
    Serial.print(tdma_slot_us);
    Serial.print(" us, guard= ");
    Serial.print(tdma_guard_us);
    Serial.println(" us");
}

void setup() {
    Serial.begin(115200);
    pinMode(A0, INPUT);
    WiFi.mode(WIFI_STA);
    delay(100);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("NODE: error inicializando ESP-NOW");
        while (true) {
            delay(1000);
        }
    }
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, basestation_mac, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("NODE: error agregando peer del basestation");
    }

    Serial.println("NODE: esperando sincronizacion TDMA...");
}

void loop() {
    uint32_t now_us = micros();

    if (next_sample_us == 0) {
        next_sample_us = now_us;
    }

    if ((int32_t)(now_us - next_sample_us) >= 0) {
        if (buffered_samples < TDMA_MAX_SAMPLES_PER_PACKET) {
            sample_buffer[buffered_samples++] = readSensorValue();
        }
        next_sample_us += 1000;
    }

    if (tdma_ready && isMySlot(now_us)) {
        uint32_t cycle = currentSlotCycle(now_us);
        if (cycle != last_sent_cycle && buffered_samples > 0) {
            sendDataPacket();
            last_sent_cycle = cycle;
        }
    }

    delay(0);
}
