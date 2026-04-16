#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <tdma_protocol.h>
#include <stdio.h>
#include <string.h>

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
static bool sync_stream_enabled = false;
static bool wallclock_synced = false;
static uint64_t wallclock_ref_unix_us = 0;
static uint32_t wallclock_ref_micros = 0;
static char serial_command_buffer[96] = {0};
static size_t serial_command_len = 0;

static void sendSync();

static void printUint64(uint64_t value) {
    uint32_t high = static_cast<uint32_t>(value / 1000000000ULL);
    uint32_t low = static_cast<uint32_t>(value % 1000000000ULL);
    if (high > 0) {
        Serial.print(high);
        char low_buffer[10];
        snprintf(low_buffer, sizeof(low_buffer), "%09lu", static_cast<unsigned long>(low));
        Serial.print(low_buffer);
    } else {
        Serial.print(low);
    }
}

static uint64_t currentUnixTimeUs() {
    if (!wallclock_synced) {
        return 0;
    }
    uint32_t now_us = micros();
    uint32_t delta_us = now_us - wallclock_ref_micros;
    return wallclock_ref_unix_us + static_cast<uint64_t>(delta_us);
}

static void setWallclockUnixUs(uint64_t unix_us) {
    wallclock_ref_unix_us = unix_us;
    wallclock_ref_micros = micros();
    wallclock_synced = true;

    Serial.print("TIME_SET,UNIX_US=");
    printUint64(unix_us);
    Serial.print(",MICROS_REF=");
    Serial.println(wallclock_ref_micros);
}

static void printClockStatus() {
    if (!wallclock_synced) {
        Serial.println("TIME_STATUS,STATE=UNSYNCED");
        return;
    }

    uint64_t unix_us = currentUnixTimeUs();
    Serial.print("TIME_STATUS,STATE=SYNCED,UNIX_US=");
    printUint64(unix_us);
    Serial.println();
}

static void printSyncStatus() {
    Serial.print("SYNC_STATUS,STATE=");
    Serial.print(sync_stream_enabled ? "RUNNING" : "STOPPED");
    Serial.print(",SYNC_SENT=");
    Serial.println(sync_sent_count);
}

static bool tryParseUnixSerialValue(const char *text, uint64_t *out_value) {
    if (!text || !*text) {
        return false;
    }

    uint64_t value = 0;
    for (const char *p = text; *p; ++p) {
        if (*p < '0' || *p > '9') {
            return false;
        }
        value = value * 10ULL + static_cast<uint64_t>(*p - '0');
    }

    *out_value = value;
    return true;
}

static void handleSerialCommand(const char *line) {
    if (strcmp(line, "SYNC,START") == 0) {
        sync_stream_enabled = true;
        last_sync_send = millis();
        sendSync();
        Serial.println("SYNC_CTRL,STATE=RUNNING");
        return;
    }

    if (strcmp(line, "SYNC,STOP") == 0) {
        sync_stream_enabled = false;
        Serial.println("SYNC_CTRL,STATE=STOPPED");
        return;
    }

    if (strcmp(line, "SYNC?") == 0) {
        printSyncStatus();
        return;
    }

    if (strncmp(line, "TIME,UNIX_US=", 13) == 0) {
        uint64_t unix_us = 0;
        if (tryParseUnixSerialValue(line + 13, &unix_us)) {
            setWallclockUnixUs(unix_us);
        } else {
            Serial.println("TIME_ERROR,REASON=INVALID_UNIX_US");
        }
        return;
    }

    if (strncmp(line, "TIME,UNIX_MS=", 13) == 0) {
        uint64_t unix_ms = 0;
        if (tryParseUnixSerialValue(line + 13, &unix_ms)) {
            setWallclockUnixUs(unix_ms * 1000ULL);
        } else {
            Serial.println("TIME_ERROR,REASON=INVALID_UNIX_MS");
        }
        return;
    }

    if (strcmp(line, "TIME?") == 0) {
        printClockStatus();
        return;
    }

    Serial.print("CMD_UNKNOWN,VALUE=");
    Serial.println(line);
}

static void processSerialCommands() {
    while (Serial.available() > 0) {
        char ch = static_cast<char>(Serial.read());
        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            serial_command_buffer[serial_command_len] = '\0';
            if (serial_command_len > 0) {
                handleSerialCommand(serial_command_buffer);
            }
            serial_command_len = 0;
            continue;
        }

        if (serial_command_len + 1 < sizeof(serial_command_buffer)) {
            serial_command_buffer[serial_command_len++] = ch;
        } else {
            serial_command_len = 0;
            Serial.println("CMD_ERROR,REASON=TOO_LONG");
        }
    }
}

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
    Serial.println("Comandos seriales:");
    Serial.println("  SYNC,START");
    Serial.println("  SYNC,STOP");
    Serial.println("  SYNC?");
    Serial.println("  TIME,UNIX_MS=1713200000123");
    Serial.println("  TIME,UNIX_US=1713200000123456");
    Serial.println("  TIME?");
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
    packet.unix_time_us = currentUnixTimeUs();

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
}

void loop() {
    processSerialCommands();

    if (blink_led) {
        digitalWrite(RX_LED, HIGH);
        delay(50);
        digitalWrite(RX_LED, LOW);
        blink_led = false;
    }

    uint32_t now = millis();
    if (sync_stream_enabled && (int32_t)(now - last_sync_send) >= 0) {
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

