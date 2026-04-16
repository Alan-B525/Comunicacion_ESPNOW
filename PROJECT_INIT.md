# Proyecto: Comunicación ESP32 ESP-NOW TDMA

## Descripción General
Sistema de comunicación inalámbrica basado en **ESP-NOW** con protocolo **TDMA (Time Division Multiple Access)** para múltiples nodos ESP32 transmitiendo datos de sensores a una estación base.

### Objetivo
- **Basestation**: Recibe datos de hasta 3 nodos ESP32 en slots de tiempo fijos (TDMA).
- **Nodos**: Adquieren datos a **1000 Hz** y transmiten en su slot asignado.
- **Confiabilidad**: Evitar colisiones mediante sincronización temporal.
- **Visualización**: Graficar datos en tiempo real con `plot.py`.

---

## Arquitectura

### Hardware
- **Basestation**: ESP32-C3 DevKit-M1 (Puerto COM10, 115200 baud)
- **Nodos**: ESP32-C3 (hasta 3 unidades, IDs 1, 2, 3)
- **Sensor**: Entrada analógica A0 (0-4095 mapeado a 0-10000)

### Software
- **Framework**: PlatformIO + Arduino + ESP-IDF
- **Protocolo**: ESP-NOW (2.4GHz, sin WiFi)
- **Build Flag**: `ARDUINO_USB_CDC_ON_BOOT=1` (USB serial via CDC)

---

## Estructura del Proyecto

```
Comunicacion_ESPNOW/
├── src/
│   └── main.cpp                 # Código del basestation
├── include/
│   └── tdma_protocol.h          # Definición de estructuras TDMA
├── examples/
│   └── tdma_node_example.cpp    # Código base para nodos (modificar NODE_ID y MAC)
├── plot.py                      # Visualizador en tiempo real
├── save_to_csv.py               # Logger serial a CSV
├── platformio.ini               # Configuración de build
└── PROJECT_INIT.md              # Este archivo
```

---

## Protocolo TDMA

### Parámetros
- **Ciclo TDMA**: 3600 µs (3.6 ms)
- **Slot por nodo**: 1200 µs
- **Guard time**: 200 µs (tolerancia de sincronización)
- **Nodos**: 3 (ID 1, 2, 3)

### Secuencia
```
Ciclo TDMA (3600 µs):
├── Slot 0 (0-1200 µs):    Nodo 1 transmite
├── Slot 1 (1200-2400 µs): Nodo 2 transmite
└── Slot 2 (2400-3600 µs): Nodo 3 transmite
```

### Paquetes

#### 1. SyncPacket (PKT_SYNC = 1)
Enviado por basestation cada 200 ms en broadcast.
```c
struct SyncPacket {
    uint8_t type = 1
    uint8_t version = 1
    uint32_t epoch_us          // Referencia de tiempo
    uint16_t cycle_us = 3600
    uint16_t slot_us = 1200
    uint16_t guard_us = 200
    uint32_t sync_id           // Incrementa en cada sync
}
```

#### 2. DataPacket (PKT_DATA = 2)
Enviado por nodos en su slot, contiene múltiples muestras.
```c
struct DataPacket {
    uint8_t type = 2
    uint8_t sender_id          // 1-3
    uint16_t packet_seq        // Detector de pérdidas
    uint32_t base_ts_us        // Timestamp de primera muestra
    uint8_t sample_count       // Cuántas muestras (máx 8)
    uint16_t samples[8]        // Valores 0-10000
}
```

#### 3. AckPacket (PKT_ACK = 3)
Reservado para confirmaciones futuras.

---

## Flujo de Operación

### Basestation (main.cpp)
1. **setup()**: Inicializa ESP-NOW, WiFi modo STA, registra callbacks.
2. **loop()**: 
   - Cada 200 ms: Envía SyncPacket en broadcast.
   - Recibe DataPacket en callback `OnDataRecv()`.
   - Valida: ID, secuencia, cantidad de muestras.
   - Detecta pérdidas comparando `packet_seq`.
   - Imprime: `DATA,ID=X,SEQ=Y,SAMPLES=Z,BASE_US=...,RX_US=...`

### Nodo (tdma_node_example.cpp)
1. **setup()**: Inicializa ESP-NOW como cliente, registra peer (basestation).
2. **loop()**:
   - Recibe SyncPacket → Sincroniza `tdma_epoch_us`.
   - Adquiere sensor a 1000 Hz → Buffer FIFO.
   - Calcula si está en su slot: `(micros() - epoch_us) % cycle_us`.
   - Si es su slot → Agrupada ~8 muestras en DataPacket.
   - Envía DataPacket a basestation.

---

## Configuración de Nodos

### Paso 1: Obtener MAC del Basestation
En el monitor del basestation, verás:
```
--- ESP32 C3 Base Station ESP-NOW TDMA ---
MAC Address de Base Station: AA:BB:CC:DD:EE:FF
```

### Paso 2: Configurar Nodo
Edita `examples/tdma_node_example.cpp`:
```cpp
#define NODE_ID 1  // Cambia a 1, 2 o 3
#define BASESTATION_MAC {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
```

### Paso 3: Sube el código en nueva instancia PlatformIO
- Copia el contenido a otro proyecto o cambia main.cpp.
- Usa el mismo `platformio.ini` con `ARDUINO_USB_CDC_ON_BOOT=1`.
- Compila y sube a cada ESP32 con su NODE_ID.

---

## Visualización (plot.py)

### Requisitos
```bash
pip install pyserial matplotlib
```

### Uso
```bash
python plot.py
```

- Configura `SERIAL_PORT` ('COM10' en Windows).
- Gráfico superior: Datos en tiempo real.
- Consola inferior: Últimos 10 mensajes del basestation.
- Actualiza cada 1 segundo.

## Logging a CSV (save_to_csv.py)

### Uso
```bash
python save_to_csv.py --port COM10 --output medicion.csv
```

- Escucha líneas `DATA,...,VALUES=...` del basestation.
- Guarda una fila por muestra con `timestamp` y `sample_value`.
- Si no pasas `--output`, crea `basestation_log_YYYYMMDD_HHMMSS.csv`.
- Finaliza con `Ctrl+C`.

---

## Estadísticas y Debugging

### Output del Basestation
- **Cuando recibe datos**: `DATA,ID=1,SEQ=5,SAMPLES=8,BASE_US=12345,RX_US=12600`
- **Control de LED**: Pin 8 parpadea al recibir cada paquete.
- **Estadísticas (comentadas)**: Deshabilitadas por defecto para reducir ruido.

### Habilitar Estadísticas
Descomenta en `main.cpp` loop():
```cpp
if (now - last_stats_time >= STATS_INTERVAL_MS) {
    printStats();
    last_stats_time = now;
}
```

Imprime cada 5 segundos:
```
STATS_BEGIN
Sync enviados: 25
Nodo 1 - Paquetes: 100 | Muestras: 800 | Perdidos: 0 | Errores: 0
Nodo 2 - Paquetes: 0 | Muestras: 0 | Perdidos: 0 | Errores: 0
Nodo 3 - Paquetes: 0 | Muestras: 0 | Perdidos: 0 | Errores: 0
STATS_END
```

---

## Posibles Mejoras / TODOs

- [ ] Implementar ACK desde basestation a nodos (confirmación de recepción).
- [ ] Agregar reintentos si un nodo pierde sincronización.
- [ ] Reducir guard time si la sincronización es estable.
- [ ] Aumentar muestras por paquete si el channel lo permite.
- [ ] Logging a SD card en basestation.
- [ ] Dashboard Web para visualización remota.

---

## Troubleshooting

### No se ven datos en el monitor
- **Verificar puerto COM**: `pio device monitor`
- **Resetear ESP32**: Botón físico si no responde.
- **Compilar con flags correctos**: `ARDUINO_USB_CDC_ON_BOOT=1` en platformio.ini.

### Pérdida frecuente de paquetes
- Aumentar `guard_us` (tolerancia de timing).
- Reducir número de nodos activos.
- Cambiar canal WiFi (check `WIFI_CHANNEL = 1`).

### Nodo no recibe SYNC
- Verificar MAC del basestation es correcta en nodo.
- Ambos deben usar `WIFI_CHANNEL = 1`.
- Esperar más tiempo (SYNC se envía cada 200 ms).

---

## Archivos Clave

| Archivo | Propósito |
|---------|----------|
| `src/main.cpp` | Código basestation (recibe, procesa datos) |
| `include/tdma_protocol.h` | Estructuras de paquetes TDMA |
| `examples/tdma_node_example.cpp` | Plantilla para nodos (modificar ID y MAC) |
| `plot.py` | Visualizador en tiempo real |
| `platformio.ini` | Configuración build + monitor |

---

## Notas Técnicas

- **ESP-NOW**: Capa 2 (no IP), permite comunicación entre ESP32 sin WiFi.
- **TDMA**: Evita colisiones asignando slots temporales fijos.
- **Sincronización**: Cada nodo calcula su slot local usando `micros()` y `epoch_us` del SYNC.
- **Muestreo**: 1000 Hz por nodo × 8 muestras/paquete = ~125 paquetes/s por nodo en modo normal.
- **USB CDC**: Requiere build flag en ESP32-C3 para usar Serial sobre USB nativo.

---

**Última actualización**: Abril 2026  
**Versión del protocolo**: 1  
**Estado**: Beta (funcional, mejoras en progreso)
