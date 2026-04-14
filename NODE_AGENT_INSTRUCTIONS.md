# Instrucciones para el agente de nodos transmisores

## Objetivo
Generar el código Arduino/PlatformIO para los nodos ESP32 C3 que enviarán datos a la base station usando ESP-NOW.

## Requisitos principales
- Cada nodo debe ser un ESP32 C3 independiente.
- Debe enviar paquetes periódicos a la base station.
- El paquete debe incluir:
  - `sender_id` (1, 2, 3)
  - `seq` (número de secuencia)
  - `value` (dato entre 0 y 10000)
  - `tx_ms` (timestamp en el nodo)
- El nodo debe poder ajustar la frecuencia de transmisión con un `delay()` configurable.

## Formato del paquete
Usar esta estructura de datos:

```cpp
struct SensorPacket {
    uint8_t sender_id;
    uint16_t seq;
    uint16_t value;
    uint32_t tx_ms;
};
```

## Comportamiento esperado
- En el `setup()`:
  - inicializar `Serial` a `115200`
  - poner WiFi en `WIFI_STA`
  - inicializar `esp_now`
  - registrar callback de envío
  - agregar la base station como peer usando su MAC (obtenida desde la base)
- En el `loop()`:
  - generar un valor que suba de `0` a `10000` y luego baje nuevamente
  - enviar el paquete con `esp_now_send()` a la base station
  - mostrar en `Serial` el estado de envío y el valor actual
  - usar un `delay()` configurable para probar distintas frecuencias

## Parámetros configurables
- `senderID` de cada nodo (1, 2, 3)
- `broadcastAddress` o `peer MAC` de la base station
- intervalo de envío (`delay_ms`)
- modo de datos:
  - secuencia creciente y decreciente entre `0` y `10000`

## Errores y diagnóstico
- Incluir mensajes serial claros para:
  - error en inicialización de ESP-NOW
  - error al agregar el peer
  - resultado de cada envío
  - cambio de dirección de la secuencia cuando llegue a 0 o 10000

## Extras recomendados
- Agregar un contador de paquetes enviados
- Permitir cambiar la frecuencia sin recompilar (por ejemplo, con `#define TX_INTERVAL_MS`)
- Si el nodo falla al enviar, reintentar después de un pequeño `delay`

## Nombre del archivo sugerido
- `src/sender.cpp`

## Especial atención
- El código de los nodos NO debe esperar datos de la base station.
- Debe funcionar en modo transmitente, enviando siempre al MAC de la base station.
- No es necesario usar encriptación para las pruebas iniciales.
