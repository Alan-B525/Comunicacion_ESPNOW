import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime

import serial
import serial.tools.list_ports

DEFAULT_SERIAL_PORT = "auto"
DEFAULT_BAUD_RATE = 921600
EXPECTED_SAMPLE_PERIOD_US = 1000
MAX_SAMPLES_PER_PACKET = 8

CSV_HEADERS = [
    "node_id",
    "packet_seq",
    "sample_count",
    "base_ts_us",
    "rx_us",
]
for i in range(MAX_SAMPLES_PER_PACKET):
    CSV_HEADERS.append(f"sample_{i}")
for i in range(MAX_SAMPLES_PER_PACKET):
    CSV_HEADERS.append(f"sample_{i}_ts_us")


def list_ports_text():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return "Sin puertos detectados"
    return " | ".join(f"{port.device} ({port.description})" for port in ports)


def parse_data_header(line):
    if not line.startswith("DATA,"):
        return None

    prefix, separator, values_text = line.partition(",VALUES=")
    if not separator:
        return None

    fields = {}
    for part in prefix.split(",")[1:]:
        key, sep, value = part.partition("=")
        if not sep:
            return None
        fields[key] = value

    required_keys = ("ID", "SEQ", "SAMPLES", "BASE_US", "RX_US")
    if any(key not in fields for key in required_keys):
        return None

    try:
        return {
            "node_id": int(fields["ID"]),
            "packet_seq": int(fields["SEQ"]),
            "sample_count": int(fields["SAMPLES"]),
            "base_ts_us": int(fields["BASE_US"]),
            "rx_us": int(fields["RX_US"]),
            "values_text": values_text,
        }
    except ValueError:
        return None


def parse_samples(values_text):
    samples = []
    for token in values_text.split(","):
        token = token.strip()
        if not token:
            continue
        try:
            samples.append(int(token))
        except ValueError:
            return None
    return samples


def default_output_path():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(os.getcwd(), f"basestation_log_{timestamp}.csv")


def ensure_parent_dir(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.exists(parent):
        os.makedirs(parent, exist_ok=True)


def open_csv_writer(path):
    ensure_parent_dir(path)
    file_exists = os.path.exists(path)
    csv_file = open(path, "a", newline="", encoding="utf-8")
    writer = csv.DictWriter(csv_file, fieldnames=CSV_HEADERS)
    if not file_exists or os.path.getsize(path) == 0:
        writer.writeheader()
        csv_file.flush()
    return csv_file, writer


def build_packet_row(header, samples):
    row = {
        "node_id": header["node_id"],
        "packet_seq": header["packet_seq"],
        "sample_count": header["sample_count"],
        "base_ts_us": header["base_ts_us"],
        "rx_us": header["rx_us"],
    }

    for sample_index in range(MAX_SAMPLES_PER_PACKET):
        sample_key = f"sample_{sample_index}"
        sample_ts_key = f"sample_{sample_index}_ts_us"
        if sample_index < len(samples):
            row[sample_key] = samples[sample_index]
            row[sample_ts_key] = (
                header["base_ts_us"] + sample_index * EXPECTED_SAMPLE_PERIOD_US
            )
        else:
            row[sample_key] = ""
            row[sample_ts_key] = ""

    return row


def parse_args():
    parser = argparse.ArgumentParser(
        description="Guarda en CSV las muestras DATA recibidas por el basestation."
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_SERIAL_PORT,
        help=(
            "Puerto serie a escuchar (ej: COM13) o 'auto' para autodetectar basestation. "
            f"Default: {DEFAULT_SERIAL_PORT}"
        ),
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help=f"Baud rate del puerto serie. Default: {DEFAULT_BAUD_RATE}",
    )
    parser.add_argument(
        "--output",
        default=default_output_path(),
        help="Ruta del archivo CSV de salida. Si no se indica, crea uno con timestamp.",
    )
    parser.add_argument(
        "--no-autostart",
        action="store_true",
        help="No envia TIME y SYNC,START automaticamente al basestation.",
    )
    return parser.parse_args()


def send_command(ser, command):
    payload = f"{command}\n".encode("utf-8")
    ser.write(payload)
    ser.flush()


def _is_basestation_port(port, baud):
    try:
        with serial.Serial(port, baud, timeout=0.2, write_timeout=0.2) as test_ser:
            time.sleep(0.25)
            test_ser.reset_input_buffer()
            test_ser.reset_output_buffer()
            send_command(test_ser, "SYNC?")

            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                raw = test_ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("SYNC_STATUS,"):
                    return True

        return False
    except (serial.SerialException, OSError):
        return False


def resolve_basestation_port(port_arg, baud):
    if port_arg and port_arg.lower() != "auto":
        return port_arg, False

    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if _is_basestation_port(port.device, baud):
            return port.device, True

    return None, True


def sync_basestation_clock_and_start(ser):
    unix_us = int(time.time() * 1_000_000)
    send_command(ser, f"TIME,UNIX_US={unix_us}")
    send_command(ser, "SYNC,START")
    print(f"Comandos enviados al basestation: TIME,UNIX_US={unix_us} ; SYNC,START")


def stop_basestation_sync(ser):
    send_command(ser, "SYNC,STOP")
    print("Comando enviado al basestation: SYNC,STOP")


def main():
    args = parse_args()
    running = True
    saved_rows = 0
    saved_packets = 0
    ser = None

    def stop_handler(_signum, _frame):
        nonlocal running
        running = False
        print("\nDeteniendo captura...")

    signal.signal(signal.SIGINT, stop_handler)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, stop_handler)

    selected_port, was_auto = resolve_basestation_port(args.port, args.baud)
    if not selected_port:
        print("No se encontro basestation en ningun puerto serial.", file=sys.stderr)
        print(f"Puertos detectados: {list_ports_text()}", file=sys.stderr)
        return 1

    csv_file, writer = open_csv_writer(args.output)
    print(f"Guardando CSV en: {os.path.abspath(args.output)}")
    if was_auto:
        print(f"Basestation autodetectado en: {selected_port}")
    print(f"Escuchando {selected_port} @ {args.baud}")
    print(f"Puertos detectados: {list_ports_text()}")

    try:
        with serial.Serial(selected_port, args.baud, timeout=0.5) as ser:
            print("Conectado. Presiona Ctrl+C para terminar.")
            if not args.no_autostart:
                # Permite que el puerto serie se estabilice antes del handshake inicial.
                time.sleep(0.25)
                sync_basestation_clock_and_start(ser)

            while running:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                header = parse_data_header(line)
                if not header:
                    continue

                samples = parse_samples(header["values_text"])
                if samples is None or len(samples) != header["sample_count"]:
                    print(f"Linea DATA invalida ignorada: {line}")
                    continue

                row = build_packet_row(header, samples)
                writer.writerow(row)
                csv_file.flush()

                saved_rows += 1
                saved_packets += 1

                print(
                    f"Packet #{saved_packets}: nodo={header['node_id']} seq={header['packet_seq']} "
                    f"muestras={len(samples)} total_filas={saved_rows}"
                )
    except serial.SerialException as ex:
        print(f"Error serial: {ex}", file=sys.stderr)
        print(f"Puertos detectados: {list_ports_text()}", file=sys.stderr)
        return 1
    finally:
        try:
            if ser and ser.is_open and not args.no_autostart:
                stop_basestation_sync(ser)
        except serial.SerialException:
            pass
        csv_file.close()
        print(
            f"Captura finalizada. Paquetes guardados: {saved_packets}. "
            f"Filas escritas: {saved_rows}."
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
