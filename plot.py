import threading
import time
from collections import defaultdict, deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports

SERIAL_PORT = "COM13"
BAUD_RATE = 921600
MAX_NODES = 10
MAX_POINTS = 2000
MAX_CONSOLE_LINES = 14
EXPECTED_SAMPLE_PERIOD_S = 0.001
STALE_NODE_SECONDS = 2.5

NODE_COLORS = [
    "#ff595e", "#1982c4", "#8ac926", "#ffca3a", "#6a4c93",
    "#fb8500", "#219ebc", "#d62828", "#3a86ff", "#2a9d8f",
]

state_lock = threading.Lock()
node_time = defaultdict(lambda: deque(maxlen=MAX_POINTS))
node_value = defaultdict(lambda: deque(maxlen=MAX_POINTS))
node_packet_samples = defaultdict(lambda: deque(maxlen=MAX_POINTS))
node_loss_events = defaultdict(lambda: deque(maxlen=50))
node_stats = defaultdict(
    lambda: {
        "packets": 0,
        "samples": 0,
        "last_seq": 0,
        "last_rx": 0.0,
        "has_values": False,
        "lost_packets": 0,
        "gap_events": 0,
        "last_gap_rx": 0.0,
    }
)
console_lines = deque(maxlen=MAX_CONSOLE_LINES)
parse_debug_lines = deque(maxlen=8)
serial_status = {"connected": False, "error": ""}
program_start = time.time()
ani = None


def color_for_node(node_id):
    idx = (max(1, node_id) - 1) % len(NODE_COLORS)
    return NODE_COLORS[idx]


def list_ports_text():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return "Sin puertos detectados"
    return " | ".join(f"{p.device} ({p.description})" for p in ports)


def append_console(msg):
    with state_lock:
        console_lines.append(msg)


def parse_data_header(line):
    if not line.startswith("DATA,"):
        return None

    prefix, separator, values_text = line.partition(",VALUES=")
    if not separator:
        prefix = line
        values_text = ""

    fields = {}
    for part in prefix.split(",")[1:]:
        key, sep, value = part.partition("=")
        if not sep:
            return None
        fields[key] = value

    required_keys = ("ID", "SEQ", "SAMPLES", "BASE_US", "RX_US")
    if any(k not in fields for k in required_keys):
        return None

    try:
        sender_id = int(fields["ID"])
        if sender_id < 1 or sender_id > MAX_NODES:
            return None
        return {
            "sender_id": sender_id,
            "packet_seq": int(fields["SEQ"]),
            "sample_count": int(fields["SAMPLES"]),
            "base_ts_us": int(fields["BASE_US"]),
            "values_text": values_text,
            "has_values": bool(separator),
        }
    except ValueError:
        return None


def parse_loss_header(line):
    if not line.startswith("LOSS,"):
        return None

    fields = {}
    for part in line.split(",")[1:]:
        key, sep, value = part.partition("=")
        if not sep:
            return None
        fields[key] = value

    required_keys = ("ID", "MISSED", "PREV_SEQ", "CURR_SEQ", "RX_US")
    if any(k not in fields for k in required_keys):
        return None

    try:
        sender_id = int(fields["ID"])
        if sender_id < 1 or sender_id > MAX_NODES:
            return None
        return {
            "sender_id": sender_id,
            "missed": int(fields["MISSED"]),
            "prev_seq": int(fields["PREV_SEQ"]),
            "curr_seq": int(fields["CURR_SEQ"]),
            "rx_us": int(fields["RX_US"]),
        }
    except ValueError:
        return None


def parse_samples(values_text):
    samples = []
    for tok in values_text.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            samples.append(int(tok))
        except ValueError:
            return None
    return samples


def build_sample_times(base_ts_us, sample_count):
    base_s = base_ts_us / 1_000_000.0
    return [base_s + i * EXPECTED_SAMPLE_PERIOD_S for i in range(sample_count)]


def clear_stale_nodes_locked(now_s):
    for node_id, stats in node_stats.items():
        last_rx = stats.get("last_rx", 0.0)
        if last_rx > 0 and (now_s - last_rx) > STALE_NODE_SECONDS:
            node_time[node_id].clear()
            node_value[node_id].clear()


def serial_reader():
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.2)
            with state_lock:
                serial_status["connected"] = True
                serial_status["error"] = ""
            append_console(f"Conectado a {SERIAL_PORT} @ {BAUD_RATE}")

            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                append_console(line)

                loss = parse_loss_header(line)
                if loss:
                    sender_id = loss["sender_id"]
                    now_s = time.time()
                    with state_lock:
                        node_stats[sender_id]["lost_packets"] += max(0, loss["missed"])
                        node_stats[sender_id]["gap_events"] += 1
                        node_stats[sender_id]["last_gap_rx"] = now_s
                        node_loss_events[sender_id].append(loss)
                    continue

                header = parse_data_header(line)
                if not header:
                    if line.startswith("DATA,"):
                        with state_lock:
                            parse_debug_lines.append(f"Cabecera invalida: {line}")
                    continue

                sender_id = header["sender_id"]
                now_s = time.time()
                with state_lock:
                    node_stats[sender_id]["packets"] += 1
                    node_stats[sender_id]["last_seq"] = header["packet_seq"]
                    node_stats[sender_id]["last_rx"] = now_s
                    node_stats[sender_id]["has_values"] = header["has_values"]
                    node_packet_samples[sender_id].append(header["sample_count"])

                samples = parse_samples(header["values_text"])
                if samples is None:
                    with state_lock:
                        parse_debug_lines.append(f"VALUES invalido: {line}")
                    continue
                if not samples:
                    continue

                times = build_sample_times(header["base_ts_us"], len(samples))
                with state_lock:
                    node_time[sender_id].extend(times)
                    node_value[sender_id].extend(samples)
                    node_stats[sender_id]["samples"] += len(samples)

        except serial.SerialException as ex:
            with state_lock:
                serial_status["connected"] = False
                serial_status["error"] = str(ex)
            append_console(f"Error serial: {ex}")
            append_console(f"Puertos: {list_ports_text()}")
            time.sleep(1.0)
        except Exception as ex:
            with state_lock:
                serial_status["connected"] = False
                serial_status["error"] = str(ex)
            append_console(f"Error: {ex}")
            time.sleep(1.0)


def normalize_times(arr):
    if not arr:
        return []
    t0 = arr[0]
    return [x - t0 for x in arr]


def update(_frame):
    with state_lock:
        clear_stale_nodes_locked(time.time())
        snapshot_t = {k: list(v) for k, v in node_time.items()}
        snapshot_y = {k: list(v) for k, v in node_value.items()}
        snapshot_stats = {k: dict(v) for k, v in node_stats.items()}
        snapshot_loss = {k: list(v) for k, v in node_loss_events.items()}
        snapshot_packet_samples = {k: list(v) for k, v in node_packet_samples.items()}
        snapshot_console = list(console_lines)
        snapshot_parse_debug = list(parse_debug_lines)
        connected = serial_status["connected"]
        error = serial_status["error"]

    fig.patch.set_facecolor("#f7f9fc")
    ax_header.clear()
    ax_stats.clear()
    ax_console.clear()
    for ax in node_axes:
        ax.clear()

    uptime = time.time() - program_start
    ax_header.axis("off")
    ax_header.text(0.01, 0.62, "ESP-NOW TDMA Monitor", fontsize=17, fontweight="bold", color="#1f2937")
    ax_header.text(0.01, 0.18, f"Puerto: {SERIAL_PORT} | Baud: {BAUD_RATE} | Uptime: {uptime:0.1f}s", fontsize=10, color="#334155")
    ax_header.text(
        0.99,
        0.45,
        "ONLINE" if connected else "OFFLINE",
        ha="right",
        va="center",
        fontsize=11,
        fontweight="bold",
        color="white",
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "#2a9d8f" if connected else "#e63946", "edgecolor": "none"},
    )

    active_nodes = sorted(snapshot_stats.keys())[:MAX_NODES]
    for idx, node_id in enumerate(active_nodes[:len(node_axes)]):
        ax = node_axes[idx]
        vals = snapshot_y.get(node_id, [])
        times = normalize_times(snapshot_t.get(node_id, []))
        sample_meta = snapshot_packet_samples.get(node_id, [])
        stats = snapshot_stats[node_id]
        age = time.time() - stats.get("last_rx", 0.0)
        alive = age <= STALE_NODE_SECONDS
        color = color_for_node(node_id)

        if times and vals:
            ax.plot(times, vals, color=color, linewidth=2.0)
            ax.set_ylabel("Valor")
        else:
            ax.plot([], [])

        ax.set_title(f"Nodo {node_id} {'RX' if alive else 'SIN DATOS'}", fontsize=10, fontweight="bold")
        ax.set_xlabel("Tiempo (s)")
        ax.grid(True, linestyle="--", alpha=0.3)
        if not vals and sample_meta:
            ax.text(0.03, 0.9, "Sin VALUES en stream serial", transform=ax.transAxes, fontsize=8, color="#b45309")

    for idx in range(len(active_nodes), len(node_axes)):
        node_axes[idx].axis("off")

    ax_stats.axis("off")
    stats_lines = ["ESTADISTICAS", "-"]
    if active_nodes:
        now_s = time.time()
        for node_id in active_nodes:
            st = snapshot_stats[node_id]
            age = now_s - st.get("last_rx", 0.0)
            gap_age = now_s - st.get("last_gap_rx", 0.0) if st.get("last_gap_rx", 0.0) > 0 else -1
            state = "RX" if age <= STALE_NODE_SECONDS else "SIN DATOS"
            mode = "VALUES" if st.get("has_values", False) else "SOLO META"
            recent_losses = sum(ev.get("missed", 0) for ev in snapshot_loss.get(node_id, [])[-5:])
            gap_txt = f"gap_age={gap_age:0.1f}s" if gap_age >= 0 else "gap_age=NA"
            stats_lines.append(
                f"N{node_id:02d} | {state:<8} | {mode:<9} | pkt={st['packets']:<5} samp={st['samples']:<6} seq={st['last_seq']:<6}"
            )
            stats_lines.append(
                f"     lost={st['lost_packets']:<5} gaps={st['gap_events']:<4} recent5={recent_losses:<4} age={age:0.1f}s {gap_txt}"
            )
    else:
        stats_lines.append("Esperando nodos...")
    if snapshot_parse_debug:
        stats_lines.append("-")
        stats_lines.extend(snapshot_parse_debug[-3:])

    ax_stats.text(
        0.01,
        0.98,
        "\n".join(stats_lines),
        va="top",
        ha="left",
        family="monospace",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "#eef2ff", "edgecolor": "#c7d2fe"},
        color="#1f2937",
    )

    ax_console.axis("off")
    title = "Consola" if not error else f"Consola | Error: {error}"
    text = "\n".join(snapshot_console[-MAX_CONSOLE_LINES:]) if snapshot_console else "Esperando mensajes..."
    ax_console.text(
        0.01,
        0.98,
        f"{title}\n" + "-" * 52 + f"\n{text}",
        va="top",
        ha="left",
        family="monospace",
        fontsize=8.5,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "#0f172a", "edgecolor": "#1e293b", "alpha": 0.96},
        color="#e2e8f0",
    )


def main():
    reader = threading.Thread(target=serial_reader, daemon=True)
    reader.start()

    global fig, ax_header, node_axes, ax_stats, ax_console, ani
    plt.style.use("tableau-colorblind10")
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(7, 2, height_ratios=[0.32, 1, 1, 1, 1, 1, 0.9])

    ax_header = fig.add_subplot(gs[0, :])
    node_axes = [fig.add_subplot(gs[r, c]) for r in range(1, 6) for c in range(2)]
    ax_stats = fig.add_subplot(gs[6, 0])
    ax_console = fig.add_subplot(gs[6, 1])

    ani = animation.FuncAnimation(fig, update, interval=180, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()