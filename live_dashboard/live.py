"""
Enhanced Live Cognitive Radio Dashboard (Receiver-side)
--------------------------------------------------------
Single-window desktop dashboard built with PyQtGraph (much lower latency
than matplotlib for streaming serial data, and lets us mix live plots
with live tables/logs in one native window).

Panels:
  [top]           Connection / link status bar (port, uptime, packet
                   counters, CRC failures, jammer captures, current
                   SF/CR/frequency, PLR)
  [mid-left]      RSSI / SNR over time (data plane)
  [mid-right]     Live per-packet table -- every decoded packet
                   (seq, RSSI, SNR, CFO, sensor value), newest on top,
                   colour-coded for link quality
  [bottom-left]   Control-plane timeline: BUSY_SKIP / PREPARE_SENT /
                   PREPARE_ACK / COMMIT_ACK / APPLIED / FAILED, with
                   each handshake's steps joined by a line and labelled
                   with its commandID so you can follow one command
                   through its whole lifecycle
  [bottom-right]  Rolling activity/event log (every control transition,
                   CRC failure, jammer capture, and any unrecognised
                   serial line) -- nothing silently dropped

Requires the patched cr_receiver.ino (adds PKTROW / PKTEVT / full
CTRLROW,RX,{PREPARE_ACK,COMMIT_ACK,APPLIED,FAILED} lines).

Install:
    pip install pyqtgraph pyserial PyQt5

Run:
    python live_dashboard.py
"""

import sys
import time
import random
import threading
import queue
from collections import deque

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

try:
    import serial
except ImportError:
    serial = None

# ============================================================= CONFIG
RX_PORT = "COM8"          # <-- change to your receiver's port (e.g. /dev/ttyUSB0)
BAUD = 115200

DEMO_MODE = False          # True = generate synthetic data, no hardware needed.
                           # Good for previewing the dashboard before wiring
                           # up the real firmware.

DATA_WINDOW = 300         # samples kept on the RSSI/SNR plot
CTRL_WINDOW = 40          # control-plane events kept on the timeline
TABLE_ROWS = 300          # packet rows kept in the live table
LOG_MAX = 500             # lines kept in the activity log

SYNC_INTERVAL_S = 8.0     # must match SYNC_INTERVAL_MS in cr_receiver.ino
SYNC_HISTORY = 40         # ping/pong results kept for the sync history strip
AI_HISTORY = 25           # per-window AI decisions kept in the AI panel
CMD_HISTORY = 12          # control-plane commands kept in the handshake list

RX_STATES = ["BUSY_SKIP", "PREPARE_SENT", "PREPARE_ACK",
             "COMMIT_ACK", "APPLIED", "FAILED"]

STATE_COLOR = {
    "BUSY_SKIP":     "#e74c3c",
    "FAILED":        "#e74c3c",
    "PREPARE_SENT":  "#e67e22",
    "PREPARE_ACK":   "#3498db",
    "COMMIT_ACK":    "#2980b9",
    "APPLIED":       "#2ecc71",
}

# prediction class -> (label, action description, colour) -- mirrors the
# switch in executeDecision() in cr_receiver.ino
AI_LABELS = {
    0: ("Normal Channel",    "No action -- link is healthy",                "#2ecc71"),
    1: ("Jammer Detected",   "Channel hopping (+200 kHz)",                  "#e74c3c"),
    2: ("Weak Link",         "Increasing robustness (SF/CR stepped up)",    "#e67e22"),
    3: ("Excellent Link",    "Optimizing throughput (SF/CR stepped down)",  "#3498db"),
}

SYNC_STATUS_COLOR = {
    "PING_SENT":     "#5a5f70",
    "MATCH":         "#2ecc71",
    "MISMATCH":      "#e67e22",
    "TIMEOUT":       "#e74c3c",
    "DESYNC_FIXED":  "#e74c3c",
    "PING_FAIL":     "#e74c3c",
}

SYNC_STATUS_LABEL = {
    "PING_SENT":     "PING sent, awaiting PONG",
    "MATCH":         "PONG OK -- configs match",
    "MISMATCH":      "PONG mismatch",
    "TIMEOUT":       "PONG timeout",
    "DESYNC_FIXED":  "Desync confirmed -- receiver corrected",
    "PING_FAIL":     "PING transmit failed",
}

# ============================================================= PARSING
def parse_line(line: str, sink: "queue.Queue"):
    """Turn one raw serial line into a structured event dict on `sink`.
    Anything unrecognised still gets forwarded as a 'raw' event so the
    activity log shows literally everything coming off the serial port.
    """
    if line.startswith("PKTROW,"):
        p = line.split(",")
        try:
            sink.put({
                "type": "packet",
                "seq": int(p[1]), "rssi": float(p[2]), "snr": float(p[3]),
                "cfo": float(p[4]), "sensor": int(p[5]),
                "fw_ts": int(p[6]) if len(p) > 6 else 0,
            })
        except (IndexError, ValueError):
            sink.put({"type": "raw", "text": line})

    elif line.startswith("PKTEVT,CRC_FAIL"):
        sink.put({"type": "crc_fail"})

    elif line.startswith("PKTEVT,JAMMER,"):
        p = line.split(",")
        try:
            sink.put({"type": "jammer", "rssi": float(p[2]),
                      "snr": float(p[3]), "cfo": float(p[4])})
        except (IndexError, ValueError):
            sink.put({"type": "raw", "text": line})

    elif line.startswith("CTRLROW,RX,"):
        p = line.split(",")
        if len(p) >= 7:
            sink.put({"type": "ctrl", "state": p[2], "cid": p[3],
                       "freq": p[4], "sf": p[5], "cr": p[6]})
        else:
            sink.put({"type": "raw", "text": line})

    elif line.startswith("AIROW,"):
        p = line.split(",")
        try:
            sink.put({"type": "ai", "prediction": int(p[1])})
        except (IndexError, ValueError):
            sink.put({"type": "raw", "text": line})

    elif line.startswith("SYNCROW,"):
        p = line.split(",")
        status = p[1] if len(p) > 1 else "?"
        detail = {}
        if status == "MISMATCH" and len(p) >= 8:
            try:
                detail = {"rx_freq": p[2], "rx_sf": p[3], "rx_cr": p[4],
                           "tx_freq": p[5], "tx_sf": p[6], "tx_cr": p[7]}
            except IndexError:
                detail = {}
        elif status == "PING_FAIL" and len(p) >= 3:
            detail = {"err": p[2]}
        sink.put({"type": "sync", "status": status, "detail": detail})

    elif line.startswith("CSVROW,"):
        p = line.split(",")
        try:
            sink.put({
                "type": "window",
                "meanRSSI": float(p[1]), "varRSSI": float(p[2]),
                "meanSNR": float(p[3]), "varSNR": float(p[4]),
                "cfo": float(p[5]), "plr": float(p[6]),
                "crc": int(float(p[7])), "sf": int(float(p[8])),
                "cr": int(float(p[9])),
            })
        except (IndexError, ValueError):
            sink.put({"type": "raw", "text": line})

    else:
        stripped = line.strip()
        if stripped:
            sink.put({"type": "raw", "text": stripped})


def serial_reader(port, baud, sink, stop_event):
    """Background thread. Never blocks the GUI thread."""
    if serial is None:
        sink.put({"type": "status", "connected": False,
                   "error": "pyserial not installed"})
        return
    while not stop_event.is_set():
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                sink.put({"type": "status", "connected": True, "port": port})
                while not stop_event.is_set():
                    raw = ser.readline().decode(errors="ignore").strip()
                    if raw:
                        parse_line(raw, sink)
        except serial.SerialException as e:
            sink.put({"type": "status", "connected": False,
                       "port": port, "error": str(e)})
            time.sleep(2)


def demo_generator(sink, stop_event):
    """Synthetic feed so the dashboard can be exercised with no hardware
    attached. Mirrors the real event mix: drifting RSSI/SNR, periodic
    control-plane handshakes, occasional CRC failures / jammer bursts.
    """
    sink.put({"type": "status", "connected": True, "port": "DEMO"})
    seq = 0
    cid = 0
    freq, sf, cr = 433000, 8, 5
    rssi_base, snr_base = -95.0, 8.0
    next_ctrl_at = time.time() + 6
    next_sync_at = time.time() + SYNC_INTERVAL_S
    next_ai_at = time.time() + 4
    while not stop_event.is_set():
        seq += 1
        rssi_base += random.uniform(-1.5, 1.5)
        snr_base += random.uniform(-0.5, 0.5)
        sink.put({"type": "packet", "seq": seq,
                   "rssi": rssi_base, "snr": snr_base,
                   "cfo": random.uniform(-200, 200),
                   "sensor": random.randint(200, 900), "fw_ts": seq})

        if random.random() < 0.06:
            sink.put({"type": "crc_fail"})
        if random.random() < 0.03:
            sink.put({"type": "jammer", "rssi": random.uniform(-60, -40),
                       "snr": random.uniform(-5, 5), "cfo": random.uniform(-500, 500)})

        if time.time() >= next_ctrl_at:
            cid += 1
            freq += 200
            sink.put({"type": "ctrl", "state": "PREPARE_SENT", "cid": str(cid),
                       "freq": str(freq), "sf": str(sf), "cr": str(cr)})
            time.sleep(0.3)
            sink.put({"type": "ctrl", "state": "PREPARE_ACK", "cid": str(cid),
                       "freq": str(freq), "sf": str(sf), "cr": str(cr)})
            time.sleep(0.3)
            if random.random() < 0.85:
                sink.put({"type": "ctrl", "state": "COMMIT_ACK", "cid": str(cid),
                           "freq": str(freq), "sf": str(sf), "cr": str(cr)})
                time.sleep(0.5)
                sink.put({"type": "ctrl", "state": "APPLIED", "cid": str(cid),
                           "freq": str(freq), "sf": str(sf), "cr": str(cr)})
            else:
                sink.put({"type": "ctrl", "state": "FAILED", "cid": str(cid),
                           "freq": "0", "sf": "0", "cr": "0"})
            next_ctrl_at = time.time() + random.uniform(10, 18)

        sink.put({"type": "window", "meanRSSI": rssi_base, "varRSSI": 1.2,
                   "meanSNR": snr_base, "varSNR": 0.4, "cfo": 50.0,
                   "plr": random.uniform(0, 0.1), "crc": random.randint(0, 2),
                   "sf": sf, "cr": cr})

        if time.time() >= next_ai_at:
            sink.put({"type": "ai", "prediction": random.choices(
                [0, 1, 2, 3], weights=[0.55, 0.1, 0.2, 0.15])[0]})
            next_ai_at = time.time() + random.uniform(3, 6)

        if time.time() >= next_sync_at:
            sink.put({"type": "sync", "status": "PING_SENT", "detail": {}})
            roll = random.random()
            if roll < 0.85:
                sink.put({"type": "sync", "status": "MATCH", "detail": {}})
            elif roll < 0.95:
                sink.put({"type": "sync", "status": "MISMATCH", "detail": {
                    "rx_freq": str(freq), "rx_sf": str(sf), "rx_cr": str(cr),
                    "tx_freq": str(freq), "tx_sf": str(sf), "tx_cr": str(cr)}})
            else:
                sink.put({"type": "sync", "status": "TIMEOUT", "detail": {}})
            next_sync_at = time.time() + SYNC_INTERVAL_S

        time.sleep(0.4)


# ============================================================= GUI
def link_quality_color(rssi):
    if rssi >= -80:
        return QtGui.QColor("#2ecc71")
    if rssi >= -100:
        return QtGui.QColor("#e67e22")
    return QtGui.QColor("#e74c3c")


class Dashboard(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cognitive Radio - Live Receiver Dashboard")
        self.resize(1500, 900)

        pg.setConfigOptions(antialias=True)
        pg.setConfigOption("background", "#161821")
        pg.setConfigOption("foreground", "#e0e0e0")

        # ---------------- state buffers ----------------
        self.t0 = time.time()
        self.rx_t = deque(maxlen=DATA_WINDOW)
        self.rx_rssi = deque(maxlen=DATA_WINDOW)
        self.rx_snr = deque(maxlen=DATA_WINDOW)

        self.ctrl_events = deque(maxlen=CTRL_WINDOW)   # dicts: t, state, cid, freq, sf, cr
        self.cmd_order = deque(maxlen=CMD_HISTORY)      # cids, oldest first
        self.cmd_summary = {}                            # cid -> {states:[(t,state)], freq, sf, cr}
        self.cmd_dirty = True

        self.sync_history = deque(maxlen=SYNC_HISTORY)  # dicts: t, status, detail
        self.sync_state = {"last_status": None, "last_t": None,
                             "last_ping_t": None, "mismatch_streak": 0}
        self.sync_dirty = True

        self.last_window = None                          # most recent CSVROW dict
        self.ai_history = deque(maxlen=AI_HISTORY)       # dicts: t, prediction, label, action, window
        self.ai_dirty = True

        self.stats = {
            "connected": False, "port": "-", "total_packets": 0,
            "crc_fail": 0, "jammer": 0, "plr": 0.0,
            "sf": 8, "cr": 5, "freq": 433000,
        }

        self._build_ui()

        # ---------------- background feed ----------------
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        target = demo_generator if DEMO_MODE else serial_reader
        args = (self.q, self.stop_event) if DEMO_MODE else (RX_PORT, BAUD, self.q, self.stop_event)
        threading.Thread(target=target, args=args, daemon=True).start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.drain_queue)
        self.timer.start(100)  # 10 Hz UI refresh

    # ------------------------------------------------------------ UI build
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setSpacing(8)

        root.addWidget(self._build_status_bar())

        splitter_top = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter_top.addWidget(self._build_data_plot())
        splitter_top.addWidget(self._build_packet_table())
        splitter_top.setSizes([700, 700])

        splitter_bottom_left = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        splitter_bottom_left.addWidget(self._build_sync_panel())
        splitter_bottom_left.addWidget(self._build_ai_panel())
        splitter_bottom_left.setSizes([260, 260])

        splitter_bottom = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter_bottom.addWidget(splitter_bottom_left)
        splitter_bottom.addWidget(self._build_event_log())
        splitter_bottom.setSizes([760, 640])

        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        main_splitter.addWidget(splitter_top)
        main_splitter.addWidget(splitter_bottom)
        main_splitter.setSizes([450, 400])
        root.addWidget(main_splitter)

    def _build_status_bar(self):
        bar = QtWidgets.QFrame()
        bar.setStyleSheet("QFrame { background:#1e202c; border-radius:6px; }"
                            "QLabel { color:#e0e0e0; font-size:13px; }")
        lay = QtWidgets.QHBoxLayout(bar)

        def stat(name, initial="-"):
            box = QtWidgets.QVBoxLayout()
            title = QtWidgets.QLabel(name)
            title.setStyleSheet("color:#8a8fa3; font-size:10px;")
            value = QtWidgets.QLabel(initial)
            value.setStyleSheet("font-weight:bold; font-size:15px;")
            box.addWidget(title)
            box.addWidget(value)
            wrap = QtWidgets.QWidget()
            wrap.setLayout(box)
            lay.addWidget(wrap)
            return value

        self.lbl_status = stat("LINK", "\u25CF DISCONNECTED")
        self.lbl_status.setStyleSheet("font-weight:bold; font-size:15px; color:#e74c3c;")
        self.lbl_port = stat("PORT", RX_PORT if not DEMO_MODE else "DEMO")
        self.lbl_uptime = stat("UPTIME", "0s")
        self.lbl_packets = stat("PACKETS", "0")
        self.lbl_crc = stat("CRC FAILS", "0")
        self.lbl_jammer = stat("JAMMER CAPTURES", "0")
        self.lbl_plr = stat("PLR (latest window)", "0.0%")
        self.lbl_cfg = stat("FREQ / SF / CR", "- / - / -")
        lay.addStretch()
        return bar

    def _build_data_plot(self):
        pw = pg.PlotWidget(title="RX Data Plane -- RSSI / SNR")
        pw.addLegend(offset=(10, 10))
        pw.setLabel("bottom", "time (s)")
        pw.showGrid(x=True, y=True, alpha=0.2)
        self.curve_rssi = pw.plot([], [], pen=pg.mkPen("#3498db", width=2), name="RSSI (dBm)")
        self.curve_snr = pw.plot([], [], pen=pg.mkPen("#2ecc71", width=2), name="SNR (dB)")
        self.plot_data = pw
        return pw

    def _build_packet_table(self):
        wrap = QtWidgets.QGroupBox("RX Data Plane -- Live Packet Feed")
        wrap.setStyleSheet("QGroupBox { color:#e0e0e0; font-weight:bold; }")
        v = QtWidgets.QVBoxLayout(wrap)
        table = QtWidgets.QTableWidget(0, 6)
        table.setHorizontalHeaderLabels(
            ["t (s)", "Seq", "RSSI (dBm)", "SNR (dB)", "CFO (Hz)", "Sensor"])
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        table.setStyleSheet("QTableWidget { background:#161821; color:#e0e0e0; gridline-color:#2a2d3a;}"
                              "QHeaderView::section { background:#1e202c; color:#8a8fa3; }")
        v.addWidget(table)
        self.packet_table = table
        return wrap

    def _build_sync_panel(self):
        wrap = QtWidgets.QGroupBox("RX Control Plane -- Sync Heartbeat & Command Handshakes")
        wrap.setStyleSheet("QGroupBox { color:#e0e0e0; font-weight:bold; }")
        v = QtWidgets.QVBoxLayout(wrap)

        # ---- headline sync status row ----
        self.lbl_sync_status = QtWidgets.QLabel("No PING sent yet")
        self.lbl_sync_status.setStyleSheet("font-weight:bold; font-size:14px; color:#8a8fa3;")
        v.addWidget(self.lbl_sync_status)

        self.lbl_sync_meta = QtWidgets.QLabel(
            "last PING: -- | next PING in: -- | mismatch streak: 0")
        self.lbl_sync_meta.setStyleSheet("color:#8a8fa3; font-size:11px;")
        v.addWidget(self.lbl_sync_meta)

        # ---- rolling ping/pong result strip (bead per heartbeat) ----
        strip_wrap = QtWidgets.QWidget()
        self.sync_strip_layout = QtWidgets.QHBoxLayout(strip_wrap)
        self.sync_strip_layout.setSpacing(3)
        self.sync_strip_layout.setContentsMargins(0, 4, 0, 4)
        self.sync_strip_layout.addStretch()
        v.addWidget(strip_wrap)

        sep = QtWidgets.QFrame()
        sep.setFrameShape(QtWidgets.QFrame.HLine)
        sep.setStyleSheet("color:#2a2d3a;")
        v.addWidget(sep)

        # ---- command handshake list (one line per commandID, live-updated) ----
        cmd_title = QtWidgets.QLabel("Command handshakes (PREPARE -> COMMIT -> APPLIED)")
        cmd_title.setStyleSheet("color:#8a8fa3; font-size:11px; font-weight:bold;")
        v.addWidget(cmd_title)

        cmd_list = QtWidgets.QListWidget()
        cmd_list.setStyleSheet(
            "QListWidget { background:#161821; color:#e0e0e0; font-family: Consolas, monospace; font-size:11px; border:none; }")
        v.addWidget(cmd_list)
        self.cmd_list = cmd_list
        return wrap

    def _build_ai_panel(self):
        wrap = QtWidgets.QGroupBox("AI Decision Engine -- Per-Window Output")
        wrap.setStyleSheet("QGroupBox { color:#e0e0e0; font-weight:bold; }")
        v = QtWidgets.QVBoxLayout(wrap)

        # ---- current verdict ----
        self.lbl_ai_verdict = QtWidgets.QLabel("Waiting for first inference window...")
        self.lbl_ai_verdict.setStyleSheet("font-weight:bold; font-size:14px; color:#8a8fa3;")
        v.addWidget(self.lbl_ai_verdict)
        self.lbl_ai_action = QtWidgets.QLabel("")
        self.lbl_ai_action.setStyleSheet("color:#8a8fa3; font-size:11px;")
        v.addWidget(self.lbl_ai_action)

        # ---- current window's feature vector, in plain language ----
        self.lbl_ai_window = QtWidgets.QLabel(
            "RSSI: -- | SNR: -- | CFO: -- | PLR: -- | CRC fails: -- | SF/CR: --/--")
        self.lbl_ai_window.setStyleSheet("color:#c0c4d0; font-size:11px;")
        self.lbl_ai_window.setWordWrap(True)
        v.addWidget(self.lbl_ai_window)

        sep = QtWidgets.QFrame()
        sep.setFrameShape(QtWidgets.QFrame.HLine)
        sep.setStyleSheet("color:#2a2d3a;")
        v.addWidget(sep)

        hist_title = QtWidgets.QLabel("Decision history (most recent first)")
        hist_title.setStyleSheet("color:#8a8fa3; font-size:11px; font-weight:bold;")
        v.addWidget(hist_title)

        ai_list = QtWidgets.QListWidget()
        ai_list.setStyleSheet(
            "QListWidget { background:#161821; color:#e0e0e0; font-family: Consolas, monospace; font-size:11px; border:none; }")
        v.addWidget(ai_list)
        self.ai_list = ai_list
        return wrap

    def _build_event_log(self):
        wrap = QtWidgets.QGroupBox("Activity Log -- every control transition, CRC/jammer event, raw line")
        wrap.setStyleSheet("QGroupBox { color:#e0e0e0; font-weight:bold; }")
        v = QtWidgets.QVBoxLayout(wrap)
        lst = QtWidgets.QListWidget()
        lst.setStyleSheet("QListWidget { background:#161821; color:#e0e0e0; font-family: Consolas, monospace; font-size:11px;}")
        v.addWidget(lst)
        self.event_log = lst
        return wrap

    # ------------------------------------------------------------ log helper
    def log(self, text, color="#c0c4d0"):
        item = QtWidgets.QListWidgetItem(f"[{time.time() - self.t0:7.2f}s] {text}")
        item.setForeground(QtGui.QColor(color))
        self.event_log.insertItem(0, item)
        while self.event_log.count() > LOG_MAX:
            self.event_log.takeItem(self.event_log.count() - 1)

    # ------------------------------------------------------------ main loop
    def drain_queue(self):
        drained = 0
        while drained < 500:
            try:
                ev = self.q.get_nowait()
            except queue.Empty:
                break
            drained += 1
            self._handle_event(ev)

        self._refresh_plots()
        self._refresh_status_bar()

    def _handle_event(self, ev):
        et = ev["type"]
        t = time.time() - self.t0

        if et == "status":
            self.stats["connected"] = ev.get("connected", False)
            self.stats["port"] = ev.get("port", "-")
            if ev.get("connected"):
                self.log(f"Connected to {ev.get('port')}", "#2ecc71")
            else:
                self.log(f"Disconnected ({ev.get('error', 'unknown')}) -- retrying...", "#e74c3c")

        elif et == "packet":
            self.stats["total_packets"] += 1
            self.rx_t.append(t)
            self.rx_rssi.append(ev["rssi"])
            self.rx_snr.append(ev["snr"])
            self._add_packet_row(t, ev)

        elif et == "crc_fail":
            self.stats["crc_fail"] += 1
            self.log("CRC failure on data plane", "#e67e22")

        elif et == "jammer":
            self.stats["jammer"] += 1
            self.log(f"Jammer/foreign packet captured  RSSI={ev['rssi']:.1f}  SNR={ev['snr']:.1f}", "#e74c3c")

        elif et == "ctrl":
            self.ctrl_events.append({"t": t, **ev})
            state = ev["state"]
            cid = ev["cid"]
            self.stats["freq"] = ev.get("freq", self.stats["freq"])
            self.stats["sf"] = ev.get("sf", self.stats["sf"])
            self.stats["cr"] = ev.get("cr", self.stats["cr"])
            color = STATE_COLOR.get(state, "#c0c4d0")
            self.log(f"CTRL cid={cid}  {state}  freq={ev.get('freq')} sf={ev.get('sf')} cr={ev.get('cr')}", color)

            entry = self.cmd_summary.get(cid)
            if entry is None:
                entry = {"states": [], "t0": t, "freq": ev.get("freq"),
                          "sf": ev.get("sf"), "cr": ev.get("cr")}
                self.cmd_summary[cid] = entry
                self.cmd_order.append(cid)
            entry["states"].append((t, state))
            if ev.get("freq") not in (None, "0"):
                entry["freq"] = ev.get("freq")
                entry["sf"] = ev.get("sf")
                entry["cr"] = ev.get("cr")
            while len(self.cmd_order) > CMD_HISTORY:
                stale = self.cmd_order.popleft()
                self.cmd_summary.pop(stale, None)
            self.cmd_dirty = True

        elif et == "window":
            self.stats["plr"] = ev.get("plr", 0.0) * 100.0
            self.last_window = ev
            self.ai_dirty = True   # feature readout updates even before next prediction

        elif et == "ai":
            prediction = ev["prediction"]
            label, action, color = AI_LABELS.get(
                prediction, (f"Unknown class {prediction}", "-", "#c0c4d0"))
            entry = {"t": t, "prediction": prediction, "label": label,
                       "action": action, "color": color, "window": self.last_window}
            self.ai_history.appendleft(entry)
            self.ai_dirty = True
            self.log(f"AI window verdict: {label} -> {action}", color)

        elif et == "sync":
            status = ev["status"]
            self.sync_history.append({"t": t, "status": status, "detail": ev.get("detail", {})})
            self.sync_state["last_status"] = status
            self.sync_state["last_t"] = t
            if status == "PING_SENT":
                self.sync_state["last_ping_t"] = t
            elif status == "MATCH":
                self.sync_state["mismatch_streak"] = 0
            elif status == "MISMATCH":
                self.sync_state["mismatch_streak"] += 1
            elif status == "DESYNC_FIXED":
                self.sync_state["mismatch_streak"] = 0
            self.sync_dirty = True
            d = ev.get("detail", {})
            if status == "MISMATCH" and d:
                self.log(f"SYNC mismatch  RX={d.get('rx_freq')}/{d.get('rx_sf')}/{d.get('rx_cr')}"
                          f"  TX={d.get('tx_freq')}/{d.get('tx_sf')}/{d.get('tx_cr')}",
                          SYNC_STATUS_COLOR.get(status))
            elif status not in ("PING_SENT",):
                self.log(f"SYNC {SYNC_STATUS_LABEL.get(status, status)}", SYNC_STATUS_COLOR.get(status, "#c0c4d0"))

        elif et == "raw":
            text = ev["text"]
            color = "#5a5f70"
            if text.startswith("[SYNC]"):
                color = "#e67e22"
            elif text.startswith("[AI]"):
                color = "#3498db"
            self.log(f"[RAW] {text}", color)

    def _add_packet_row(self, t, ev):
        table = self.packet_table
        table.insertRow(0)
        vals = [f"{t:.2f}", str(ev["seq"]), f"{ev['rssi']:.1f}",
                 f"{ev['snr']:.1f}", f"{ev['cfo']:.1f}", str(ev["sensor"])]
        for col, v in enumerate(vals):
            item = QtWidgets.QTableWidgetItem(v)
            if col == 2:  # RSSI column drives the colour cue
                item.setForeground(link_quality_color(ev["rssi"]))
            table.setItem(0, col, item)
        while table.rowCount() > TABLE_ROWS:
            table.removeRow(table.rowCount() - 1)

    # ------------------------------------------------------------ plot refresh
    def _refresh_plots(self):
        if self.rx_t:
            t = list(self.rx_t)
            self.curve_rssi.setData(t, list(self.rx_rssi))
            self.curve_snr.setData(t, list(self.rx_snr))
            self.plot_data.setXRange(max(0, t[-1] - 60), t[-1] + 1)

        # cheap label refresh every tick, so the "next PING in" countdown
        # keeps ticking even between discrete sync events
        self._refresh_sync_labels()

        if self.sync_dirty:
            self._refresh_sync_strip()
            self.sync_dirty = False

        if self.cmd_dirty:
            self._refresh_cmd_list()
            self.cmd_dirty = False

        if self.ai_dirty:
            self._refresh_ai_panel()
            self.ai_dirty = False

    def _refresh_sync_labels(self):
        now = time.time() - self.t0
        st = self.sync_state
        status = st["last_status"]

        if status is None:
            self.lbl_sync_status.setText("No PING sent yet")
            self.lbl_sync_status.setStyleSheet("font-weight:bold; font-size:14px; color:#8a8fa3;")
        else:
            self.lbl_sync_status.setText(SYNC_STATUS_LABEL.get(status, status))
            self.lbl_sync_status.setStyleSheet(
                f"font-weight:bold; font-size:14px; color:{SYNC_STATUS_COLOR.get(status, '#c0c4d0')};")

        last_ping = st["last_ping_t"]
        if last_ping is not None:
            next_in = max(0.0, SYNC_INTERVAL_S - (now - last_ping))
            last_ping_str = f"{now - last_ping:.1f}s ago"
            next_in_str = f"~{next_in:.1f}s"
        else:
            last_ping_str = "--"
            next_in_str = "--"
        streak = st["mismatch_streak"]
        streak_color = "#e74c3c" if streak else "#8a8fa3"
        self.lbl_sync_meta.setText(
            f"last PING: {last_ping_str}  |  next PING in: {next_in_str}  |  mismatch streak: {streak}")
        self.lbl_sync_meta.setStyleSheet(f"color:{streak_color}; font-size:11px;")

    def _refresh_sync_strip(self):
        # rebuild the bead strip (cheap -- capped at SYNC_HISTORY items)
        layout = self.sync_strip_layout
        while layout.count():
            item = layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        for e in self.sync_history:
            bead = QtWidgets.QLabel()
            bead.setFixedSize(12, 12)
            bead.setToolTip(f"[{e['t']:.1f}s] {SYNC_STATUS_LABEL.get(e['status'], e['status'])}")
            color = SYNC_STATUS_COLOR.get(e["status"], "#3a3d4a")
            radius = 6 if e["status"] != "PING_SENT" else 2
            bead.setStyleSheet(
                f"background:{color}; border-radius:{radius}px;" if e["status"] != "PING_SENT"
                else f"background:transparent; border:2px solid {color}; border-radius:6px;")
            layout.addWidget(bead)
        layout.addStretch()

    def _refresh_cmd_list(self):
        self.cmd_list.clear()
        for cid in reversed(self.cmd_order):
            entry = self.cmd_summary.get(cid)
            if not entry:
                continue
            states = entry["states"]
            chain = " -> ".join(s for _, s in states)
            last_state = states[-1][1]
            elapsed = states[-1][0] - states[0][0]
            cfg = f"{entry.get('freq')}kHz / SF{entry.get('sf')} / CR{entry.get('cr')}"
            text = f"#{cid}  {chain}   ({cfg})   Δt={elapsed:.1f}s"
            item = QtWidgets.QListWidgetItem(text)
            item.setForeground(QtGui.QColor(STATE_COLOR.get(last_state, "#c0c4d0")))
            self.cmd_list.addItem(item)

    def _refresh_ai_panel(self):
        w = self.last_window
        if w is not None:
            self.lbl_ai_window.setText(
                f"RSSI: {w['meanRSSI']:.1f} dBm (var {w['varRSSI']:.2f})  |  "
                f"SNR: {w['meanSNR']:.1f} dB (var {w['varSNR']:.2f})  |  "
                f"CFO: {w['cfo']:.1f} Hz  |  PLR: {w['plr']*100:.1f}%  |  "
                f"CRC fails: {w['crc']}  |  SF/CR: {w['sf']}/{w['cr']}")

        if self.ai_history:
            latest = self.ai_history[0]
            self.lbl_ai_verdict.setText(f"[AI] {latest['label']}")
            self.lbl_ai_verdict.setStyleSheet(
                f"font-weight:bold; font-size:14px; color:{latest['color']};")
            self.lbl_ai_action.setText(latest["action"])
            self.lbl_ai_action.setStyleSheet(f"color:{latest['color']}; font-size:11px;")

        self.ai_list.clear()
        for e in self.ai_history:
            text = f"[{e['t']:7.2f}s]  {e['label']:<18}  {e['action']}"
            item = QtWidgets.QListWidgetItem(text)
            item.setForeground(QtGui.QColor(e["color"]))
            self.ai_list.addItem(item)

    def _refresh_status_bar(self):
        s = self.stats
        if s["connected"]:
            self.lbl_status.setText("\u25CF CONNECTED")
            self.lbl_status.setStyleSheet("font-weight:bold; font-size:15px; color:#2ecc71;")
        else:
            self.lbl_status.setText("\u25CF DISCONNECTED")
            self.lbl_status.setStyleSheet("font-weight:bold; font-size:15px; color:#e74c3c;")
        self.lbl_port.setText(str(s["port"]))
        self.lbl_uptime.setText(f"{time.time() - self.t0:.0f}s")
        self.lbl_packets.setText(str(s["total_packets"]))
        self.lbl_crc.setText(str(s["crc_fail"]))
        self.lbl_jammer.setText(str(s["jammer"]))
        self.lbl_plr.setText(f"{s['plr']:.1f}%")
        self.lbl_cfg.setText(f"{s['freq']} kHz / SF{s['sf']} / CR{s['cr']}")

    def closeEvent(self, event):
        self.stop_event.set()
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = Dashboard()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()