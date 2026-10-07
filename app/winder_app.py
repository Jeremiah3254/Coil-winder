"""winder_app.py — control app for the coil winder (runs on a laptop, talks to the Mega over USB).

Set the coil recipe, home/start/pause/stop the machine, watch progress, and log every run:
  runs/<timestamp>_run.csv   progress samples (4 per second)
  runs/summary.csv           one row per finished or stopped coil

Run:  pip install -r requirements.txt   then   python winder_app.py
"""
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports
from PySide6 import QtCore, QtGui, QtWidgets

# ---- Tunables ----
BAUD = 115200
READ_TIMEOUT_S = 0.1
JOG_STEP_MM = 1.0
LOG_DIR = Path(__file__).resolve().parent / "runs"
TELEMETRY_FIELDS = ["pc_time", "ms", "state", "fault", "turns", "target", "layer", "layers", "x", "rpm"]
SUMMARY_FIELDS = ["finished_at", "completed", "target", "turns", "layers", "dur_ms",
                  "pitch", "width", "offset", "rpm", "log_file"]
COLOR_PANEL, COLOR_SURFACE, COLOR_INK, COLOR_MUTED = "#E9EEEC", "#F6F8F7", "#1C2B2D", "#5B6B6A"
COLOR_ACCENT, COLOR_OK, COLOR_FAULT, COLOR_GRID = "#B5602C", "#2E6A4F", "#B42318", "#C8D2CE"
FONT_FAMILIES = ["Bahnschrift", "Segoe UI", "Helvetica Neue", "Arial"]

BUSY = {"HOMING", "POSITIONING", "WINDING", "PAUSING", "STOPPING"}
STATE_COLORS = {"WINDING": COLOR_OK, "POSITIONING": COLOR_OK, "READY": COLOR_OK, "DONE": COLOR_OK,
                "PAUSING": COLOR_ACCENT, "PAUSED": COLOR_ACCENT, "FAULT": COLOR_FAULT, "E-STOP": COLOR_FAULT}


class Link(QtCore.QThread):
    """Reads JSON lines from the Mega on a background thread."""
    message = QtCore.Signal(dict)
    lost = QtCore.Signal(str)

    def __init__(self, port_name):
        super().__init__()
        self.port_name = port_name
        self.ser = None
        self._stop = False

    def open(self):
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = self.port_name, BAUD, READ_TIMEOUT_S
        ser.open()   # opening the port resets the Mega; it boots in about 2 s and says hello
        self.ser = ser

    def run(self):
        pending = b""
        while not self._stop:
            try:
                chunk = self.ser.readline()
            except Exception as exc:  # cable unplugged
                self.lost.emit(str(exc))
                return
            if not chunk:
                continue
            if not chunk.endswith(b"\n"):   # partial line after a timeout
                pending += chunk
                continue
            line, pending = pending + chunk, b""
            try:
                obj = json.loads(line.decode(errors="ignore"))
            except ValueError:
                continue
            if isinstance(obj, dict):
                self.message.emit(obj)

    def send(self, obj):
        self.ser.write((json.dumps(obj) + "\n").encode())

    def close(self):
        self._stop = True
        self.wait(1500)
        if self.ser is not None:
            self.ser.close()


class RunLogger:
    def __init__(self):
        self.file = self.writer = self.path = None

    @property
    def active(self):
        return self.file is not None

    def start(self):
        LOG_DIR.mkdir(exist_ok=True)
        self.path = LOG_DIR / f"{datetime.now():%Y%m%d_%H%M%S}_run.csv"
        self.file = open(self.path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=TELEMETRY_FIELDS, extrasaction="ignore")
        self.writer.writeheader()

    def row(self, tel):
        if self.active:
            self.writer.writerow(dict(tel, pc_time=f"{time.time():.3f}"))

    def finish(self, summary, recipe):
        LOG_DIR.mkdir(exist_ok=True)
        path = LOG_DIR / "summary.csv"
        new = not path.exists()
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
            if new:
                w.writeheader()
            w.writerow(dict(summary, **recipe, finished_at=datetime.now().isoformat(timespec="seconds"),
                            log_file=self.path.name if self.path else ""))
        if self.file:
            self.file.close()
        self.file = self.writer = None


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Coil Winder")
        self.link = None
        self.state = "DISCONNECTED"
        self.logger = RunLogger()
        self.run_recipe = {}
        self._build_ui()
        self._refresh_ports()
        self._apply_state()

    # ---------------- layout ----------------
    def _build_ui(self):
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        outer = QtWidgets.QHBoxLayout(root)
        outer.setContentsMargins(16, 16, 16, 16)
        outer.setSpacing(16)
        side = QtWidgets.QVBoxLayout()
        side.setSpacing(12)
        outer.addLayout(side, 0)
        side.addWidget(self._connection_box())
        side.addWidget(self._recipe_box())
        side.addWidget(self._machine_box())
        side.addStretch(1)

        main = QtWidgets.QVBoxLayout()
        main.setSpacing(10)
        outer.addLayout(main, 1)
        head = QtWidgets.QHBoxLayout()
        self.state_label = QtWidgets.QLabel("Not connected")
        self.state_label.setObjectName("statePill")
        self.msg_label = QtWidgets.QLabel("")
        self.msg_label.setObjectName("muted")
        head.addWidget(self.state_label, 0)
        head.addWidget(self.msg_label, 1)
        main.addLayout(head)

        counter = QtWidgets.QHBoxLayout()
        self.turns_label = QtWidgets.QLabel("0.0")
        self.turns_label.setObjectName("counter")
        self.target_label = QtWidgets.QLabel("of 0 turns")
        self.target_label.setObjectName("counterUnit")
        counter.addWidget(self.turns_label, 0, QtCore.Qt.AlignBottom)
        counter.addWidget(self.target_label, 0, QtCore.Qt.AlignBottom)
        counter.addStretch(1)
        main.addLayout(counter)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(8)
        main.addWidget(self.progress)

        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(32)
        self.readout = {}
        for col, (key, label) in enumerate([("layer", "Layer"), ("x", "Guide position"), ("rpm", "Spindle speed")]):
            name = QtWidgets.QLabel(label)
            name.setObjectName("muted")
            value = QtWidgets.QLabel("–")
            value.setObjectName("readout")
            grid.addWidget(name, 0, col)
            grid.addWidget(value, 1, col)
            self.readout[key] = value
        main.addLayout(grid)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        main.addWidget(self.log, 1)

    def _group(self, title):
        box = QtWidgets.QGroupBox(title)
        box.setFixedWidth(320)
        return box

    def _connection_box(self):
        box = self._group("Connection")
        lay = QtWidgets.QGridLayout(box)
        self.port_combo = QtWidgets.QComboBox()
        refresh = QtWidgets.QPushButton("Refresh")
        refresh.clicked.connect(self._refresh_ports)
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.setObjectName("primary")
        self.connect_btn.clicked.connect(self._toggle_connection)
        lay.addWidget(self.port_combo, 0, 0, 1, 2)
        lay.addWidget(refresh, 1, 0)
        lay.addWidget(self.connect_btn, 1, 1)
        return box

    def _spin(self, lo, hi, step, decimals, suffix):
        w = QtWidgets.QSpinBox() if decimals == 0 else QtWidgets.QDoubleSpinBox()
        if decimals:
            w.setDecimals(decimals)
        w.setRange(lo, hi)
        w.setSingleStep(step)
        w.setSuffix(suffix)
        return w

    def _recipe_box(self):
        box = self._group("Coil recipe")
        form = QtWidgets.QFormLayout(box)
        self.fields = {
            "turns": self._spin(1, 100000, 10, 0, " turns"),
            "pitch": self._spin(0.02, 5.0, 0.01, 3, " mm"),
            "width": self._spin(0.5, 150.0, 0.5, 1, " mm"),
            "offset": self._spin(2.5, 150.0, 0.5, 1, " mm"),
            "rpm": self._spin(10, 300, 10, 0, " RPM"),
        }
        labels = {"turns": "Turns", "pitch": "Wire diameter", "width": "Bobbin width",
                  "offset": "First turn at", "rpm": "Speed"}
        tips = {"pitch": "Measure the wire's outside diameter with calipers.",
                "offset": "Guide position of the first turn, measured from the home switch."}
        for key, w in self.fields.items():
            w.setToolTip(tips.get(key, ""))
            form.addRow(labels[key], w)
        self.save_btn = QtWidgets.QPushButton("Save to machine")
        self.save_btn.clicked.connect(self._save_recipe)
        form.addRow(self.save_btn)
        return box

    def _machine_box(self):
        box = self._group("Machine")
        lay = QtWidgets.QGridLayout(box)
        self.btn = {}
        specs = [("home", "Home axis", 0, 0, ""), ("start", "Start winding", 0, 1, "primary"),
                 ("pause", "Pause", 1, 0, ""), ("resume", "Resume", 1, 1, ""),
                 ("stop", "Stop", 2, 0, "danger"), ("reset", "Clear fault", 2, 1, ""),
                 ("jog-", f"Jog −{JOG_STEP_MM:g} mm", 3, 0, ""), ("jog+", f"Jog +{JOG_STEP_MM:g} mm", 3, 1, "")]
        for key, text, r, c, role in specs:
            b = QtWidgets.QPushButton(text)
            if role:
                b.setObjectName(role)
            lay.addWidget(b, r, c)
            self.btn[key] = b
        self.btn["home"].clicked.connect(lambda: self._send({"cmd": "home"}))
        self.btn["start"].clicked.connect(lambda: self._send(dict(self._recipe(), cmd="start")))
        self.btn["pause"].clicked.connect(lambda: self._send({"cmd": "pause"}))
        self.btn["resume"].clicked.connect(lambda: self._send({"cmd": "resume"}))
        self.btn["stop"].clicked.connect(lambda: self._send({"cmd": "stop"}))
        self.btn["reset"].clicked.connect(lambda: self._send({"cmd": "reset"}))
        self.btn["jog-"].clicked.connect(lambda: self._send({"cmd": "jog", "mm": -JOG_STEP_MM}))
        self.btn["jog+"].clicked.connect(lambda: self._send({"cmd": "jog", "mm": JOG_STEP_MM}))
        return box

    # ---------------- connection ----------------
    def _refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems([p.device for p in serial.tools.list_ports.comports()])
        if current:
            self.port_combo.setCurrentText(current)

    def _toggle_connection(self):
        if self.link:
            self._disconnect("Disconnected")
            return
        name = self.port_combo.currentText()
        if not name:
            self._log("No port selected. Plug in the Mega and click Refresh.")
            return
        link = Link(name)
        try:
            link.open()
        except Exception as exc:
            self._log(f"Could not open {name}: {exc} (is the Arduino IDE serial monitor still open?)")
            return
        link.message.connect(self._on_message)
        link.lost.connect(lambda why: self._disconnect(f"Connection lost: {why}"))
        link.start()
        self.link = link
        self.state = "CONNECTING"
        self.connect_btn.setText("Disconnect")
        self._log(f"Connected to {name}. Waiting for the Mega to start...")
        self._apply_state()

    def _disconnect(self, why):
        if self.link:
            self.link.close()
        self.link = None
        self.state = "DISCONNECTED"
        self.connect_btn.setText("Connect")
        self.state_label.setText("Not connected")
        self._log(why)
        self._apply_state()

    def _send(self, obj):
        if self.link:
            try:
                self.link.send(obj)
            except Exception as exc:
                self._disconnect(f"Send failed: {exc}")

    def _recipe(self):
        return {k: w.value() for k, w in self.fields.items()}

    def _save_recipe(self):
        self._send(dict(self._recipe(), cmd="set"))
        self._send({"cmd": "save"})

    # ---------------- messages from the Mega ----------------
    def _on_message(self, m):
        kind = m.get("type")
        if kind == "tel":
            self._on_telemetry(m)
        elif kind == "state":
            self._log(f"{m.get('state')}: {m.get('msg', '')}")
            if m.get("state") == "POSITIONING" and not self.logger.active:
                self.logger.start()
                self.run_recipe = self._recipe()
                self.statusBar().showMessage(f"Logging to {self.logger.path}")
        elif kind == "ack":
            if not m.get("ok"):
                self._log(f"{m.get('cmd')} refused: {m.get('msg', '')}")
            elif m.get("cmd") == "save":
                self._log(m.get("msg", ""))
        elif kind == "recipe":
            for k, w in self.fields.items():
                if k in m:
                    w.setValue(m[k])
        elif kind == "done":
            verdict = "Coil complete" if m.get("completed") else "Run stopped early"
            self._log(f"{verdict}: {m.get('turns')} of {m.get('target')} turns, {m.get('layers')} layers, "
                      f"{m.get('dur_ms', 0) / 1000:.0f} s")
            self.logger.finish(m, self.run_recipe or self._recipe())
            self.statusBar().showMessage(f"Summary saved to {LOG_DIR / 'summary.csv'}")
        elif kind == "hello":
            self._log(f"Machine firmware {m.get('fw')} ready")

    def _on_telemetry(self, m):
        prev = self.state
        self.state = m.get("state", self.state)
        if self.state in ("WINDING", "POSITIONING") and not self.logger.active:
            self.logger.start()
            self.run_recipe = self._recipe()
        self.logger.row(m)
        target = max(1, int(m.get("target", 1)))
        turns = float(m.get("turns", 0.0))
        self.turns_label.setText(f"{turns:,.1f}")
        self.target_label.setText(f"of {target:,} turns")
        self.progress.setValue(int(min(100, 100 * turns / target)))
        layers = m.get("layers", 0)
        self.readout["layer"].setText(f"{m.get('layer', 0)} of {layers}" if layers else "–")
        self.readout["x"].setText(f"{m.get('x', 0):.2f} mm")
        self.readout["rpm"].setText(f"{m.get('rpm', 0):.0f} RPM")
        fault = m.get("fault", "NONE")
        self.state_label.setText((self.state if fault == "NONE" else f"{self.state}  {fault}").replace("_", " ").title())
        if not m.get("homed", True) and self.state == "IDLE":
            self.msg_label.setText("Click Home axis first")
        if prev != self.state:
            self._apply_state()

    # ---------------- enable/disable ----------------
    def _apply_state(self):
        s = self.state
        connected = self.link is not None
        idle = connected and s in ("IDLE", "READY", "DONE")
        enable = {"home": idle, "start": connected and s in ("READY", "DONE"),
                  "pause": connected and s in ("WINDING", "POSITIONING"), "resume": connected and s == "PAUSED",
                  "stop": connected and (s in BUSY or s == "PAUSED"), "reset": connected and s in ("FAULT", "E-STOP"),
                  "jog-": connected and s in ("READY", "DONE"), "jog+": connected and s in ("READY", "DONE")}
        for k, b in self.btn.items():
            b.setEnabled(enable[k])
        for w in self.fields.values():
            w.setEnabled(not connected or (s not in BUSY and s != "PAUSED"))
        self.save_btn.setEnabled(idle)
        self.port_combo.setEnabled(not connected)
        self.state_label.setStyleSheet(f"background:{STATE_COLORS.get(s, COLOR_MUTED)}; color:white;")
        self.turns_label.setStyleSheet(f"color:{COLOR_FAULT if s in ('FAULT', 'E-STOP') else COLOR_INK};")
        self.msg_label.setText("" if connected else "Plug in the Mega, pick its COM port, and click Connect.")

    def _log(self, text):
        self.log.appendPlainText(f"{datetime.now():%H:%M:%S}  {text}")

    def closeEvent(self, event):
        if self.link:
            self.link.close()
        super().closeEvent(event)


def stylesheet():
    return f"""
    QWidget {{ background:{COLOR_PANEL}; color:{COLOR_INK}; font-size:13px; }}
    QGroupBox {{ background:{COLOR_SURFACE}; border:1px solid {COLOR_GRID}; border-radius:6px;
                 margin-top:14px; padding:10px 10px 8px 10px; font-weight:600; }}
    QGroupBox::title {{ subcontrol-origin:margin; left:10px; padding:0 4px; color:{COLOR_MUTED}; }}
    QGroupBox QWidget {{ background:{COLOR_SURFACE}; font-weight:400; }}
    QPushButton {{ background:{COLOR_SURFACE}; border:1px solid {COLOR_INK}; border-radius:4px; padding:6px 10px; }}
    QPushButton:hover {{ background:#DDE5E2; }}
    QPushButton:disabled {{ color:#9AA7A5; border-color:{COLOR_GRID}; }}
    QPushButton#primary {{ background:{COLOR_ACCENT}; color:white; border-color:{COLOR_ACCENT}; font-weight:600; }}
    QPushButton#primary:disabled {{ background:#D9B9A6; border-color:#D9B9A6; }}
    QPushButton#danger {{ color:{COLOR_FAULT}; border-color:{COLOR_FAULT}; font-weight:600; }}
    QSpinBox, QDoubleSpinBox, QComboBox {{ background:white; border:1px solid {COLOR_GRID}; border-radius:4px; padding:3px 6px; }}
    QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {{ background:{COLOR_PANEL}; color:{COLOR_MUTED}; }}
    QLabel#statePill {{ border-radius:4px; padding:4px 12px; font-weight:700; font-size:14px; }}
    QLabel#muted {{ color:{COLOR_MUTED}; }}
    QLabel#counter {{ font-size:72px; font-weight:600; }}
    QLabel#counterUnit {{ font-size:20px; color:{COLOR_MUTED}; padding-bottom:14px; }}
    QLabel#readout {{ font-size:20px; font-weight:600; }}
    QProgressBar {{ background:{COLOR_GRID}; border:none; border-radius:4px; }}
    QProgressBar::chunk {{ background:{COLOR_ACCENT}; border-radius:4px; }}
    QPlainTextEdit {{ background:{COLOR_SURFACE}; border:1px solid {COLOR_GRID}; border-radius:6px; color:{COLOR_MUTED}; }}
    """


def main():
    app = QtWidgets.QApplication(sys.argv)
    font = QtGui.QFont()
    font.setFamilies(FONT_FAMILIES)
    app.setFont(font)
    app.setStyleSheet(stylesheet())
    win = MainWindow()
    win.resize(1100, 720)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
