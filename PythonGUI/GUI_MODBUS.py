import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports


DEFAULT_PORT = "COM8"
DEFAULT_BAUD = 115200
DEFAULT_UNIT_ID = 1
POLL_INTERVAL_S = 0.25
REQUEST_TIMEOUT_S = 1.0
OUTPUT_FILE = Path("modbus_duomenys.csv")


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def s16(value: int) -> int:
    return struct.unpack(">h", struct.pack(">H", value & 0xFFFF))[0]


def u32_from_regs(low: int, high: int) -> int:
    return ((high & 0xFFFF) << 16) | (low & 0xFFFF)


def s32_from_regs(low: int, high: int) -> int:
    value = u32_from_regs(low, high)
    if value & 0x80000000:
        value -= 0x100000000
    return value


def scaled_s16(value: int, scale: float) -> float:
    return s16(value) / scale


def parse_float(text: str) -> float:
    return float(text.strip().replace(",", "."))


@dataclass
class ModbusSnapshot:
    timestamp: str
    map_version: int
    seq: int
    uptime_ms: int
    pt1000_c: float
    lm35_c: float
    lid_mv: int
    pt_raw: int
    lm_raw: int
    pwm_pt_pct: float
    pwm_lm_pct: float
    target_pt_c: float
    target_lm_c: float
    ina0_ok: bool
    ina0_voltage_v: float
    ina0_current_a: float
    ina0_power_w: float
    ina1_ok: bool
    ina1_voltage_v: float
    ina1_current_a: float
    ina1_power_w: float
    lid_closed: bool
    lse_ready: bool


class ModbusRtuClient:
    def __init__(self, port: str, baud: int, unit_id: int):
        self.unit_id = unit_id
        self.serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,
            timeout=REQUEST_TIMEOUT_S,
            write_timeout=REQUEST_TIMEOUT_S,
        )
        self.lock = threading.Lock()
        self.last_tx = b""
        self.last_rx = b""

    def close(self) -> None:
        self.serial.close()

    def _request(self, function: int, payload: bytes, expected_len: int) -> bytes:
        frame = bytes([self.unit_id, function]) + payload
        crc = crc16_modbus(frame)
        frame += bytes([crc & 0xFF, crc >> 8])
        self.last_tx = frame
        self.last_rx = b""

        with self.lock:
            self.serial.reset_input_buffer()
            self.serial.write(frame)
            self.serial.flush()
            response = self.serial.read(expected_len)
            self.last_rx = response

        if len(response) != expected_len:
            raise TimeoutError(f"Timeout: expected {expected_len} bytes, got {len(response)}")

        rx_crc = response[-2] | (response[-1] << 8)
        calc_crc = crc16_modbus(response[:-2])
        if rx_crc != calc_crc:
            raise ValueError(f"CRC mismatch: rx=0x{rx_crc:04X} calc=0x{calc_crc:04X}")

        if response[0] != self.unit_id:
            raise ValueError(f"Unexpected unit id {response[0]}")

        if response[1] & 0x80:
            raise ValueError(f"Modbus exception {response[2]}")

        if response[1] != function:
            raise ValueError(f"Unexpected function {response[1]}")

        return response

    def read_registers(self, function: int, address: int, quantity: int) -> list[int]:
        payload = struct.pack(">HH", address, quantity)
        expected_len = 5 + quantity * 2
        response = self._request(function, payload, expected_len)
        byte_count = response[2]
        if byte_count != quantity * 2:
            raise ValueError(f"Unexpected byte count {byte_count}")
        return [
            (response[3 + i * 2] << 8) | response[4 + i * 2]
            for i in range(quantity)
        ]

    def read_discrete_inputs(self, address: int, quantity: int) -> list[bool]:
        payload = struct.pack(">HH", address, quantity)
        byte_count = (quantity + 7) // 8
        response = self._request(2, payload, 5 + byte_count)
        if response[2] != byte_count:
            raise ValueError(f"Unexpected byte count {response[2]}")
        states = []
        for idx in range(quantity):
            states.append(bool(response[3 + idx // 8] & (1 << (idx % 8))))
        return states

    def write_holding_register(self, address: int, value: int) -> None:
        payload = struct.pack(">HH", address, value & 0xFFFF)
        response = self._request(6, payload, 8)
        if response[2:6] != payload:
            raise ValueError("Write echo mismatch")

    def read_snapshot(self) -> ModbusSnapshot:
        discrete = self.read_discrete_inputs(0, 4)
        regs_0 = self.read_registers(4, 0, 5)
        regs_10 = self.read_registers(4, 10, 7)
        regs_20 = self.read_registers(4, 20, 4)
        regs_30 = self.read_registers(4, 30, 6)
        regs_40 = self.read_registers(4, 40, 6)

        return ModbusSnapshot(
            timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            map_version=regs_0[0],
            seq=u32_from_regs(regs_0[1], regs_0[2]),
            uptime_ms=u32_from_regs(regs_0[3], regs_0[4]),
            pt1000_c=scaled_s16(regs_10[0], 100.0),
            lm35_c=scaled_s16(regs_10[1], 100.0),
            lid_mv=s16(regs_10[2]),
            pt_raw=s32_from_regs(regs_10[3], regs_10[4]),
            lm_raw=s32_from_regs(regs_10[5], regs_10[6]),
            pwm_pt_pct=scaled_s16(regs_20[0], 10.0),
            pwm_lm_pct=scaled_s16(regs_20[1], 10.0),
            target_pt_c=scaled_s16(regs_20[2], 100.0),
            target_lm_c=scaled_s16(regs_20[3], 100.0),
            ina0_ok=discrete[1],
            ina0_voltage_v=s32_from_regs(regs_30[0], regs_30[1]) / 1000.0,
            ina0_current_a=s32_from_regs(regs_30[2], regs_30[3]) / 1000000.0,
            ina0_power_w=s32_from_regs(regs_30[4], regs_30[5]) / 1000000.0,
            ina1_ok=discrete[2],
            ina1_voltage_v=s32_from_regs(regs_40[0], regs_40[1]) / 1000.0,
            ina1_current_a=s32_from_regs(regs_40[2], regs_40[3]) / 1000000.0,
            ina1_power_w=s32_from_regs(regs_40[4], regs_40[5]) / 1000000.0,
            lid_closed=discrete[0],
            lse_ready=discrete[3],
        )


class ModbusGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MIRI TL6 Modbus Monitor")
        self.geometry("900x620")
        self.minsize(820, 560)

        self.client = None
        self.worker = None
        self.stop_event = threading.Event()
        self.poll_enabled = False
        self.events = queue.Queue()
        self.latest_snapshot = None
        self.csv_file = None
        self.csv_writer = None

        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.unit_var = tk.StringVar(value=str(DEFAULT_UNIT_ID))
        self.status_var = tk.StringVar(value="Disconnected")
        self.raw_var = tk.StringVar(value="TX: ---   RX: ---")
        self.timeout_var = tk.StringVar(value=str(REQUEST_TIMEOUT_S))
        self.pt_set_var = tk.StringVar(value="25.20")
        self.lm_set_var = tk.StringVar(value="25.00")

        self.value_vars = {}

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._process_events)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        controls = ttk.Frame(root)
        controls.pack(fill=tk.X)

        ttk.Label(controls, text="Port").grid(row=0, column=0, sticky=tk.W)
        self.port_combo = ttk.Combobox(controls, textvariable=self.port_var, width=12)
        self.port_combo.grid(row=0, column=1, padx=(6, 14), sticky=tk.W)

        ttk.Label(controls, text="Baud").grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(controls, textvariable=self.baud_var, width=10).grid(
            row=0, column=3, padx=(6, 14), sticky=tk.W
        )

        ttk.Label(controls, text="Unit").grid(row=0, column=4, sticky=tk.W)
        ttk.Entry(controls, textvariable=self.unit_var, width=6).grid(
            row=0, column=5, padx=(6, 14), sticky=tk.W
        )

        ttk.Label(controls, text="Timeout").grid(row=0, column=6, sticky=tk.W)
        ttk.Entry(controls, textvariable=self.timeout_var, width=6).grid(
            row=0, column=7, padx=(6, 14), sticky=tk.W
        )

        ttk.Button(controls, text="Refresh", command=self._refresh_ports).grid(
            row=0, column=8, padx=(0, 8)
        )
        self.connect_button = ttk.Button(controls, text="Connect", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=9, padx=(0, 8))
        ttk.Button(controls, text="Test Read", command=self._test_read).grid(row=0, column=10)

        ttk.Label(root, textvariable=self.status_var).pack(fill=tk.X, pady=(8, 2))
        ttk.Label(root, textvariable=self.raw_var).pack(fill=tk.X, pady=(0, 10))

        setpoints = ttk.LabelFrame(root, text="Holding Registers")
        setpoints.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(setpoints, text="Target PT1000 C").grid(row=0, column=0, padx=8, pady=8)
        ttk.Entry(setpoints, textvariable=self.pt_set_var, width=10).grid(row=0, column=1)
        ttk.Button(setpoints, text="Write PT", command=lambda: self._write_setpoint(0)).grid(
            row=0, column=2, padx=(8, 20)
        )

        ttk.Label(setpoints, text="Target LM35 C").grid(row=0, column=3, padx=8, pady=8)
        ttk.Entry(setpoints, textvariable=self.lm_set_var, width=10).grid(row=0, column=4)
        ttk.Button(setpoints, text="Write LM", command=lambda: self._write_setpoint(1)).grid(
            row=0, column=5, padx=(8, 20)
        )

        body = ttk.Frame(root)
        body.pack(fill=tk.BOTH, expand=True)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)

        left = ttk.LabelFrame(body, text="Temperatures and State")
        right = ttk.LabelFrame(body, text="Power Meters")
        left.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 5))
        right.grid(row=0, column=1, sticky=tk.NSEW, padx=(5, 0))

        for parent, rows in (
            (
                left,
                [
                    ("map_version", "Map version"),
                    ("seq", "Sequence"),
                    ("uptime_ms", "Uptime ms"),
                    ("pt1000_c", "PT1000 C"),
                    ("lm35_c", "LM35 C"),
                    ("lid_mv", "Lid ADC mV"),
                    ("lid_closed", "Lid closed"),
                    ("lse_ready", "LSE ready"),
                    ("pwm_pt_pct", "PWM PT %"),
                    ("pwm_lm_pct", "PWM LM %"),
                    ("target_pt_c", "Target PT C"),
                    ("target_lm_c", "Target LM C"),
                    ("pt_raw", "PT raw"),
                    ("lm_raw", "LM raw"),
                ],
            ),
            (
                right,
                [
                    ("ina0_ok", "INA0 ok"),
                    ("ina0_voltage_v", "INA0 V"),
                    ("ina0_current_a", "INA0 A"),
                    ("ina0_power_w", "INA0 W"),
                    ("ina1_ok", "INA1 ok"),
                    ("ina1_voltage_v", "INA1 V"),
                    ("ina1_current_a", "INA1 A"),
                    ("ina1_power_w", "INA1 W"),
                ],
            ),
        ):
            parent.columnconfigure(1, weight=1)
            for row, (key, label) in enumerate(rows):
                ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, padx=8, pady=4)
                var = tk.StringVar(value="---")
                self.value_vars[key] = var
                ttk.Label(parent, textvariable=var, anchor=tk.E).grid(
                    row=row, column=1, sticky=tk.EW, padx=8, pady=4
                )

    def _refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self.client is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        try:
            port = self.port_var.get().strip()
            baud = int(self.baud_var.get())
            unit_id = int(self.unit_var.get())
            timeout = float(self.timeout_var.get())
            global REQUEST_TIMEOUT_S
            REQUEST_TIMEOUT_S = timeout
            self.client = ModbusRtuClient(port, baud, unit_id)
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))
            self.client = None
            return

        self._open_csv()
        self.connect_button.configure(text="Disconnect")
        self.status_var.set(
            f"Port open: {self.port_var.get()}  {self.baud_var.get()} 8N2  unit {self.unit_var.get()}. Press Test Read."
        )

    def _disconnect(self):
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=1.0)
            self.worker = None
        if self.client is not None:
            self.client.close()
            self.client = None
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        self.connect_button.configure(text="Connect")
        self.status_var.set("Disconnected")
        self.poll_enabled = False

    def _open_csv(self):
        self.csv_file = OUTPUT_FILE.open("w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp",
            "map_version",
            "seq",
            "uptime_ms",
            "pt1000_c",
            "lm35_c",
            "lid_mv",
            "lid_closed",
            "lse_ready",
            "pwm_pt_pct",
            "pwm_lm_pct",
            "target_pt_c",
            "target_lm_c",
            "ina0_ok",
            "ina0_voltage_v",
            "ina0_current_a",
            "ina0_power_w",
            "ina1_ok",
            "ina1_voltage_v",
            "ina1_current_a",
            "ina1_power_w",
            "pt_raw",
            "lm_raw",
        ])

    def _poll_loop(self):
        while not self.stop_event.is_set():
            try:
                snapshot = self.client.read_snapshot()
                self.events.put(("snapshot", snapshot))
            except Exception as exc:
                self.events.put(("error", str(exc)))
            time.sleep(POLL_INTERVAL_S)

    def _write_setpoint(self, index: int):
        if self.client is None:
            messagebox.showwarning("Not connected", "Connect to the Modbus device first.")
            return

        try:
            text = self.pt_set_var.get() if index == 0 else self.lm_set_var.get()
            value = int(round(parse_float(text) * 100.0))
            self.client.write_holding_register(index, value)
            self.status_var.set(f"Wrote holding register {index} = {value}")
        except Exception as exc:
            self._update_raw_trace()
            messagebox.showerror("Write failed", str(exc))

    def _test_read(self):
        if self.client is None:
            messagebox.showwarning("Not connected", "Connect to the Modbus device first.")
            return

        try:
            regs = self.client.read_registers(4, 0, 1)
            self._update_raw_trace()
            if not self.poll_enabled:
                self.poll_enabled = True
                self.stop_event.clear()
                self.worker = threading.Thread(target=self._poll_loop, daemon=True)
                self.worker.start()
            messagebox.showinfo("Test read OK", f"Input register 0 = {regs[0]}\n\nPolling started.")
        except Exception as exc:
            self._update_raw_trace()
            messagebox.showerror(
                "Test read failed",
                f"{exc}\n\n"
                "Checklist:\n"
                "- correct USB-RS485 COM port\n"
                "- firmware flashed with Modbus enabled\n"
                "- slave/unit id = 1\n"
                "- serial format = 115200 8N2\n"
                "- A/B lines not swapped\n"
                "- common GND connected\n"
                "- USART1 TX/RX/DE pins wired to the RS-485 transceiver",
            )

    def _process_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "snapshot":
                    self.latest_snapshot = payload
                    self._update_raw_trace()
                    self._show_snapshot(payload)
                    self._log_snapshot(payload)
                    self.status_var.set(
                        f"Last read {payload.timestamp}  seq={payload.seq}  unit {self.unit_var.get()}"
                    )
                elif kind == "error":
                    self._update_raw_trace()
                    self.status_var.set(f"Read error: {payload}")
        except queue.Empty:
            pass

        self.after(50, self._process_events)

    def _update_raw_trace(self):
        if self.client is None:
            self.raw_var.set("TX: ---   RX: ---")
            return

        tx = self.client.last_tx.hex(" ").upper() or "---"
        rx = self.client.last_rx.hex(" ").upper() or "---"
        self.raw_var.set(f"TX: {tx}   RX: {rx}")

    def _show_snapshot(self, snapshot: ModbusSnapshot):
        formats = {
            "pt1000_c": "{:.2f}",
            "lm35_c": "{:.2f}",
            "pwm_pt_pct": "{:.1f}",
            "pwm_lm_pct": "{:.1f}",
            "target_pt_c": "{:.2f}",
            "target_lm_c": "{:.2f}",
            "ina0_voltage_v": "{:.3f}",
            "ina0_current_a": "{:.6f}",
            "ina0_power_w": "{:.6f}",
            "ina1_voltage_v": "{:.3f}",
            "ina1_current_a": "{:.6f}",
            "ina1_power_w": "{:.6f}",
        }

        for key, var in self.value_vars.items():
            value = getattr(snapshot, key)
            if isinstance(value, bool):
                var.set("yes" if value else "no")
            elif key in formats:
                var.set(formats[key].format(value))
            else:
                var.set(str(value))

        self.pt_set_var.set(f"{snapshot.target_pt_c:.2f}")
        self.lm_set_var.set(f"{snapshot.target_lm_c:.2f}")

    def _log_snapshot(self, snapshot: ModbusSnapshot):
        if self.csv_writer is None:
            return

        self.csv_writer.writerow([
            snapshot.timestamp,
            snapshot.map_version,
            snapshot.seq,
            snapshot.uptime_ms,
            snapshot.pt1000_c,
            snapshot.lm35_c,
            snapshot.lid_mv,
            int(snapshot.lid_closed),
            int(snapshot.lse_ready),
            snapshot.pwm_pt_pct,
            snapshot.pwm_lm_pct,
            snapshot.target_pt_c,
            snapshot.target_lm_c,
            int(snapshot.ina0_ok),
            snapshot.ina0_voltage_v,
            snapshot.ina0_current_a,
            snapshot.ina0_power_w,
            int(snapshot.ina1_ok),
            snapshot.ina1_voltage_v,
            snapshot.ina1_current_a,
            snapshot.ina1_power_w,
            snapshot.pt_raw,
            snapshot.lm_raw,
        ])
        self.csv_file.flush()

    def _on_close(self):
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    app = ModbusGui()
    app.mainloop()
