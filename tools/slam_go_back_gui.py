#!/usr/bin/env python3
"""Minimal Go/Back control panel for the STM32 SLAM firmware.

The desktop app only sends text commands and displays firmware telemetry.
SLAM, A*, obstacle handling, and return-home decisions run entirely on the MCU.
"""

from __future__ import annotations

import argparse
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Optional

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 115200
LOG_LIMIT = 500

SLAM_HB_RE = re.compile(
    r"SLAM HB state=(?P<state>\S+)\s+seq=(?P<seq>\d+)\s+target=(?P<target>\d+,\d+)\s+"
    r"path_i=(?P<path_i>\d+)\s+path_len=(?P<path_len>\d+)\s+front=(?P<front>\d+)\s+rev=(?P<rev>\d+)"
)
SLAM_STATE_RE = re.compile(r"SLAM state=(?P<state>\S+)\s+reason=(?P<reason>\S+)\s+seq=(?P<seq>\d+).*")
MAP_STAT_RE = re.compile(r"MAP STAT\s+(?P<body>.*)")


class SerialLink:
    def __init__(self, rx_queue: queue.Queue[str]) -> None:
        self.rx_queue = rx_queue
        self.serial_obj = None
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None

    @property
    def connected(self) -> bool:
        return self.serial_obj is not None and self.serial_obj.is_open

    def connect(self, port: str, baud: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: python -m pip install pyserial")

        self.disconnect()
        self.stop_event.clear()
        self.serial_obj = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.serial_obj is not None:
            try:
                self.serial_obj.close()
            except Exception:
                pass
        self.serial_obj = None

    def send(self, command: str) -> None:
        if not self.connected:
            raise RuntimeError("serial port is not connected")

        payload = (command.strip() + "\r\n").encode("ascii", errors="ignore")
        self.serial_obj.write(payload)
        self.serial_obj.flush()
        self.rx_queue.put(f"[TX] {command.strip()}")

    def _read_loop(self) -> None:
        buffer = b""

        while not self.stop_event.is_set() and self.serial_obj is not None:
            try:
                data = self.serial_obj.read(1024)
            except Exception as exc:
                self.rx_queue.put(f"[ERROR] {exc}")
                break

            if not data:
                time.sleep(0.01)
                continue

            buffer += data
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                line = raw_line.decode("ascii", errors="replace").strip()
                if line:
                    self.rx_queue.put(line)


class GoBackGui(tk.Tk):
    def __init__(self, port: str, baud: int) -> None:
        super().__init__()
        self.title("STM32 SLAM Go / Back")
        self.geometry("760x520")

        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.link = SerialLink(self.rx_queue)
        self.port_var = tk.StringVar(value=port)
        self.baud_var = tk.IntVar(value=baud)
        self.connection_var = tk.StringVar(value="Disconnected")
        self.slam_var = tk.StringVar(value="SLAM --")
        self.path_var = tk.StringVar(value="Path --")
        self.map_var = tk.StringVar(value="Map --")
        self.last_var = tk.StringVar(value="")
        self.log_lines: list[str] = []

        self._build_ui()
        self.after(40, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self)
        top.pack(side=tk.TOP, fill=tk.X, padx=10, pady=8)

        ttk.Label(top, text="Port").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.port_var, width=10).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(top, text="Baud").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(top, text="Connect", command=self._connect).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Disconnect", command=self._disconnect).pack(side=tk.LEFT, padx=2)

        controls = ttk.Frame(self)
        controls.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 8))
        ttk.Button(controls, text="Go", command=lambda: self._send("SLAM")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(controls, text="Back", command=lambda: self._send("BACK")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(controls, text="Stop", command=lambda: self._send("SLAM OFF")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
        ttk.Button(controls, text="Show Map", command=lambda: self._send("SHOW MAP")).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        status = ttk.Frame(self)
        status.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(0, 8))
        for label, var in (
            ("Connection", self.connection_var),
            ("SLAM", self.slam_var),
            ("Path", self.path_var),
            ("Map", self.map_var),
            ("Last", self.last_var),
        ):
            row = ttk.Frame(status)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=12).pack(side=tk.LEFT)
            ttk.Label(row, textvariable=var).pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.log = tk.Text(self, height=18, state=tk.DISABLED)
        self.log.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

    def _connect(self) -> None:
        try:
            self.link.connect(self.port_var.get(), self.baud_var.get())
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))
            return
        self.connection_var.set(f"Connected {self.port_var.get()} @ {self.baud_var.get()}")

    def _disconnect(self) -> None:
        self.link.disconnect()
        self.connection_var.set("Disconnected")

    def _send(self, command: str) -> None:
        try:
            self.link.send(command)
        except Exception as exc:
            messagebox.showerror("Send failed", str(exc))

    def _poll(self) -> None:
        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_line(line)
        self.after(40, self._poll)

    def _handle_line(self, line: str) -> None:
        self.last_var.set(line)
        self.log_lines.append(line)
        if len(self.log_lines) > LOG_LIMIT:
            del self.log_lines[: len(self.log_lines) - LOG_LIMIT]

        if match := SLAM_HB_RE.match(line):
            self.slam_var.set(f"{match.group('state')} front={match.group('front')}mm rev={match.group('rev')}")
            self.path_var.set(f"seq={match.group('seq')} {match.group('path_i')}/{match.group('path_len')} target={match.group('target')}")
        elif match := SLAM_STATE_RE.match(line):
            self.slam_var.set(f"{match.group('state')} reason={match.group('reason')} seq={match.group('seq')}")
        elif match := MAP_STAT_RE.match(line):
            self.map_var.set(match.group("body"))

        self.log.configure(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.insert(tk.END, "\n".join(self.log_lines[-160:]))
        self.log.configure(state=tk.DISABLED)
        self.log.see(tk.END)

    def _close(self) -> None:
        self.link.disconnect()
        self.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="Send Go/Back commands to STM32 SLAM firmware.")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    app = GoBackGui(args.port, args.baud)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
