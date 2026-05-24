import math
import queue
import re
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import ttk

import serial


PORT = "COM6"
BAUDRATE = 115200
CANVAS_PX = 720
MAX_RANGE_MM = 8000
MAX_POINTS = 1200

POINT_RE = re.compile(
    r"^LP\s+seq=(?P<seq>\d+)\s+a=(?P<angle>\d+)\s+d=(?P<dist>\d+)\s+"
    r"q=(?P<quality>\d+)\s+s=(?P<start>[01])"
)


@dataclass
class LidarPoint:
    seq: int
    angle_cdeg: int
    distance_mm: int
    quality: int
    scan_start: bool


class LidarDebugViewer:
    def __init__(self, root, ser, rx_queue, stop_event):
        self.root = root
        self.ser = ser
        self.rx_queue = rx_queue
        self.stop_event = stop_event
        self.points = deque(maxlen=MAX_POINTS)
        self.last_line = ""
        self.rx_points = 0
        self.scan_start_count = 0
        self.max_range_mm = MAX_RANGE_MM

        root.title("RPLidar Parsed Point Viewer")
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.canvas = tk.Canvas(root, width=CANVAS_PX, height=CANVAS_PX, bg="#111418", highlightthickness=0)
        self.canvas.grid(row=0, column=0, columnspan=5, padx=10, pady=(10, 6), sticky="nsew")

        self.status_var = tk.StringVar(value="Send 92 to start parsed lidar point stream.")
        self.status = ttk.Label(root, textvariable=self.status_var, wraplength=CANVAS_PX)
        self.status.grid(row=1, column=0, columnspan=5, padx=10, pady=(0, 6), sticky="w")

        self.range_var = tk.IntVar(value=MAX_RANGE_MM)
        ttk.Label(root, text="Range mm").grid(row=2, column=0, padx=(10, 4), pady=(0, 8), sticky="e")
        self.range_scale = ttk.Scale(
            root,
            from_=1000,
            to=12000,
            variable=self.range_var,
            command=lambda _value: self.update_range(),
        )
        self.range_scale.grid(row=2, column=1, columnspan=4, padx=(4, 10), pady=(0, 8), sticky="ew")

        ttk.Button(root, text="92 Start", command=lambda: self.send_command("92")).grid(
            row=3, column=0, padx=(10, 4), pady=(0, 10), sticky="ew"
        )
        ttk.Button(root, text="93 Stop", command=lambda: self.send_command("93")).grid(
            row=3, column=1, padx=4, pady=(0, 10), sticky="ew"
        )
        ttk.Button(root, text="Clear", command=self.clear_points).grid(
            row=3, column=2, padx=4, pady=(0, 10), sticky="ew"
        )
        ttk.Button(root, text="Close", command=self.close).grid(
            row=3, column=4, padx=(4, 10), pady=(0, 10), sticky="ew"
        )

        for col in range(5):
            root.columnconfigure(col, weight=1)
        root.rowconfigure(0, weight=1)

        self.root.after(30, self.poll_serial_lines)
        self.root.after(100, self.draw)

    def update_range(self):
        self.max_range_mm = max(500, int(self.range_var.get()))

    def send_command(self, cmd):
        payload = (cmd.strip() + "\r\n").encode()
        try:
            self.ser.write(payload)
            self.ser.flush()
            print(f"[TX] {repr(payload)}")
        except serial.SerialException as exc:
            print(f"[ERROR] Serial write failed: {exc}")

    def clear_points(self):
        self.points.clear()
        self.rx_points = 0
        self.scan_start_count = 0
        self.draw()

    def poll_serial_lines(self):
        if self.stop_event.is_set():
            self.root.destroy()
            return

        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break

            self.last_line = line
            point = parse_point(line)
            if point is None:
                continue

            self.points.append(point)
            self.rx_points += 1
            if point.scan_start:
                self.scan_start_count += 1

        self.root.after(30, self.poll_serial_lines)

    def draw(self):
        self.canvas.delete("all")
        center = CANVAS_PX / 2.0
        radius = CANVAS_PX * 0.46

        self.draw_grid(center, radius)

        for point in self.points:
            if point.distance_mm <= 0 or point.distance_mm > self.max_range_mm:
                continue

            angle_rad = math.radians(point.angle_cdeg / 100.0)
            r = (point.distance_mm / float(self.max_range_mm)) * radius
            x = center + math.cos(angle_rad) * r
            y = center - math.sin(angle_rad) * r
            color = self.quality_color(point.quality)
            size = 4 if point.scan_start else 2
            self.canvas.create_oval(x - size, y - size, x + size, y + size, fill=color, outline="")

        self.update_status()

        if not self.stop_event.is_set():
            self.root.after(100, self.draw)

    def draw_grid(self, center, radius):
        for fraction in (0.25, 0.5, 0.75, 1.0):
            r = radius * fraction
            self.canvas.create_oval(center - r, center - r, center + r, center + r, outline="#2b3138")

        self.canvas.create_line(center - radius, center, center + radius, center, fill="#3a414a")
        self.canvas.create_line(center, center - radius, center, center + radius, fill="#3a414a")
        self.canvas.create_oval(center - 5, center - 5, center + 5, center + 5, fill="#2fbf71", outline="")

        self.canvas.create_text(center + radius - 18, center - 12, text="0", fill="#9aa4b2")
        self.canvas.create_text(center + 14, center - radius + 12, text="90", fill="#9aa4b2")
        self.canvas.create_text(center - radius + 22, center - 12, text="180", fill="#9aa4b2")
        self.canvas.create_text(center + 18, center + radius - 12, text="270", fill="#9aa4b2")

    @staticmethod
    def quality_color(quality):
        if quality >= 40:
            return "#53d86a"
        if quality >= 15:
            return "#f0c24b"
        return "#ef6b5b"

    def update_status(self):
        text = (
            f"points={len(self.points)} total_rx={self.rx_points} "
            f"scan_start={self.scan_start_count} range={self.max_range_mm}mm"
        )
        if self.points:
            last = self.points[-1]
            text += (
                f" last: seq={last.seq} angle={last.angle_cdeg / 100.0:.2f}deg "
                f"dist={last.distance_mm}mm q={last.quality} start={int(last.scan_start)}"
            )
        elif self.last_line:
            text += f" last_line={self.last_line}"
        self.status_var.set(text)

    def close(self):
        self.stop_event.set()
        try:
            if self.ser.is_open:
                self.send_command("93")
        finally:
            self.root.destroy()


def parse_point(line):
    match = POINT_RE.match(line)
    if not match:
        return None

    return LidarPoint(
        seq=int(match.group("seq")),
        angle_cdeg=int(match.group("angle")),
        distance_mm=int(match.group("dist")),
        quality=int(match.group("quality")),
        scan_start=(match.group("start") == "1"),
    )


def read_loop(ser, rx_queue, stop_event):
    buffer = b""

    while not stop_event.is_set():
        try:
            data = ser.read(1024)
            if not data:
                time.sleep(0.01)
                continue

            buffer += data
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                text = raw_line.decode(errors="ignore").strip()
                if text:
                    print(f"[RX] {text}")
                    rx_queue.put(text)

        except serial.SerialException as exc:
            if not stop_event.is_set():
                print(f"[ERROR] Serial read failed: {exc}")
            break


def main():
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except serial.SerialException as exc:
        print(f"[ERROR] Cannot open {PORT}: {exc}")
        return

    print(f"[INFO] Opened {PORT} at {BAUDRATE} baud")
    print("[INFO] Click 92 Start to stream parsed lidar points, 93 Stop to stop.")

    rx_queue = queue.Queue()
    stop_event = threading.Event()
    reader = threading.Thread(target=read_loop, args=(ser, rx_queue, stop_event), daemon=True)
    reader.start()

    root = tk.Tk()
    LidarDebugViewer(root, ser, rx_queue, stop_event)

    try:
        root.mainloop()
    finally:
        stop_event.set()
        if ser.is_open:
            ser.close()
        print("\n[INFO] Serial closed")


if __name__ == "__main__":
    main()
