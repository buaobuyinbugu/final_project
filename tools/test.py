import math
import queue
import re
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import ttk
from typing import List, Optional, Set, Tuple

import serial


PORT = "COM3"
BAUDRATE = 115200

DEFAULT_WIDTH = 80
DEFAULT_HEIGHT = 80
DEFAULT_CELL_MM = 50
CANVAS_PX = 640
POLL_MAX_LINES = 400
VERBOSE_SERIAL = False

MAZE_CELLS = 5
MAZE_CELL_MM = 700
MAZE_START_X_MM = -1500
MAZE_START_Y_MM = -1500
MAZE_START_CELL = (0, 0)
MAZE_EXIT_CELL = (4, 4)
MAZE_WALL_SNAP_TOLERANCE_MM = 110
MAZE_WALL_MIN_HITS = 2

COLOR_UNKNOWN = "#22252b"
COLOR_FREE = "#f3f5f7"
COLOR_OCCUPIED = "#e34b4b"
COLOR_ROBOT = "#29b36a"
COLOR_GRID = "#3a3f47"
COLOR_MAZE_GRID = "#6f7682"
COLOR_MAZE_WALL = "#ffcf5a"
COLOR_MAZE_TEXT = "#101317"

MAP_HEADER_RE = re.compile(
    r"^MAP\s+(?P<state>\w+)\s+w=(?P<w>\d+)\s+h=(?P<h>\d+)\s+"
    r"cell=(?P<cell>\d+)mm\s+rev=(?P<rev>\d+)"
)
MAP_ROW_RE = re.compile(r"^MAP\s+ROW\s+y=(?P<y>\d+)\s+rev=(?P<rev>\d+)\s+data=(?P<data>[?.#]+)")
MAP_STAT_RE = re.compile(r"^MAP\s+STAT\s+(?P<body>.*)$")
POSE_RE = re.compile(r"\bpose=(-?\d+),(-?\d+),(-?\d+)")
POSE_LINE_RE = re.compile(r"^POSE\s+(-?\d+),(-?\d+),(-?\d+)$")


@dataclass
class MapModel:
    width: int = DEFAULT_WIDTH
    height: int = DEFAULT_HEIGHT
    cell_mm: int = DEFAULT_CELL_MM
    revision: int = 0
    state: str = "IDLE"
    rows: List[str] = field(default_factory=list)
    pending_rows: List[str] = field(default_factory=list)
    pending_seen: set = field(default_factory=set)
    vertical_walls: Set[Tuple[int, int]] = field(default_factory=set)
    horizontal_walls: Set[Tuple[int, int]] = field(default_factory=set)
    stat_text: str = ""
    pose: Optional[Tuple[int, int, int]] = None
    map_dirty: bool = True
    pose_dirty: bool = True
    status_dirty: bool = True

    def __post_init__(self) -> None:
        self.reset_rows()

    def reset_rows(self) -> None:
        self.rows = ["?" * self.width for _ in range(self.height)]
        self.pending_rows = ["?" * self.width for _ in range(self.height)]
        self.pending_seen.clear()
        self.vertical_walls.clear()
        self.horizontal_walls.clear()
        self.map_dirty = True
        self.pose_dirty = True
        self.status_dirty = True

    def configure(self, state: str, width: int, height: int, cell_mm: int, revision: int) -> None:
        shape_changed = (width != self.width) or (height != self.height)
        new_map_started = state == "START" or revision < self.revision
        self.state = state
        self.width = width
        self.height = height
        self.cell_mm = cell_mm
        self.revision = revision
        if shape_changed or new_map_started or len(self.rows) != self.height:
            self.reset_rows()
        self.map_dirty = True
        self.status_dirty = True

    def update_row(self, y: int, revision: int, data: str) -> None:
        if not 0 <= y < self.height:
            return

        if len(data) < self.width:
            data = data.ljust(self.width, "?")
        elif len(data) > self.width:
            data = data[: self.width]

        self.pending_rows[y] = data
        self.rows[y] = data
        self.pending_seen.add(y)
        self.revision = revision
        self.map_dirty = True
        self.status_dirty = True
        if len(self.pending_seen) >= self.height:
            self.update_maze_walls()
            self.pending_seen.clear()

    def update_maze_walls(self) -> None:
        origin_x = MAZE_START_X_MM - (MAZE_START_CELL[0] + 0.5) * MAZE_CELL_MM
        origin_y = MAZE_START_Y_MM - (MAZE_START_CELL[1] + 0.5) * MAZE_CELL_MM
        max_x = origin_x + MAZE_CELLS * MAZE_CELL_MM
        max_y = origin_y + MAZE_CELLS * MAZE_CELL_MM
        half_w_mm = self.width * self.cell_mm / 2.0
        half_h_mm = self.height * self.cell_mm / 2.0
        vertical_counts = {}
        horizontal_counts = {}

        for row_index, row in enumerate(self.rows):
            y_mm = half_h_mm - (row_index + 0.5) * self.cell_mm
            if not (origin_y - 80 <= y_mm <= max_y + 80):
                continue

            for col_index, char in enumerate(row):
                if char != "#":
                    continue

                x_mm = (col_index + 0.5) * self.cell_mm - half_w_mm
                if not (origin_x - 80 <= x_mm <= max_x + 80):
                    continue

                vertical_index = round((x_mm - origin_x) / MAZE_CELL_MM)
                if 0 <= vertical_index <= MAZE_CELLS:
                    boundary_x = origin_x + vertical_index * MAZE_CELL_MM
                    segment_y = int((y_mm - origin_y) // MAZE_CELL_MM)
                    if (
                        abs(x_mm - boundary_x) <= MAZE_WALL_SNAP_TOLERANCE_MM
                        and 0 <= segment_y < MAZE_CELLS
                    ):
                        key = (vertical_index, segment_y)
                        vertical_counts[key] = vertical_counts.get(key, 0) + 1

                horizontal_index = round((y_mm - origin_y) / MAZE_CELL_MM)
                if 0 <= horizontal_index <= MAZE_CELLS:
                    boundary_y = origin_y + horizontal_index * MAZE_CELL_MM
                    segment_x = int((x_mm - origin_x) // MAZE_CELL_MM)
                    if (
                        abs(y_mm - boundary_y) <= MAZE_WALL_SNAP_TOLERANCE_MM
                        and 0 <= segment_x < MAZE_CELLS
                    ):
                        key = (segment_x, horizontal_index)
                        horizontal_counts[key] = horizontal_counts.get(key, 0) + 1

        self.vertical_walls = {key for key, hits in vertical_counts.items() if hits >= MAZE_WALL_MIN_HITS}
        self.horizontal_walls = {key for key, hits in horizontal_counts.items() if hits >= MAZE_WALL_MIN_HITS}

    def update_stat(self, body: str) -> None:
        self.stat_text = body
        pose_match = POSE_RE.search(body)
        if pose_match:
            self.update_pose(tuple(int(pose_match.group(i)) for i in range(1, 4)))
        self.status_dirty = True

    def update_pose(self, pose: Tuple[int, int, int]) -> None:
        self.pose = pose
        self.pose_dirty = True
        self.status_dirty = True


def parse_map_line(line: str, model: MapModel) -> bool:
    header = MAP_HEADER_RE.match(line)
    if header:
        model.configure(
            header.group("state"),
            int(header.group("w")),
            int(header.group("h")),
            int(header.group("cell")),
            int(header.group("rev")),
        )
        return True

    row = MAP_ROW_RE.match(line)
    if row:
        model.update_row(int(row.group("y")), int(row.group("rev")), row.group("data"))
        return True

    stat = MAP_STAT_RE.match(line)
    if stat:
        model.update_stat(stat.group("body"))
        return True

    pose = POSE_LINE_RE.match(line)
    if pose:
        model.update_pose(tuple(int(pose.group(i)) for i in range(1, 4)))
        return True

    return False


def read_loop(ser: serial.Serial, rx_queue: queue.Queue, stop_event: threading.Event) -> None:
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
                    if VERBOSE_SERIAL and not text.startswith("MAP ROW "):
                        print(f"[RX] {text}")
                    rx_queue.put(text)

        except serial.SerialException as e:
            if not stop_event.is_set():
                print(f"[ERROR] Serial read failed: {e}")
            break


def send_command(ser: serial.Serial, cmd: str) -> None:
    cmd = cmd.strip()
    if not cmd:
        return

    if cmd in ["0", "1", "2", "3"]:
        payload = cmd.encode()
    elif cmd == "91":
        payload = b"91\r\n"
    else:
        payload = (cmd + "\r\n").encode()

    ser.write(payload)
    ser.flush()
    if VERBOSE_SERIAL:
        print(f"[TX] {repr(payload)}")


class MapViewer:
    def __init__(
        self,
        root: tk.Tk,
        ser: serial.Serial,
        model: MapModel,
        rx_queue: queue.Queue,
        stop_event: threading.Event,
    ) -> None:
        self.root = root
        self.ser = ser
        self.model = model
        self.rx_queue = rx_queue
        self.stop_event = stop_event
        self.map_image: Optional[tk.PhotoImage] = None
        self.scaled_map_image: Optional[tk.PhotoImage] = None

        self.root.title("STM32 Mapping Viewer")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.static_layer_items: List[int] = []
        self.robot_layer_items: List[int] = []

        self.canvas = tk.Canvas(root, width=CANVAS_PX, height=CANVAS_PX, bg=COLOR_UNKNOWN, highlightthickness=0)
        self.canvas.grid(row=0, column=0, columnspan=6, padx=10, pady=(10, 6), sticky="nsew")

        self.status_var = tk.StringVar(value="No map data yet. Send 91 to start mapping.")
        self.status = ttk.Label(root, textvariable=self.status_var, wraplength=CANVAS_PX)
        self.status.grid(row=1, column=0, columnspan=6, padx=10, pady=(0, 6), sticky="w")

        self.command_var = tk.StringVar(value="91")
        self.command_entry = ttk.Entry(root, textvariable=self.command_var, width=18)
        self.command_entry.grid(row=2, column=0, padx=(10, 4), pady=(0, 10), sticky="ew")
        self.command_entry.bind("<Return>", lambda _event: self.send_entry_command())

        ttk.Button(root, text="Send", command=self.send_entry_command).grid(
            row=2, column=1, padx=4, pady=(0, 10), sticky="ew"
        )
        ttk.Button(root, text="91 Start Map", command=lambda: self.send_command_text("91")).grid(
            row=2, column=2, padx=4, pady=(0, 10), sticky="ew"
        )
        ttk.Button(root, text="96 Wall", command=lambda: self.send_command_text("96")).grid(
            row=2, column=3, padx=4, pady=(0, 10), sticky="ew"
        )
        ttk.Button(root, text="97 Stop Map", command=lambda: self.send_command_text("97")).grid(
            row=2, column=4, padx=4, pady=(0, 10), sticky="ew"
        )
        ttk.Button(root, text="0 Brake", command=lambda: self.send_command_text("0")).grid(
            row=2, column=5, padx=(4, 10), pady=(0, 10), sticky="ew"
        )

        for col in range(6):
            root.columnconfigure(col, weight=1)
        root.rowconfigure(0, weight=1)

        self.root.after(40, self.poll_serial_lines)
        self.root.after(80, self.redraw_if_needed)

    def send_entry_command(self) -> None:
        self.send_command_text(self.command_var.get())

    def send_command_text(self, cmd: str) -> None:
        try:
            send_command(self.ser, cmd)
        except serial.SerialException as e:
            print(f"[ERROR] Serial write failed: {e}")

    def poll_serial_lines(self) -> None:
        if self.stop_event.is_set():
            self.root.destroy()
            return

        processed = 0
        while processed < POLL_MAX_LINES:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            parse_map_line(line, self.model)
            processed += 1

        self.root.after(20, self.poll_serial_lines)

    def redraw_if_needed(self) -> None:
        if self.model.map_dirty:
            self.draw_map()
            self.model.map_dirty = False
            self.model.pose_dirty = False
            self.model.status_dirty = False
        else:
            if self.model.pose_dirty:
                self.draw_robot_layer()
                self.model.pose_dirty = False
            if self.model.status_dirty:
                self.update_status_text()
                self.model.status_dirty = False

        if not self.stop_event.is_set():
            self.root.after(40, self.redraw_if_needed)

    def draw_map(self) -> None:
        self.canvas.delete("all")
        if self.model.width <= 0 or self.model.height <= 0:
            return

        cell_px = max(1, int(min(CANVAS_PX / self.model.width, CANVAS_PX / self.model.height)))
        map_w_px = cell_px * self.model.width
        map_h_px = cell_px * self.model.height
        self.ox = (CANVAS_PX - map_w_px) / 2.0
        self.oy = (CANVAS_PX - map_h_px) / 2.0
        self.cell_px = cell_px

        self.draw_map_image(self.ox, self.oy, self.cell_px)
        self.draw_grid_axes(self.ox, self.oy, self.cell_px)
        self.draw_maze_overlay(self.ox, self.oy, self.cell_px)
        self.draw_robot_layer()
        self.update_status_text()

    def draw_robot_layer(self) -> None:
        for item in self.robot_layer_items:
            self.canvas.delete(item)
        self.robot_layer_items.clear()
        if hasattr(self, "ox") and hasattr(self, "oy") and hasattr(self, "cell_px"):
            self.draw_robot_pose(self.ox, self.oy, self.cell_px)

    def draw_map_image(self, ox: float, oy: float, cell_px: int) -> None:
        self.map_image = tk.PhotoImage(width=self.model.width, height=self.model.height)
        for y, row in enumerate(self.model.rows):
            color_row = []
            for char in row:
                if char == ".":
                    color = COLOR_FREE
                elif char == "#":
                    color = COLOR_OCCUPIED
                else:
                    color = COLOR_UNKNOWN
                color_row.append(color)
            self.map_image.put("{" + " ".join(color_row) + "}", to=(0, y))

        self.scaled_map_image = self.map_image.zoom(cell_px, cell_px)
        self.canvas.create_image(ox, oy, image=self.scaled_map_image, anchor="nw")

    def world_to_canvas(self, x_mm: float, y_mm: float, ox: float, oy: float, cell_px: float) -> Tuple[float, float]:
        map_w_mm = self.model.width * self.model.cell_mm
        map_h_mm = self.model.height * self.model.cell_mm
        cell_x = (x_mm + map_w_mm / 2.0) / self.model.cell_mm
        cell_y = (map_h_mm / 2.0 - y_mm) / self.model.cell_mm
        return ox + cell_x * cell_px, oy + cell_y * cell_px

    def draw_grid_axes(self, ox: float, oy: float, cell_px: float) -> None:
        center_x = ox + (self.model.width / 2.0) * cell_px
        center_y = oy + (self.model.height / 2.0) * cell_px
        self.canvas.create_line(center_x, oy, center_x, oy + self.model.height * cell_px, fill=COLOR_GRID)
        self.canvas.create_line(ox, center_y, ox + self.model.width * cell_px, center_y, fill=COLOR_GRID)

    def draw_maze_overlay(self, ox: float, oy: float, cell_px: float) -> None:
        origin_x = MAZE_START_X_MM - (MAZE_START_CELL[0] + 0.5) * MAZE_CELL_MM
        origin_y = MAZE_START_Y_MM - (MAZE_START_CELL[1] + 0.5) * MAZE_CELL_MM
        max_x = origin_x + MAZE_CELLS * MAZE_CELL_MM
        max_y = origin_y + MAZE_CELLS * MAZE_CELL_MM

        for i in range(MAZE_CELLS + 1):
            x_mm = origin_x + i * MAZE_CELL_MM
            x0, y0 = self.world_to_canvas(x_mm, origin_y, ox, oy, cell_px)
            x1, y1 = self.world_to_canvas(x_mm, max_y, ox, oy, cell_px)
            self.canvas.create_line(x0, y0, x1, y1, fill=COLOR_MAZE_GRID, width=1)

        for j in range(MAZE_CELLS + 1):
            y_mm = origin_y + j * MAZE_CELL_MM
            x0, y0 = self.world_to_canvas(origin_x, y_mm, ox, oy, cell_px)
            x1, y1 = self.world_to_canvas(max_x, y_mm, ox, oy, cell_px)
            self.canvas.create_line(x0, y0, x1, y1, fill=COLOR_MAZE_GRID, width=1)

        for i in range(MAZE_CELLS + 1):
            x_mm = origin_x + i * MAZE_CELL_MM
            for j in range(MAZE_CELLS):
                y0_mm = origin_y + j * MAZE_CELL_MM
                y1_mm = y0_mm + MAZE_CELL_MM
                if (i, j) in self.model.vertical_walls:
                    x0, y0 = self.world_to_canvas(x_mm, y0_mm, ox, oy, cell_px)
                    x1, y1 = self.world_to_canvas(x_mm, y1_mm, ox, oy, cell_px)
                    self.canvas.create_line(x0, y0, x1, y1, fill=COLOR_MAZE_WALL, width=4)

        for j in range(MAZE_CELLS + 1):
            y_mm = origin_y + j * MAZE_CELL_MM
            for i in range(MAZE_CELLS):
                x0_mm = origin_x + i * MAZE_CELL_MM
                x1_mm = x0_mm + MAZE_CELL_MM
                if (i, j) in self.model.horizontal_walls:
                    x0, y0 = self.world_to_canvas(x0_mm, y_mm, ox, oy, cell_px)
                    x1, y1 = self.world_to_canvas(x1_mm, y_mm, ox, oy, cell_px)
                    self.canvas.create_line(x0, y0, x1, y1, fill=COLOR_MAZE_WALL, width=4)

        self.draw_maze_label("S", MAZE_START_CELL, origin_x, origin_y, ox, oy, cell_px)
        self.draw_maze_label("E", MAZE_EXIT_CELL, origin_x, origin_y, ox, oy, cell_px)

    def draw_maze_label(
        self,
        label: str,
        cell: Tuple[int, int],
        origin_x: float,
        origin_y: float,
        ox: float,
        oy: float,
        cell_px: float,
    ) -> None:
        cx_mm = origin_x + (cell[0] + 0.5) * MAZE_CELL_MM
        cy_mm = origin_y + (cell[1] + 0.5) * MAZE_CELL_MM
        cx, cy = self.world_to_canvas(cx_mm, cy_mm, ox, oy, cell_px)
        radius = max(10.0, (MAZE_CELL_MM / self.model.cell_mm) * cell_px * 0.13)
        fill = "#8ff0b2" if label == "S" else "#89c2ff"
        self.canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, fill=fill, outline="")
        self.canvas.create_text(cx, cy, text=label, fill=COLOR_MAZE_TEXT, font=("Arial", 11, "bold"))

    def draw_robot_pose(self, ox: float, oy: float, cell_px: float) -> None:
        if self.model.pose is None:
            return

        x_mm, y_mm, heading_cdeg = self.model.pose
        cx, cy = self.world_to_canvas(x_mm, y_mm, ox, oy, cell_px)
        if not (ox <= cx <= ox + self.model.width * cell_px and oy <= cy <= oy + self.model.height * cell_px):
            return

        radius = max(4.0, cell_px * 1.2)
        self.robot_layer_items.append(
            self.canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, fill=COLOR_ROBOT, outline="")
        )

        heading_rad = math.radians(heading_cdeg / 100.0)
        line_len = max(12.0, cell_px * 3.0)
        hx = cx + math.cos(heading_rad) * line_len
        hy = cy - math.sin(heading_rad) * line_len
        self.robot_layer_items.append(self.canvas.create_line(cx, cy, hx, hy, fill="#0b5f37", width=3))

    def update_status_text(self) -> None:
        known = sum(row.count(".") + row.count("#") for row in self.model.rows)
        total = self.model.width * self.model.height
        text = (
            f"state={self.model.state} rev={self.model.revision} "
            f"size={self.model.width}x{self.model.height} cell={self.model.cell_mm}mm "
            f"known={known}/{total} rows={len(self.model.pending_seen)}/{self.model.height}"
        )
        if self.model.pose is not None:
            x_mm, y_mm, heading_cdeg = self.model.pose
            text += f" pose=({x_mm},{y_mm},{heading_cdeg / 100.0:.1f}deg)"
        if self.model.stat_text:
            text += f" | {self.model.stat_text}"
        self.status_var.set(text)

    def close(self) -> None:
        self.stop_event.set()
        self.root.destroy()


def console_input_loop(ser: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            break

        if cmd.lower() in ["exit", "quit", "q"]:
            stop_event.set()
            break
        if cmd:
            try:
                send_command(ser, cmd)
            except serial.SerialException as e:
                print(f"[ERROR] Serial write failed: {e}")
                stop_event.set()
                break


def main() -> None:
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
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {PORT}: {e}")
        return

    if VERBOSE_SERIAL:
        print(f"[INFO] Opened {PORT} at {BAUDRATE} baud")
        print("[INFO] Send 91 to start mapping. MAP ROW uses '?' unknown, '.' free, '#' occupied.")
        print("[INFO] You can type commands here, or use the GUI buttons. Type exit to quit.")
        print()

    rx_queue = queue.Queue()
    stop_event = threading.Event()
    model = MapModel()

    reader = threading.Thread(target=read_loop, args=(ser, rx_queue, stop_event), daemon=True)
    reader.start()
    console = threading.Thread(target=console_input_loop, args=(ser, stop_event), daemon=True)
    console.start()

    root = tk.Tk()
    MapViewer(root, ser, model, rx_queue, stop_event)

    try:
        root.mainloop()
    finally:
        stop_event.set()
        if ser.is_open:
            ser.close()
        if VERBOSE_SERIAL:
            print("\n[INFO] Serial closed")


if __name__ == "__main__":
    main()
