"""Small Windows Bluetooth serial console for the STM32 car firmware.

The script is intentionally separate from the visualizers. It focuses on
finding/opening a Bluetooth SPP COM port and sending line-based commands with
CRLF, which is what the firmware parser handles most reliably.
"""

import argparse
import sys
import threading
import time
from typing import Iterable, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    print("pyserial is required. Install it with: python -m pip install pyserial")
    raise SystemExit(2) from exc


DEFAULT_BAUDRATE = 921600
DEFAULT_READ_SIZE = 256
BLUETOOTH_KEYWORDS = ("bluetooth", "standard serial over bluetooth", "bth")


def iter_ports() -> Iterable[list_ports.ListPortInfo]:
    return list_ports.comports()


def print_ports() -> None:
    ports = list(iter_ports())
    if not ports:
        print("No serial ports found.")
        return

    for port in ports:
        details = " ".join(
            item for item in (port.description, port.manufacturer, port.hwid) if item
        )
        print(f"{port.device:>6}  {details}")


def pick_bluetooth_port() -> Optional[str]:
    candidates = []
    for port in iter_ports():
        text = " ".join(
            item for item in (port.device, port.description, port.manufacturer, port.hwid) if item
        ).lower()
        if any(keyword in text for keyword in BLUETOOTH_KEYWORDS):
            candidates.append(port.device)

    if len(candidates) == 1:
        return candidates[0]

    if candidates:
        print("Multiple Bluetooth-looking ports found:")
        for candidate in candidates:
            print(f"  {candidate}")
    return None


def reader_loop(ser: serial.Serial, stop_event: threading.Event) -> None:
    buffer = b""
    while not stop_event.is_set():
        try:
            chunk = ser.read(DEFAULT_READ_SIZE)
        except serial.SerialException as exc:
            print(f"\n[serial read error] {exc}")
            stop_event.set()
            break

        if not chunk:
            time.sleep(0.01)
            continue

        buffer += chunk
        while b"\n" in buffer:
            raw_line, buffer = buffer.split(b"\n", 1)
            line = raw_line.decode(errors="replace").rstrip("\r")
            if line:
                print(f"< {line}")

        if len(buffer) > 1024:
            text = buffer.decode(errors="replace")
            print(f"< {text}", end="")
            buffer = b""


def write_command(ser: serial.Serial, command: str) -> None:
    command = command.strip()
    if not command:
        return
    ser.write((command + "\r\n").encode("ascii", errors="ignore"))
    ser.flush()
    print(f"> {command}")


def run_console(port: str, baudrate: int) -> int:
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.05,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False,
        )
    except serial.SerialException as exc:
        print(f"Failed to open {port}: {exc}")
        return 2

    stop_event = threading.Event()
    thread = threading.Thread(target=reader_loop, args=(ser, stop_event), daemon=True)
    thread.start()

    print(f"Connected to {port} @ {baudrate}. Commands: GO, BACK, SLAM OFF, GYRO CAL, 96, 99")
    print("Type /ports to list ports, /quit to exit.")
    try:
        while not stop_event.is_set():
            try:
                text = input()
            except EOFError:
                break

            command = text.strip()
            if not command:
                continue
            if command.lower() in ("/q", "/quit", "quit", "exit"):
                break
            if command.lower() == "/ports":
                print_ports()
                continue
            write_command(ser, command)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        try:
            ser.close()
        except serial.SerialException:
            pass

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Connect to the STM32 Bluetooth SPP COM port.")
    parser.add_argument("--port", "-p", help="COM port, for example COM7.")
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--list", action="store_true", help="List serial ports and exit.")
    args = parser.parse_args()

    if args.list:
        print_ports()
        return 0

    port = args.port or pick_bluetooth_port()
    if not port:
        print("Could not auto-select a Bluetooth COM port. Available ports:")
        print_ports()
        print("Run again with: python tools\\bluetooth_connect.py --port COMx")
        return 1

    return run_console(port, args.baud)


if __name__ == "__main__":
    sys.exit(main())
