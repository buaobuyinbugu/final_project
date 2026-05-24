import re
import threading
import time

import serial


PORT = "COM6"
BAUDRATE = 115200
START_ODOM_ON_OPEN = True

ODOM_RE = re.compile(
    r"^ODOM\s+seq=(?P<seq>\d+)\s+"
    r"lc=(?P<left>-?\d+)\s+"
    r"rc=(?P<right>-?\d+)\s+"
    r"avg=(?P<avg>\d+)\s+"
    r"dist=(?P<dist>-?\d+)\s+"
    r"heading=(?P<heading>-?\d+)\s+"
    r"gz=(?P<gyro>-?\d+)\s+"
    r"gb=(?P<bias>-?\d+)\s+"
    r"k=(?P<k>-?\d+)"
)


def send_command(ser, cmd):
    payload = (cmd.strip() + "\r\n").encode()
    ser.write(payload)
    ser.flush()
    print(f"[TX] {repr(payload)}")


def format_odom(match):
    seq = int(match.group("seq"))
    left = int(match.group("left"))
    right = int(match.group("right"))
    avg = int(match.group("avg"))
    dist = int(match.group("dist"))
    heading_cdeg = int(match.group("heading"))
    gyro = int(match.group("gyro"))
    bias = int(match.group("bias"))
    k = int(match.group("k"))
    heading_deg = heading_cdeg / 100.0
    gyro_dps = gyro / 100.0
    bias_dps = bias / 100.0
    mm_per_count = k / 1000.0

    return (
        f"seq={seq:4d}  "
        f"left={left:8d}  "
        f"right={right:8d}  "
        f"avg={avg:8d}  "
        f"dist={dist:7d} mm  "
        f"heading={heading_deg:8.2f} deg  "
        f"gyro={gyro_dps:7.2f} dps  "
        f"bias={bias_dps:7.2f} dps  "
        f"k={k} ({mm_per_count:.4f} mm/count)"
    )


def read_loop(ser, stop_event):
    buffer = b""

    while not stop_event.is_set():
        try:
            data = ser.read(1024)
        except serial.SerialException as exc:
            print(f"[ERROR] Serial read failed: {exc}")
            stop_event.set()
            break

        if not data:
            time.sleep(0.01)
            continue

        buffer += data
        while b"\n" in buffer:
            raw_line, buffer = buffer.split(b"\n", 1)
            line = raw_line.decode(errors="ignore").strip()
            if not line:
                continue

            match = ODOM_RE.match(line)
            if match:
                print(format_odom(match))
            elif line.startswith(("ACK", "ODOM START", "ODOM STOP", "ERR")):
                print(f"[RX] {line}")


def input_loop(ser, stop_event):
    print()
    print("[INFO] Commands:")
    print("       94 / start   reset odometry debug and start printing")
    print("       95 / stop    stop odometry debug")
    print("       exit         quit")
    print()

    while not stop_event.is_set():
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            break

        lower = cmd.lower()
        if lower in ("exit", "quit", "q"):
            stop_event.set()
            break
        if lower == "start":
            cmd = "94"
        elif lower == "stop":
            cmd = "95"
        if cmd:
            try:
                send_command(ser, cmd)
            except serial.SerialException as exc:
                print(f"[ERROR] Serial write failed: {exc}")
                stop_event.set()
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
    print("[INFO] Printing ODOM debug from firmware every 500 ms.")
    print("[INFO] Move the car exactly 1000 mm, then report avg/dist/heading.")

    stop_event = threading.Event()
    reader = threading.Thread(target=read_loop, args=(ser, stop_event), daemon=True)
    reader.start()

    if START_ODOM_ON_OPEN:
        send_command(ser, "94")

    try:
        input_loop(ser, stop_event)
    finally:
        stop_event.set()
        try:
            if ser.is_open:
                send_command(ser, "95")
        except serial.SerialException:
            pass
        if ser.is_open:
            ser.close()
        print("\n[INFO] Serial closed")


if __name__ == "__main__":
    main()
