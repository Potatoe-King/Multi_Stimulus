#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rpi_led_pressure_adc_logger.py

Raspberry Pi 5:
- Reads 2 ADS1263 ADC1 differential pairs (software scan: pair1, pair2).
- Reads pressure stream from an ESP32 (Adafruit MPRLS).
- Controls LED panel via a second ESP32 (HUB75 controller) using keyboard input.
- Logs CSV: t_s, led_state, pressure_hpa, diff1_V, diff2_V

Ground-truth timing:
- Uses Pi timebase (time.time()) for all timestamps.
- LED command time is logged; handshake ACK is required to confirm state.

Requires:
- config_pi5.py and ADS1263_pi5_new.py in same directory.
- pyserial: pip install pyserial
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
import threading
from pathlib import Path
from typing import Optional, Tuple

import serial  # pyserial

# Your uploaded driver modules (keep filenames as-is)
import ADS1263_pi5_new as ADS1263_mod


# --------------------------
# Shared state (thread-safe)
# --------------------------
class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self.led_state: int = 0           # 0=OFF, 1=ON
        self.pressure_hpa: float = float("nan")
        self.pressure_rx_t_s: float = float("nan")
        self.running: bool = True

    def set_led_state(self, s: int) -> None:
        with self._lock:
            self.led_state = int(s)

    def get_led_state(self) -> int:
        with self._lock:
            return int(self.led_state)

    def update_pressure(self, p_hpa: float, rx_t_s: float) -> None:
        with self._lock:
            self.pressure_hpa = float(p_hpa)
            self.pressure_rx_t_s = float(rx_t_s)

    def get_pressure(self) -> Tuple[float, float]:
        with self._lock:
            return float(self.pressure_hpa), float(self.pressure_rx_t_s)

    def stop(self) -> None:
        with self._lock:
            self.running = False

    def is_running(self) -> bool:
        with self._lock:
            return bool(self.running)


# --------------------------
# Serial helpers
# --------------------------
def open_serial(port: str, baud: int, timeout_s: float) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=timeout_s,
        write_timeout=timeout_s,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        rtscts=False,
        dsrdtr=False,
    )
    # Avoid toggling DTR resets on some ESP32 boards (best-effort)
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    # Give device a moment after open
    time.sleep(0.2)
    ser.reset_input_buffer()
    return ser


def _readline_ascii(ser: serial.Serial) -> Optional[str]:
    try:
        line = ser.readline()
        if not line:
            return None
        return line.decode("utf-8", errors="ignore").strip()
    except Exception:
        return None


# --------------------------
# LED command protocol
# --------------------------
def led_send_command(ser_led: serial.Serial, shared: SharedState, desired_state: int, ack_timeout_s: float = 1.0) -> bool:
    """
    Protocol (line-based):
      Send: "ON\n" or "OFF\n"
      Expect: "OK ON" or "OK OFF" (case-insensitive) within ack_timeout_s
    """
    cmd = "ON" if desired_state else "OFF"
    try:
        ser_led.write((cmd + "\n").encode("utf-8"))
        ser_led.flush()
    except Exception as e:
        print(f"[LED] Serial write failed: {e}", file=sys.stderr)
        return False

    t0 = time.time()
    while (time.time() - t0) < ack_timeout_s:
        line = _readline_ascii(ser_led)
        if not line:
            continue
        lo = line.strip().lower()
        if lo.startswith("ok"):
            if "on" in lo and desired_state == 1:
                shared.set_led_state(1)
                return True
            if "off" in lo and desired_state == 0:
                shared.set_led_state(0)
                return True
        # Allow READY banners, etc.
    print(f"[LED] ACK timeout for command {cmd}", file=sys.stderr)
    return False


# --------------------------
# Pressure reader thread
# --------------------------
def pressure_reader_thread(ser_press: serial.Serial, shared: SharedState) -> None:
    """
    Pressure ESP32 should stream lines like:
      "<ms>,<pressure_hpa>"
    Example:
      123456,1013.25

    We ignore the ESP32 ms for timing (Pi timebase is ground truth),
    but parse pressure_hpa and store latest value.
    """
    while shared.is_running():
        line = _readline_ascii(ser_press)
        if not line:
            continue

        # Try parse CSV: ms,pressure_hpa
        try:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 2:
                pressure_hpa = float(parts[1])
            else:
                # Fallback: if device prints only pressure
                pressure_hpa = float(parts[0])
            shared.update_pressure(pressure_hpa, time.time())
        except Exception:
            # Ignore malformed lines
            continue


# --------------------------
# Keyboard thread (terminal)
# --------------------------
def keyboard_thread(ser_led: serial.Serial, shared: SharedState) -> None:
    """
    Reads from stdin (blocking). Commands:
      on  -> LED ON
      off -> LED OFF
      q   -> quit
    """
    print("\n[Keyboard] Commands: 'on' | 'off' | 'q' (then Enter)\n")
    while shared.is_running():
        try:
            s = sys.stdin.readline()
            if not s:
                continue
            cmd = s.strip().lower()
        except Exception:
            continue

        if cmd in ("q", "quit", "exit"):
            shared.stop()
            break
        elif cmd in ("on", "1"):
            ok = led_send_command(ser_led, shared, desired_state=1)
            if ok:
                print("[LED] ON (ACKed)")
        elif cmd in ("off", "0"):
            ok = led_send_command(ser_led, shared, desired_state=0)
            if ok:
                print("[LED] OFF (ACKed)")
        else:
            print("[Keyboard] Unknown command. Use: on | off | q")


# --------------------------
# Main
# --------------------------
def make_outfile(out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    return out_dir / f"log_{ts}.csv"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port-led", required=True, help="Serial port for LED ESP32 (e.g., /dev/ttyUSB0)")
    ap.add_argument("--port-press", required=True, help="Serial port for pressure ESP32 (e.g., /dev/ttyUSB1)")
    ap.add_argument("--baud-led", type=int, default=115200)
    ap.add_argument("--baud-press", type=int, default=115200)

    ap.add_argument("--pair1", type=int, default=0, help="ADS1263 differential pair index (0..4): 0->AIN0-1, 1->AIN2-3, ...")
    ap.add_argument("--pair2", type=int, default=1, help="Second ADS1263 differential pair index (0..4)")
    ap.add_argument("--adc-rate", default="ADS1263_400SPS", help="Key in ADS1263_DRATE dict, e.g., ADS1263_100SPS, ADS1263_400SPS")
    ap.add_argument("--vref", type=float, default=5.0, help="Reference voltage used for code->volts conversion")
    ap.add_argument("--out-dir", default="logs", help="Directory to store CSV logs")

    args = ap.parse_args()

    shared = SharedState()
    t_start = time.time()

    # Open serial ports
    ser_led = open_serial(args.port_led, args.baud_led, timeout_s=0.2)
    ser_press = open_serial(args.port_press, args.baud_press, timeout_s=0.2)

    # Best-effort: read and print any boot banners
    for _ in range(5):
        line = _readline_ascii(ser_led)
        if line:
            print(f"[LED ESP32] {line}")
    for _ in range(5):
        line = _readline_ascii(ser_press)
        if line:
            print(f"[PRESS ESP32] {line}")

    # Initialize ADC (ADS1263)
    adc = ADS1263_mod.ADS1263(vref=args.vref)
    rc = adc.ADS1263_init_ADC1(rate_key=args.adc_rate)
    if rc < 0:
        print("[ADC] ADS1263 init failed.", file=sys.stderr)
        return 2
    adc.ADS1263_SetMode(1)  # differential mode

    # Ensure LED starts OFF (and we log state from ACK)
    led_send_command(ser_led, shared, desired_state=0)

    # Start threads
    th_press = threading.Thread(target=pressure_reader_thread, args=(ser_press, shared), daemon=True)
    th_kbd = threading.Thread(target=keyboard_thread, args=(ser_led, shared), daemon=True)
    th_press.start()
    th_kbd.start()

    # CSV output
    out_path = make_outfile(Path(args.out_dir))
    print(f"[Logger] Writing: {out_path}")

    # Line-buffered text file for minimal loss
    with open(out_path, "w", newline="", buffering=1) as f:
        w = csv.writer(f)
        w.writerow(["t_s", "led_state", "pressure_hpa", "diff1_V", "diff2_V"])

        try:
            while shared.is_running():
                now = time.time()
                t_s = now - t_start

                # Read two differential pairs (each call:
                #   set MUX -> discard first conversion -> return settled second conversion)
                code1 = adc.ADS1263_GetChannalValue(args.pair1)
                v1 = adc.code_to_volts(code1)

                code2 = adc.ADS1263_GetChannalValue(args.pair2)
                v2 = adc.code_to_volts(code2)

                led_state = shared.get_led_state()
                pressure_hpa, _rx_t = shared.get_pressure()

                w.writerow([f"{t_s:.6f}", led_state, f"{pressure_hpa:.3f}", f"{v1:.9f}", f"{v2:.9f}"])

        except KeyboardInterrupt:
            print("\n[Logger] Ctrl-C received. Stopping...")
        finally:
            shared.stop()
            # Attempt to turn LED off on exit
            try:
                led_send_command(ser_led, shared, desired_state=0, ack_timeout_s=0.5)
            except Exception:
                pass

            try:
                adc.ADS1263_Exit()
            except Exception:
                pass

            try:
                ser_led.close()
            except Exception:
                pass
            try:
                ser_press.close()
            except Exception:
                pass

    print("[Logger] Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
