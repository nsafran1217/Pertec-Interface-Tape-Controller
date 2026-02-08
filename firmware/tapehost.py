#!/usr/bin/env python3
"""
tapehost.py – Host application for the STM32 Tape Controller.

Communicates over USB CDC with the controller firmware using a simple
binary packet protocol.  Provides a CLI and handles file I/O on behalf
of the controller.

Requirements:  pip install pyserial

Usage:  python tapehost.py [--port /dev/ttyACM0]
"""

import sys
import os
import struct
import time
import logging
import argparse
import threading

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
#  Protocol constants  (must match protocol.h on firmware)
# ---------------------------------------------------------------------------

# MCU → Host
PKT_MSG       = 0x01
PKT_READY     = 0x02
PKT_DONE      = 0x03
PKT_FOPEN     = 0x04
PKT_FWRITE    = 0x05
PKT_FREAD     = 0x06
PKT_FCLOSE    = 0x07
PKT_INPUT_REQ = 0x08

# Host → MCU
PKT_CMD       = 0x81
PKT_FRESULT   = 0x82
PKT_FDATA     = 0x83
PKT_INPUT     = 0x84
PKT_ESCAPE    = 0x85

FMODE_READ         = 0x01
FMODE_WRITE_CREATE = 0x02

PKT_HEADER_SIZE = 3

log = logging.getLogger("tapehost")

# ---------------------------------------------------------------------------
#  Connection class
# ---------------------------------------------------------------------------

class Connection:
    """Manages the USB‑CDC serial link and packet framing."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        log.info("Opening %s at %d baud", port, baudrate)
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        self.ser.close()

    # ---- Packet I/O ------------------------------------------------------

    def send_packet(self, pkt_type: int, payload: bytes = b""):
        length = len(payload)
        if length > 0xFFFF:
            raise ValueError(f"Payload too large: {length}")
        hdr = struct.pack("<BH", pkt_type, length)
        data = hdr + payload
        log.debug("TX pkt type=0x%02X len=%d", pkt_type, length)
        self.ser.write(data)
        self.ser.flush()

    def recv_packet(self, timeout: float = 60.0) -> tuple:
        """Returns (pkt_type, payload_bytes).  Raises TimeoutError."""
        old_timeout = self.ser.timeout
        self.ser.timeout = timeout
        try:
            hdr = self._read_exact(PKT_HEADER_SIZE, timeout)
            pkt_type, length = struct.unpack("<BH", hdr)
            payload = self._read_exact(length, timeout) if length else b""
            log.debug("RX pkt type=0x%02X len=%d", pkt_type, length)
            return pkt_type, payload
        finally:
            self.ser.timeout = old_timeout

    def _read_exact(self, n: int, timeout: float) -> bytes:
        buf = b""
        deadline = time.monotonic() + timeout
        while len(buf) < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"Timed out reading {n} bytes (got {len(buf)})"
                )
            self.ser.timeout = min(remaining, 1.0)
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf += chunk
        return buf

    # ---- High-level helpers ----------------------------------------------

    def send_command(self, cmd: str):
        self.send_packet(PKT_CMD, cmd.encode("utf-8"))

    def send_escape(self):
        log.info("Sending ESCAPE")
        self.send_packet(PKT_ESCAPE)

    def wait_for_ready(self, timeout: float = 10.0):
        """Block until the controller sends PKT_READY."""
        log.info("Waiting for controller READY...")
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Controller did not become ready")
            try:
                pkt_type, _ = self.recv_packet(timeout=remaining)
                if pkt_type == PKT_READY:
                    return
                if pkt_type == PKT_MSG:
                    pass  # ignore startup messages
            except TimeoutError:
                raise TimeoutError("Controller did not become ready")


# ---------------------------------------------------------------------------
#  File I/O handler  (services requests from firmware)
# ---------------------------------------------------------------------------

class FileHandler:
    """Manages a single open file on behalf of the firmware."""

    def __init__(self):
        self.fp = None
        self.path = None
        self.mode = None
        self.error = False
        self.bytes_transferred = 0

    def handle_fopen(self, conn: Connection, payload: bytes):
        mode_byte = payload[0]
        filename = payload[1:].decode("utf-8", errors="replace")
        self._close_if_open()
        log.info("FOPEN mode=0x%02X file='%s'", mode_byte, filename)
        try:
            if mode_byte == FMODE_READ:
                if not os.path.isfile(filename):
                    log.error("File not found: %s", filename)
                    conn.send_packet(PKT_FRESULT, bytes([1]))
                    return
                self.fp = open(filename, "rb")
                self.mode = "r"
            else:
                self.fp = open(filename, "wb")
                self.mode = "w"
            self.path = filename
            self.error = False
            self.bytes_transferred = 0
            conn.send_packet(PKT_FRESULT, bytes([0]))
            log.info("File opened successfully: %s", filename)
        except OSError as e:
            log.error("Failed to open %s: %s", filename, e)
            conn.send_packet(PKT_FRESULT, bytes([1]))

    def handle_fwrite(self, conn: Connection, payload: bytes):
        if not self.fp or self.mode != "w":
            log.error("FWRITE with no file open for writing")
            self.error = True
            return
        try:
            self.fp.write(payload)
            self.bytes_transferred += len(payload)
            log.debug("FWRITE %d bytes (total %d)", len(payload),
                      self.bytes_transferred)
        except OSError as e:
            log.error("Write error: %s", e)
            self.error = True

    def handle_fread(self, conn: Connection, payload: bytes):
        if not self.fp or self.mode != "r":
            log.error("FREAD with no file open for reading")
            conn.send_packet(PKT_FDATA, b"")
            return
        count = struct.unpack("<I", payload[:4])[0]
        try:
            data = self.fp.read(count)
            self.bytes_transferred += len(data)
            log.debug("FREAD req=%d got=%d (total %d)", count, len(data),
                      self.bytes_transferred)
            conn.send_packet(PKT_FDATA, data)
        except OSError as e:
            log.error("Read error: %s", e)
            conn.send_packet(PKT_FDATA, b"")

    def handle_fclose(self, conn: Connection):
        log.info("FCLOSE (transferred %d bytes)", self.bytes_transferred)
        err = self.error
        self._close_if_open()
        conn.send_packet(PKT_FRESULT, bytes([1 if err else 0]))

    def _close_if_open(self):
        if self.fp:
            try:
                self.fp.close()
            except OSError:
                pass
            self.fp = None
            self.path = None


# ---------------------------------------------------------------------------
#  Escape key listener (background thread)
# ---------------------------------------------------------------------------

class EscapeListener:
    """Watches stdin for Ctrl-C and sends PKT_ESCAPE to the controller."""

    def __init__(self, conn: Connection):
        self.conn = conn
        self.active = False

    def start(self):
        self.active = True

    def stop(self):
        self.active = False

    def send_if_active(self):
        """Call this from signal handler or keyboard interrupt."""
        if self.active:
            try:
                self.conn.send_escape()
            except Exception:
                pass


# ---------------------------------------------------------------------------
#  Command loop
# ---------------------------------------------------------------------------

def process_controller_output(conn: Connection, fh: FileHandler,
                              esc: EscapeListener):
    """
    Read packets from controller until PKT_DONE or PKT_READY.
    Handles all intermediate file-I/O and display packets.
    Returns the final packet type.
    """
    while True:
        try:
            pkt_type, payload = conn.recv_packet(timeout=120)
        except TimeoutError:
            print("\n[ERROR] Timed out waiting for controller response.")
            return PKT_DONE

        if pkt_type == PKT_MSG:
            text = payload.decode("utf-8", errors="replace")
            print(text, end="", flush=True)

        elif pkt_type == PKT_FOPEN:
            fh.handle_fopen(conn, payload)

        elif pkt_type == PKT_FWRITE:
            fh.handle_fwrite(conn, payload)

        elif pkt_type == PKT_FREAD:
            fh.handle_fread(conn, payload)

        elif pkt_type == PKT_FCLOSE:
            fh.handle_fclose(conn)

        elif pkt_type == PKT_INPUT_REQ:
            try:
                text = input()
            except EOFError:
                text = ""
            conn.send_packet(PKT_INPUT, text.encode("utf-8"))

        elif pkt_type == PKT_READY:
            return PKT_READY

        elif pkt_type == PKT_DONE:
            return PKT_DONE

        else:
            log.warning("Unknown packet type 0x%02X (len=%d)",
                        pkt_type, len(payload))


def find_port() -> str:
    """Auto-detect the tape controller CDC ACM port."""
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483 and p.pid == 0x5740:
            log.info("Auto-detected controller on %s", p.device)
            return p.device
    return None


HELP_TEXT = """
Available commands:

  status        Show detailed tape status
  brief         Show brief tape status
  rewind        Rewind tape
  unload        Unload tape (go offline)
  read [E]      Read and display next block (E = EBCDIC)
  skip [N]      Skip N blocks (negative = reverse)
  space [N]     Space N files (negative = reverse)
  create <file> [N]  Read tape to .tap image file (N = no rewind)
  write <file>  [N]  Write .tap image file to tape (N = no rewind)
  addr <hex>    Set drive/formatter address
  retries <N>   Set error retry count (0-9)
  stop <N> [E]  Set stop condition (N tapemarks; E = stop on error)
  1600          Set 1600 bpi PE density
  6250          Set 6250 bpi GCR density
  init          Re-initialize tape interface
  debug <hex>   Send raw command to formatter

  help          Show this help
  quit / exit   Exit the host application
"""


def main():
    parser = argparse.ArgumentParser(
        description="Tape Controller Host Application"
    )
    parser.add_argument(
        "--port", "-p", default=None,
        help="Serial port (e.g. /dev/ttyACM0 or COM3). Auto-detected if omitted."
    )
    parser.add_argument(
        "--baud", "-b", type=int, default=115200,
        help="Baud rate (default: 115200, ignored for USB CDC)"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Enable debug logging"
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    port = args.port
    if port is None:
        port = find_port()
        if port is None:
            print("ERROR: No tape controller found. "
                  "Specify --port manually.")
            print("Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device}  {p.description}  "
                      f"VID:PID={p.vid}:{p.pid}")
            sys.exit(1)

    try:
        conn = Connection(port, args.baud)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        sys.exit(1)

    fh  = FileHandler()
    esc = EscapeListener(conn)

    print(f"Tape Controller Host  (connected on {port})")
    print("Waiting for controller...")

    try:
        conn.wait_for_ready(timeout=15)
    except TimeoutError:
        print("ERROR: Controller did not respond. Check connection.")
        conn.close()
        sys.exit(1)

    print("Controller ready.  Type 'help' for commands.\n")

    while True:
        try:
            line = input("tape> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        lower = line.lower()
        if lower in ("quit", "exit", "q"):
            break
        if lower == "help":
            print(HELP_TEXT)
            continue

        esc.start()
        try:
            conn.send_command(line)
            process_controller_output(conn, fh, esc)
        except KeyboardInterrupt:
            print("\n[Sending abort to controller...]")
            esc.send_if_active()
            # Drain remaining output
            try:
                process_controller_output(conn, fh, esc)
            except Exception:
                pass
        except TimeoutError as e:
            print(f"\n[ERROR] {e}")
        except serial.SerialException as e:
            print(f"\n[ERROR] Serial error: {e}")
            break
        finally:
            esc.stop()

        print()  # blank line after each command

    print("Disconnecting.")
    conn.close()


if __name__ == "__main__":
    main()