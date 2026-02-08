#!/usr/bin/env python3
"""
tape_host.py – Host application for the Pertec tape controller.

Communicates with the STM32 MCU over USB CDC using a binary packet protocol.
Provides an interactive command-line interface for tape operations.

Requirements: pyserial  (pip install pyserial)

Usage:
    python tape_host.py [--port /dev/ttyACM0] [--timeout 30]
"""

import argparse
import cmd
import os
import struct
import sys
import time

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# Protocol constants (must match protocol.h on MCU)
# ---------------------------------------------------------------------------

# Commands (host -> MCU)
CMD_PING        = 0x01
CMD_STATUS      = 0x02
CMD_INIT        = 0x03
CMD_REWIND      = 0x04
CMD_UNLOAD      = 0x05
CMD_READ_FWD    = 0x06
CMD_SKIP        = 0x07
CMD_SPACE       = 0x08
CMD_SET_ADDR    = 0x09
CMD_SET_STOP    = 0x0A
CMD_SET_RETRIES = 0x0B
CMD_DEBUG       = 0x0C
CMD_SET_1600    = 0x0D
CMD_SET_6250    = 0x0E
CMD_CREATE_IMG  = 0x0F
CMD_WRITE_IMG   = 0x10
CMD_ABORT       = 0x11
CMD_FILE_DATA   = 0x12
CMD_FILE_EOF    = 0x13

# Responses (MCU -> host)
RSP_OK          = 0x80
RSP_ERROR       = 0x81
RSP_STATUS      = 0x82
RSP_BLOCK_DATA  = 0x83
RSP_MSG         = 0x84
RSP_IMG_DATA    = 0x85
RSP_IMG_DONE    = 0x86
RSP_PONG        = 0x88
RSP_READY       = 0x89

# Flags
FLAG_NO_REWIND  = 0x01
FLAG_EBCDIC     = 0x02

# Tape status bits (from tapedriver.h)
TSTAT_OFFLINE   = 0x80
TSTAT_HARDERR   = 0x40
TSTAT_CORRERR   = 0x20
TSTAT_TAPEMARK  = 0x10
TSTAT_EOT       = 0x08
TSTAT_BLANK     = 0x04
TSTAT_LENGTH    = 0x02
TSTAT_PROTECT   = 0x01

# Pertec status bits (from pertbits.h)
PS0_IRP     = 1
PS0_IDBY    = 2
PS0_ISPEED  = 4
PS0_RDAVAIL = 8
PS0_WREMPTY = 16
PS0_IFMK    = 32
PS0_IHER    = 64
PS0_ICER    = 128
PS1_INRZ    = 1 << 8
PS1_EOT     = 2 << 8
PS1_IONL    = 4 << 8
PS1_IFPT    = 8 << 8
PS1_IRWD    = 16 << 8
PS1_ILDP    = 32 << 8
PS1_IRDY    = 64 << 8
PS1_IFBY    = 128 << 8

STATUS_LABELS = [
    (PS0_IRP,    "Odd"),
    (PS0_IDBY,   "Data busy"),
    (PS0_ISPEED, "High speed"),
    (PS0_RDAVAIL,"Read full"),
    (PS0_WREMPTY,"Write empty"),
    (PS0_IFMK,   "Tape mark"),
    (PS0_IHER,   "Hard error"),
    (PS0_ICER,   "Soft error"),
    (PS1_INRZ,   "NRZI mode"),
    (PS1_EOT,    "EOT"),
    (PS1_IONL,   "Online"),
    (PS1_IFPT,   "Protected"),
    (PS1_IRWD,   "Rewinding"),
    (PS1_ILDP,   "Load point"),
    (PS1_IRDY,   "Ready"),
]

# TAP format constants
TAP_EOM         = 0xFFFFFFFF
TAP_ERROR_FLAG  = 0x80000000
TAP_LENGTH_MASK = 0x00FFFFFF

PKT_HDR_SIZE = 5

# EBCDIC-to-ASCII table
EBCDIC_TABLE = bytes([
    0x00,0x01,0x02,0x03,0x9C,0x09,0x0A,0x7F,
    0x97,0x8D,0x8E,0x0B,0x0C,0x0D,0x0E,0x0F,
    0x10,0x11,0x12,0x13,0x9D,0x0A,0x08,0x87,
    0x18,0x19,0x92,0x8F,0x1C,0x1D,0x1E,0x1F,
    0x80,0x81,0x82,0x83,0x84,0x0A,0x17,0x1B,
    0x88,0x89,0x8A,0x8B,0x8C,0x05,0x06,0x07,
    0x90,0x91,0x16,0x93,0x94,0x95,0x96,0x04,
    0x98,0x99,0x9A,0x9B,0x14,0x15,0x9E,0x1A,
    0x20,0xA0,0xE2,0xE4,0xE0,0xE1,0xE3,0xE5,
    0xE7,0xF1,0xA2,0x2E,0x3C,0x28,0x2B,0x7C,
    0x26,0xE9,0xEA,0xEB,0xE8,0xED,0xEE,0xEF,
    0xEC,0xDF,0x21,0x24,0x2A,0x29,0x3B,0xAC,
    0x2D,0x2F,0xC2,0xC4,0xC0,0xC1,0xC3,0xC5,
    0xC7,0xD1,0xA6,0x2C,0x25,0x5F,0x3E,0x3F,
    0xF8,0xC9,0xCA,0xCB,0xC8,0xCD,0xCE,0xCF,
    0xCC,0x60,0x3A,0x23,0x40,0x27,0x3D,0x22,
    0xD8,0x61,0x62,0x63,0x64,0x65,0x66,0x67,
    0x68,0x69,0xAB,0xBB,0xF0,0xFD,0xFE,0xB1,
    0xB0,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,0x70,
    0x71,0x72,0xAA,0xBA,0xE6,0xB8,0xC6,0xA4,
    0xB5,0x7E,0x73,0x74,0x75,0x76,0x77,0x78,
    0x79,0x7A,0xA1,0xBF,0xD0,0xDD,0xDE,0xAE,
    0x5E,0xA3,0xA5,0xB7,0xA9,0xA7,0xB6,0xBC,
    0xBD,0xBE,0x5B,0x5D,0xAF,0xA8,0xB4,0xD7,
    0x7B,0x41,0x42,0x43,0x44,0x45,0x46,0x47,
    0x48,0x49,0xAD,0xF4,0xF6,0xF2,0xF3,0xF5,
    0x7D,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,
    0x51,0x52,0xB9,0xFB,0xFC,0xF9,0xFA,0xFF,
    0x5C,0xF7,0x53,0x54,0x55,0x56,0x57,0x58,
    0x59,0x5A,0xB2,0xD4,0xD6,0xD2,0xD3,0xD5,
    0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,
    0x38,0x39,0xB3,0xDB,0xDC,0xD9,0xDA,0x9F,
])


# ---------------------------------------------------------------------------
# Transport layer
# ---------------------------------------------------------------------------

class TapeLink:
    """Low-level packet I/O over serial."""

    def __init__(self, port, timeout=10):
        self.ser = serial.Serial(port, baudrate=115200,
                                 timeout=timeout, write_timeout=timeout)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        self.ser.close()

    def send_pkt(self, ptype, payload=b""):
        hdr = struct.pack("<BI", ptype, len(payload))
        self.ser.write(hdr + payload)
        self.ser.flush()

    def recv_pkt(self, timeout=None):
        """Receive one packet. Returns (type, payload_bytes).
        Handles RSP_MSG inline by printing messages."""
        old_to = self.ser.timeout
        if timeout is not None:
            self.ser.timeout = timeout
        try:
            while True:
                hdr = self._read_exact(PKT_HDR_SIZE)
                ptype = hdr[0]
                plen = struct.unpack_from("<I", hdr, 1)[0]
                payload = self._read_exact(plen) if plen else b""
                if ptype == RSP_MSG:
                    print(f"  [MCU] {payload.decode('ascii', errors='replace')}")
                    continue
                return ptype, payload
        finally:
            self.ser.timeout = old_to

    def _read_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                raise TimeoutError(
                    f"Timeout: expected {n} bytes, got {len(buf)}")
            buf.extend(chunk)
        return bytes(buf)


# ---------------------------------------------------------------------------
# High-level API
# ---------------------------------------------------------------------------

class TapeController:
    """High-level tape operations."""

    def __init__(self, link: TapeLink):
        self.link = link

    def ping(self):
        self.link.send_pkt(CMD_PING)
        t, _ = self.link.recv_pkt(timeout=3)
        return t == RSP_PONG

    def _simple_cmd(self, cmd, payload=b"", timeout=60):
        """Send command, return (ok: bool, payload or error_msg)."""
        self.link.send_pkt(cmd, payload)
        t, p = self.link.recv_pkt(timeout=timeout)
        if t == RSP_OK:
            return True, p
        if t == RSP_ERROR:
            code = struct.unpack_from("<H", p, 0)[0] if len(p) >= 2 else 0
            msg = p[2:].decode("ascii", errors="replace") if len(p) > 2 else ""
            return False, f"Error 0x{code:04X}: {msg}"
        return False, f"Unexpected response type 0x{t:02X}"

    def status(self):
        """Get tape status. Returns (raw_status, position, labels)."""
        self.link.send_pkt(CMD_STATUS)
        # Collect RSP_STATUS (may have RSP_MSG before it)
        t, p = self.link.recv_pkt(timeout=5)
        if t == RSP_STATUS and len(p) >= 6:
            raw = struct.unpack_from("<H", p, 0)[0]
            pos = struct.unpack_from("<I", p, 2)[0]
            labels = [lbl for bit, lbl in STATUS_LABELS if raw & bit]
            return raw, pos, labels
        if t == RSP_ERROR:
            code = struct.unpack_from("<H", p, 0)[0] if len(p) >= 2 else 0
            msg = p[2:].decode("ascii", errors="replace") if len(p) > 2 else ""
            raise RuntimeError(f"Status error 0x{code:04X}: {msg}")
        raise RuntimeError(f"Unexpected response 0x{t:02X}")

    def init_tape(self):
        return self._simple_cmd(CMD_INIT)

    def rewind(self):
        return self._simple_cmd(CMD_REWIND, timeout=120)

    def unload(self):
        return self._simple_cmd(CMD_UNLOAD, timeout=120)

    def skip(self, count):
        payload = struct.pack("<h", count)
        return self._simple_cmd(CMD_SKIP, payload, timeout=30)

    def space(self, count):
        payload = struct.pack("<h", count)
        return self._simple_cmd(CMD_SPACE, payload, timeout=30)

    def set_address(self, addr):
        return self._simple_cmd(CMD_SET_ADDR, bytes([addr & 0x07]))

    def set_retries(self, n):
        return self._simple_cmd(CMD_SET_RETRIES, bytes([n & 0x0F]))

    def set_stop(self, mode, stop_on_error=False):
        m = ord('V') if isinstance(mode, str) and mode.upper() == 'V' else int(mode)
        return self._simple_cmd(CMD_SET_STOP,
                                bytes([m & 0xFF, 1 if stop_on_error else 0]))

    def debug_cmd(self, value):
        return self._simple_cmd(CMD_DEBUG, struct.pack("<H", value))

    def set_1600(self):
        return self._simple_cmd(CMD_SET_1600)

    def set_6250(self):
        return self._simple_cmd(CMD_SET_6250)

    def read_block(self, ebcdic=False):
        """Read one tape block. Returns (tape_status, data)."""
        flags = FLAG_EBCDIC if ebcdic else 0
        self.link.send_pkt(CMD_READ_FWD, bytes([flags]))
        t, p = self.link.recv_pkt(timeout=30)
        if t == RSP_BLOCK_DATA and len(p) >= 2:
            ts = struct.unpack_from("<H", p, 0)[0]
            data = p[2:]
            return ts, data
        if t == RSP_ERROR:
            code = struct.unpack_from("<H", p, 0)[0] if len(p) >= 2 else 0
            msg = p[2:].decode("ascii", errors="replace") if len(p) > 2 else ""
            raise RuntimeError(f"Read error 0x{code:04X}: {msg}")
        raise RuntimeError(f"Unexpected response 0x{t:02X}")

    def create_image(self, filename, no_rewind=False):
        """Read tape into a .TAP image file."""
        flags = FLAG_NO_REWIND if no_rewind else 0
        self.link.send_pkt(CMD_CREATE_IMG, bytes([flags]))

        # First response should be RSP_OK (operation started)
        t, p = self.link.recv_pkt(timeout=120)
        if t == RSP_ERROR:
            msg = p[2:].decode("ascii", errors="replace") if len(p) > 2 else ""
            raise RuntimeError(f"Create image failed: {msg}")
        if t != RSP_OK:
            raise RuntimeError(f"Unexpected response 0x{t:02X}")

        # Receive streaming TAP data
        bytes_written = 0
        try:
            with open(filename, "wb") as f:
                while True:
                    t, p = self.link.recv_pkt(timeout=120)
                    if t == RSP_IMG_DATA:
                        f.write(p)
                        bytes_written += len(p)
                    elif t == RSP_IMG_DONE:
                        return self._parse_img_done(p, bytes_written)
                    elif t == RSP_ERROR:
                        msg = p[2:].decode("ascii", errors="replace") \
                              if len(p) > 2 else ""
                        raise RuntimeError(f"Error during read: {msg}")
                    # RSP_MSG is handled transparently by recv_pkt
        except KeyboardInterrupt:
            print("\nSending abort...")
            self.link.send_pkt(CMD_ABORT)
            # Drain until IMG_DONE
            while True:
                t, p = self.link.recv_pkt(timeout=30)
                if t == RSP_IMG_DONE:
                    return self._parse_img_done(p, bytes_written)

    def write_image(self, filename, no_rewind=False):
        """Write a .TAP image file to tape."""
        if not os.path.exists(filename):
            raise FileNotFoundError(f"File not found: {filename}")

        flags = FLAG_NO_REWIND if no_rewind else 0
        self.link.send_pkt(CMD_WRITE_IMG, bytes([flags]))

        t, p = self.link.recv_pkt(timeout=120)
        if t == RSP_ERROR:
            msg = p[2:].decode("ascii", errors="replace") if len(p) > 2 else ""
            raise RuntimeError(f"Write image failed: {msg}")
        if t != RSP_OK:
            raise RuntimeError(f"Unexpected response 0x{t:02X}")

        # Stream file data to MCU
        CHUNK = 4096
        try:
            with open(filename, "rb") as f:
                while True:
                    # Wait for RSP_READY
                    t, p = self.link.recv_pkt(timeout=120)
                    if t == RSP_READY:
                        chunk = f.read(CHUNK)
                        if chunk:
                            self.link.send_pkt(CMD_FILE_DATA, chunk)
                        else:
                            self.link.send_pkt(CMD_FILE_EOF)
                            break
                    elif t == RSP_IMG_DONE:
                        return self._parse_img_done(p, 0)
                    elif t == RSP_ERROR:
                        msg = p[2:].decode("ascii", errors="replace") \
                              if len(p) > 2 else ""
                        raise RuntimeError(f"Error during write: {msg}")
        except KeyboardInterrupt:
            print("\nSending abort...")
            self.link.send_pkt(CMD_ABORT)

        # Wait for final IMG_DONE
        while True:
            t, p = self.link.recv_pkt(timeout=60)
            if t == RSP_IMG_DONE:
                return self._parse_img_done(p, 0)
            if t == RSP_ERROR:
                msg = p[2:].decode("ascii", errors="replace") \
                      if len(p) > 2 else ""
                raise RuntimeError(f"Error: {msg}")

    @staticmethod
    def _parse_img_done(payload, local_bytes):
        if len(payload) >= 13:
            blocks, files, byte_cnt = struct.unpack_from("<III", payload, 0)
            aborted = payload[12]
            return {
                "blocks": blocks,
                "files": files,
                "bytes": byte_cnt,
                "aborted": bool(aborted),
                "local_bytes": local_bytes,
            }
        return {"raw": payload.hex()}


# ---------------------------------------------------------------------------
# Hex dump utility
# ---------------------------------------------------------------------------

def hexdump(data, ebcdic=False, max_bytes=256):
    """Pretty-print a hex dump of data."""
    show = data[:max_bytes]
    for offset in range(0, len(show), 16):
        row = show[offset:offset+16]
        hexpart = " ".join(f"{b:02x}" for b in row)
        hexpart = hexpart.ljust(48)
        if ebcdic:
            ascpart = "".join(
                chr(EBCDIC_TABLE[b]) if 0x20 <= EBCDIC_TABLE[b] < 0x7F else "."
                for b in row
            )
        else:
            ascpart = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in row)
        print(f"  {offset:04x}: {hexpart}  {ascpart}")
    if len(data) > max_bytes:
        print(f"  ... ({len(data) - max_bytes} more bytes)")


# ---------------------------------------------------------------------------
# Interactive shell
# ---------------------------------------------------------------------------

class TapeShell(cmd.Cmd):
    intro = ("Tape Controller Host v2.0\n"
             'Type "help" for a list of commands.\n')
    prompt = "tape> "

    def __init__(self, ctrl: TapeController):
        super().__init__()
        self.ctrl = ctrl

    # --- helpers ---

    def _ok_or_err(self, result):
        ok, info = result
        if ok:
            print("OK")
        else:
            print(info)

    # --- commands ---

    def do_ping(self, _arg):
        """Test communication with the controller."""
        try:
            if self.ctrl.ping():
                print("Pong! Controller is responding.")
            else:
                print("No response.")
        except Exception as e:
            print(f"Error: {e}")

    def do_status(self, _arg):
        """Show detailed tape status."""
        try:
            raw, pos, labels = self.ctrl.status()
            print(f"\nTape Status = 0x{raw:04X}  Position = {pos}")
            if labels:
                for lbl in labels:
                    print(f"  {lbl}")
            else:
                print("  (no status bits set)")
            print()
        except Exception as e:
            print(f"Error: {e}")

    def do_init(self, _arg):
        """Initialize tape interface."""
        self._ok_or_err(self.ctrl.init_tape())

    def do_rewind(self, _arg):
        """Rewind tape."""
        print("Rewinding...")
        self._ok_or_err(self.ctrl.rewind())

    def do_unload(self, _arg):
        """Unload tape and go offline."""
        self._ok_or_err(self.ctrl.unload())

    def do_dump(self, arg):
        """Read and display tape block. Use 'dump E' for EBCDIC."""
        ebcdic = arg.strip().upper() == "E"
        try:
            ts, data = self.ctrl.read_block(ebcdic)
            flags = []
            if ts & TSTAT_TAPEMARK:  flags.append("TAPEMARK")
            if ts & TSTAT_HARDERR:   flags.append("HARD ERROR")
            if ts & TSTAT_CORRERR:   flags.append("CORRECTED")
            if ts & TSTAT_EOT:       flags.append("EOT")
            if ts & TSTAT_BLANK:     flags.append("BLANK")
            if ts & TSTAT_LENGTH:    flags.append("OVERRUN")
            status_str = ", ".join(flags) if flags else "OK"
            print(f"\nStatus: {status_str}  ({len(data)} bytes)")
            if data:
                hexdump(data, ebcdic)
            print()
        except Exception as e:
            print(f"Error: {e}")

    def do_skip(self, arg):
        """Skip n blocks (+=forward, -=backward). Default: 1."""
        try:
            n = int(arg) if arg.strip() else 1
        except ValueError:
            print("Usage: skip [count]")
            return
        self._ok_or_err(self.ctrl.skip(n))

    def do_space(self, arg):
        """Space n files (+=forward, -=backward). Default: 1."""
        try:
            n = int(arg) if arg.strip() else 1
        except ValueError:
            print("Usage: space [count]")
            return
        self._ok_or_err(self.ctrl.space(n))

    def do_address(self, arg):
        """Set tape drive address (0-7)."""
        try:
            addr = int(arg, 16) if arg.strip() else 0
        except ValueError:
            print("Usage: address <0-7>")
            return
        self._ok_or_err(self.ctrl.set_address(addr))
        print(f"Address set to {addr}")

    def do_retries(self, arg):
        """Set number of read retries (0-9)."""
        try:
            n = int(arg) if arg.strip() else 0
        except ValueError:
            print("Usage: retries <0-9>")
            return
        self._ok_or_err(self.ctrl.set_retries(n))
        print(f"Retries set to {n}")

    def do_stop(self, arg):
        """Set stop condition: number of filemarks, or V for EOV. Add E for stop-on-error.
        Examples: stop 2, stop V, stop 2 E"""
        parts = arg.strip().split()
        if not parts:
            mode = 2
            soe = False
        else:
            mode = parts[0]
            soe = len(parts) > 1 and parts[1].upper() == "E"
            if mode.upper() != "V":
                try:
                    mode = int(mode)
                except ValueError:
                    print("Usage: stop <count|V> [E]")
                    return
        self._ok_or_err(self.ctrl.set_stop(mode, soe))
        m_str = "EOV" if (isinstance(mode, str) and mode.upper() == "V") else str(mode)
        print(f"Stop after {m_str} tapemarks"
              f"{', or first error' if soe else ''}")

    def do_debug(self, arg):
        """Send raw command register value (hex)."""
        try:
            val = int(arg, 16) if arg.strip() else 0
        except ValueError:
            print("Usage: debug <hex value>")
            return
        self._ok_or_err(self.ctrl.debug_cmd(val))

    def do_1600(self, _arg):
        """Set 1600 PE density (tape must be at BOT)."""
        self._ok_or_err(self.ctrl.set_1600())

    def do_6250(self, _arg):
        """Set 6250 GCR density (tape must be at BOT)."""
        self._ok_or_err(self.ctrl.set_6250())

    def do_read(self, arg):
        """Read tape to image file.
        Usage: read <filename> [N]
        N = no rewind before/after."""
        parts = arg.strip().split()
        if not parts:
            print("Usage: read <filename> [N]")
            return
        filename = parts[0]
        no_rewind = len(parts) > 1 and parts[1].upper() == "N"
        try:
            print(f"Reading tape to {filename}...")
            result = self.ctrl.create_image(filename, no_rewind)
            print(f"\nDone. {result['blocks']} blocks, "
                  f"{result['files']} files, "
                  f"{result['bytes']} tape bytes")
            if result.get("aborted"):
                print("(Operation was aborted)")
            sz = os.path.getsize(filename) if os.path.exists(filename) else 0
            print(f"Image file: {filename} ({sz} bytes)")
        except KeyboardInterrupt:
            print("\nAborted by user")
        except Exception as e:
            print(f"Error: {e}")

    def do_write(self, arg):
        """Write image file to tape.
        Usage: write <filename> [N]
        N = no rewind before/after."""
        parts = arg.strip().split()
        if not parts:
            print("Usage: write <filename> [N]")
            return
        filename = parts[0]
        no_rewind = len(parts) > 1 and parts[1].upper() == "N"
        try:
            sz = os.path.getsize(filename)
            print(f"Writing {filename} ({sz} bytes) to tape...")
            result = self.ctrl.write_image(filename, no_rewind)
            print(f"\nDone. {result['blocks']} blocks, "
                  f"{result['files']} files, "
                  f"{result['bytes']} bytes written to tape")
            if result.get("aborted"):
                print("(Operation was aborted)")
        except FileNotFoundError:
            print(f"File not found: {filename}")
        except KeyboardInterrupt:
            print("\nAborted by user")
        except Exception as e:
            print(f"Error: {e}")

    def do_info(self, arg):
        """Show summary of a .TAP image file.
        Usage: info <filename>"""
        filename = arg.strip()
        if not filename:
            print("Usage: info <filename>")
            return
        try:
            self._show_tap_info(filename)
        except Exception as e:
            print(f"Error: {e}")

    @staticmethod
    def _show_tap_info(filename):
        """Parse and display info about a .TAP file."""
        with open(filename, "rb") as f:
            data = f.read()
        pos = 0
        block = 0
        files = 0
        total_bytes = 0
        last_len = None
        last_count = 0

        def flush_count():
            nonlocal last_len, last_count
            if last_count > 0 and last_len:
                print(f"  {last_count} x {last_len} bytes")
            last_len = None
            last_count = 0

        print(f"\n{filename}: {len(data)} bytes")

        while pos + 4 <= len(data):
            hdr = struct.unpack_from("<I", data, pos)[0]
            pos += 4

            if hdr == TAP_EOM:
                flush_count()
                print("  [End of Medium]")
                break

            if hdr == 0:
                flush_count()
                files += 1
                print(f"  [Filemark]  (file {files})")
                continue

            rec_len = hdr & TAP_LENGTH_MASK
            has_err = bool(hdr & TAP_ERROR_FLAG)
            err_str = " *ERROR*" if has_err else ""

            if pos + rec_len + 4 > len(data):
                flush_count()
                print(f"  [Truncated at block {block}]")
                break

            pos += rec_len  # skip data
            trailer = struct.unpack_from("<I", data, pos)[0]
            pos += 4

            block += 1
            total_bytes += rec_len

            if rec_len != last_len:
                flush_count()
                last_len = rec_len
                last_count = 1
            else:
                last_count += 1

            if has_err:
                print(f"  Block {block}: {rec_len} bytes{err_str}")

        flush_count()
        print(f"\nSummary: {block} blocks, {files} files, "
              f"{total_bytes} data bytes\n")

    def do_quit(self, _arg):
        """Exit the program."""
        print("Goodbye.")
        return True

    do_exit = do_quit
    do_EOF = do_quit

    def emptyline(self):
        pass


# ---------------------------------------------------------------------------
# Auto-detect serial port
# ---------------------------------------------------------------------------

def find_tape_port():
    """Try to find the tape controller USB CDC port."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if p.vid == 0x0483 and p.pid == 0x5740:
            return p.device
    # Fallback: look for ACM devices
    for p in ports:
        if "ACM" in p.device or "usbmodem" in p.device:
            return p.device
    return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Host application for Pertec tape controller")
    parser.add_argument("--port", "-p", default=None,
                        help="Serial port (default: auto-detect)")
    parser.add_argument("--timeout", "-t", type=int, default=10,
                        help="Serial timeout in seconds (default: 10)")
    args = parser.parse_args()

    port = args.port
    if port is None:
        port = find_tape_port()
        if port is None:
            print("Error: Could not find tape controller. "
                  "Use --port to specify.")
            sys.exit(1)

    print(f"Connecting to {port}...")
    try:
        link = TapeLink(port, timeout=args.timeout)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        sys.exit(1)

    ctrl = TapeController(link)

    # Verify connection
    try:
        if ctrl.ping():
            print("Controller connected and responding.\n")
        else:
            print("Warning: Controller did not respond to ping.\n")
    except Exception as e:
        print(f"Warning: Ping failed ({e}). Continuing anyway.\n")

    shell = TapeShell(ctrl)
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print("\nGoodbye.")
    finally:
        link.close()


if __name__ == "__main__":
    main()