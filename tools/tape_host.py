#!/usr/bin/env python3
"""
tape_host.py – Host application for the Pertec tape controller.

Communicates with the STM32 MCU over USB CDC using a binary packet protocol.

Requirements: pyserial  (pip install pyserial)

Usage:
    python tape_host.py [--port /dev/ttyACM0] <command> [args...]

Commands:
    status                          Show detailed tape status
    init                            Initialize tape interface
    rewind                          Rewind tape
    unload                          Unload tape and go offline
    dump [-e]                       Read and display one tape block (-e for EBCDIC)
    skip <count>                    Skip n blocks (negative = backward)
    space <count>                   Space n files (negative = backward)
    address <0-7>                   Set tape drive address
    retries <0-9>                   Set number of read retries
    stop <count|V> [-e]             Set stop condition (-e = stop on error)
    debug <hex>                     Send raw command register value
    1600                            Set 1600 PE density (tape must be at BOT)
    6250                            Set 6250 GCR density (tape must be at BOT)
    read <file> [-n]                Read tape to .TAP image (-n = no rewind)
    write <file> [-n]               Write .TAP image to tape (-n = no rewind)
    info <file>                     Show summary of a .TAP image file (offline)
    ping                            Test communication with the controller
"""

import argparse
import os
import struct
import sys
import time

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# Protocol constants (must match protocol.h on MCU)
# ---------------------------------------------------------------------------

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

RSP_OK          = 0x80
RSP_ERROR       = 0x81
RSP_STATUS      = 0x82
RSP_BLOCK_DATA  = 0x83
RSP_MSG         = 0x84
RSP_IMG_DATA    = 0x85
RSP_IMG_DONE    = 0x86
RSP_PONG        = 0x88
RSP_READY       = 0x89

FLAG_NO_REWIND  = 0x01
FLAG_EBCDIC     = 0x02

TSTAT_OFFLINE   = 0x80
TSTAT_HARDERR   = 0x40
TSTAT_CORRERR   = 0x20
TSTAT_TAPEMARK  = 0x10
TSTAT_EOT       = 0x08
TSTAT_BLANK     = 0x04
TSTAT_LENGTH    = 0x02
TSTAT_PROTECT   = 0x01

PS0_IRP     = 1;      PS0_IDBY    = 2;      PS0_ISPEED  = 4
PS0_RDAVAIL = 8;      PS0_WREMPTY = 16;     PS0_IFMK    = 32
PS0_IHER    = 64;     PS0_ICER    = 128
PS1_INRZ    = 1 << 8; PS1_EOT     = 2 << 8; PS1_IONL    = 4 << 8
PS1_IFPT    = 8 << 8; PS1_IRWD    = 16 << 8; PS1_ILDP   = 32 << 8
PS1_IRDY    = 64 << 8; PS1_IFBY   = 128 << 8

STATUS_LABELS = [
    (PS0_IRP,    "Odd"),       (PS0_IDBY,   "Data busy"),
    (PS0_ISPEED, "High speed"),(PS0_RDAVAIL,"Read full"),
    (PS0_WREMPTY,"Write empty"),(PS0_IFMK,  "Tape mark"),
    (PS0_IHER,   "Hard error"),(PS0_ICER,   "Soft error"),
    (PS1_INRZ,   "NRZI mode"),(PS1_EOT,    "EOT"),
    (PS1_IONL,   "Online"),   (PS1_IFPT,   "Protected"),
    (PS1_IRWD,   "Rewinding"),(PS1_ILDP,   "Load point"),
    (PS1_IRDY,   "Ready"),
]

TAP_EOM         = 0xFFFFFFFF
TAP_ERROR_FLAG  = 0x80000000
TAP_LENGTH_MASK = 0x00FFFFFF
PKT_HDR_SIZE    = 5

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
        """Receive one packet.  Returns (type, payload_bytes).
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
        self.link.send_pkt(CMD_STATUS)
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

    def init_tape(self):       return self._simple_cmd(CMD_INIT)
    def rewind(self):          return self._simple_cmd(CMD_REWIND, timeout=120)
    def unload(self):          return self._simple_cmd(CMD_UNLOAD, timeout=120)

    def skip(self, count):
        return self._simple_cmd(CMD_SKIP, struct.pack("<h", count), timeout=30)

    def space(self, count):
        return self._simple_cmd(CMD_SPACE, struct.pack("<h", count), timeout=30)

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

    def set_1600(self):  return self._simple_cmd(CMD_SET_1600)
    def set_6250(self):  return self._simple_cmd(CMD_SET_6250)

    def read_block(self, ebcdic=False):
        flags = FLAG_EBCDIC if ebcdic else 0
        self.link.send_pkt(CMD_READ_FWD, bytes([flags]))
        t, p = self.link.recv_pkt(timeout=30)
        if t == RSP_BLOCK_DATA and len(p) >= 2:
            return struct.unpack_from("<H", p, 0)[0], p[2:]
        if t == RSP_ERROR:
            code = struct.unpack_from("<H", p, 0)[0] if len(p) >= 2 else 0
            msg = p[2:].decode("ascii", errors="replace") if len(p) > 2 else ""
            raise RuntimeError(f"Read error 0x{code:04X}: {msg}")
        raise RuntimeError(f"Unexpected response 0x{t:02X}")

    def create_image(self, filename, no_rewind=False):
        flags = FLAG_NO_REWIND if no_rewind else 0
        self.link.send_pkt(CMD_CREATE_IMG, bytes([flags]))
        t, p = self.link.recv_pkt(timeout=120)
        if t == RSP_ERROR:
            msg = p[2:].decode("ascii", errors="replace") if len(p) > 2 else ""
            raise RuntimeError(f"Create image failed: {msg}")
        if t != RSP_OK:
            raise RuntimeError(f"Unexpected response 0x{t:02X}")

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
        except KeyboardInterrupt:
            print("\nSending abort...")
            self.link.send_pkt(CMD_ABORT)
            while True:
                t, p = self.link.recv_pkt(timeout=30)
                if t == RSP_IMG_DONE:
                    return self._parse_img_done(p, bytes_written)

    def write_image(self, filename, no_rewind=False):
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

        # MCU drives the flow: it sends RSP_READY when it needs data,
        # we respond with a CMD_FILE_DATA chunk (or CMD_FILE_EOF).
        # Large chunks (16KB) keep the MCU's InQ primed so the next
        # tape record is ready as soon as TapeWrite finishes.
        CHUNK = 12288
        eof_sent = False
        try:
            with open(filename, "rb") as f:
                while True:
                    t, p = self.link.recv_pkt(timeout=300)
                    if t == RSP_READY:
                        if eof_sent:
                            continue   # drain any extra RSP_READY
                        chunk = f.read(CHUNK)
                        if chunk:
                            self.link.send_pkt(CMD_FILE_DATA, chunk)
                        else:
                            self.link.send_pkt(CMD_FILE_EOF)
                            eof_sent = True
                    elif t == RSP_IMG_DONE:
                        return self._parse_img_done(p, 0)
                    elif t == RSP_ERROR:
                        msg = p[2:].decode("ascii", errors="replace") \
                              if len(p) > 2 else ""
                        raise RuntimeError(f"Error during write: {msg}")
                    # RSP_MSG is handled transparently by recv_pkt
        except KeyboardInterrupt:
            print("\nSending abort...")
            self.link.send_pkt(CMD_ABORT)
            while True:
                t, p = self.link.recv_pkt(timeout=30)
                if t == RSP_IMG_DONE:
                    return self._parse_img_done(p, 0)

    @staticmethod
    def _parse_img_done(payload, local_bytes):
        if len(payload) >= 13:
            blocks, files, byte_cnt = struct.unpack_from("<III", payload, 0)
            aborted = payload[12]
            return {
                "blocks": blocks, "files": files, "bytes": byte_cnt,
                "aborted": bool(aborted), "local_bytes": local_bytes,
            }
        return {"raw": payload.hex()}


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def hexdump(data, ebcdic=False, max_bytes=256):
    show = data[:max_bytes]
    for offset in range(0, len(show), 16):
        row = show[offset:offset+16]
        hexpart = " ".join(f"{b:02x}" for b in row).ljust(48)
        if ebcdic:
            ascpart = "".join(
                chr(EBCDIC_TABLE[b]) if 0x20 <= EBCDIC_TABLE[b] < 0x7F else "."
                for b in row)
        else:
            ascpart = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in row)
        print(f"  {offset:04x}: {hexpart}  {ascpart}")
    if len(data) > max_bytes:
        print(f"  ... ({len(data) - max_bytes} more bytes)")


def show_tap_info(filename):
    with open(filename, "rb") as f:
        data = f.read()
    pos = 0; block = 0; files = 0; total_bytes = 0
    last_len = None; last_count = 0

    def flush():
        nonlocal last_len, last_count
        if last_count > 0 and last_len:
            print(f"  {last_count} x {last_len} bytes")
        last_len = None; last_count = 0

    print(f"\n{filename}: {len(data)} bytes")
    while pos + 4 <= len(data):
        hdr = struct.unpack_from("<I", data, pos)[0]; pos += 4
        if hdr == TAP_EOM:
            flush(); print("  [End of Medium]"); break
        if hdr == 0:
            flush(); files += 1; print(f"  [Filemark]  (file {files})"); continue
        rec_len = hdr & TAP_LENGTH_MASK
        has_err = bool(hdr & TAP_ERROR_FLAG)
        if pos + rec_len + 4 > len(data):
            flush(); print(f"  [Truncated at block {block}]"); break
        pos += rec_len
        pos += 4  # trailer
        block += 1; total_bytes += rec_len
        if rec_len != last_len:
            flush(); last_len = rec_len; last_count = 1
        else:
            last_count += 1
        if has_err:
            print(f"  Block {block}: {rec_len} bytes *ERROR*")
    flush()
    print(f"\nSummary: {block} blocks, {files} files, {total_bytes} data bytes\n")


def find_tape_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if p.vid == 0x0483 and p.pid == 0x5740:
            return p.device
    for p in ports:
        if "ACM" in p.device or "usbmodem" in p.device:
            return p.device
    return None


# ---------------------------------------------------------------------------
# Command handlers
# ---------------------------------------------------------------------------

def ok_or_die(result):
    ok, info = result
    if ok:
        print("OK")
    else:
        print(info, file=sys.stderr)
        sys.exit(1)


def cmd_ping(ctrl, args):
    if ctrl.ping():
        print("Pong! Controller is responding.")
    else:
        print("No response.", file=sys.stderr); sys.exit(1)

def cmd_status(ctrl, args):
    raw, pos, labels = ctrl.status()
    print(f"Tape Status = 0x{raw:04X}  Position = {pos}")
    for lbl in labels:
        print(f"  {lbl}")

def cmd_init(ctrl, args):
    ok_or_die(ctrl.init_tape())

def cmd_rewind(ctrl, args):
    print("Rewinding...")
    ok_or_die(ctrl.rewind())

def cmd_unload(ctrl, args):
    ok_or_die(ctrl.unload())

def cmd_dump(ctrl, args):
    p = argparse.ArgumentParser(prog="dump")
    p.add_argument("-e", action="store_true", help="EBCDIC display")
    a = p.parse_args(args)
    ts, data = ctrl.read_block(ebcdic=a.e)
    flags = []
    if ts & TSTAT_TAPEMARK:  flags.append("TAPEMARK")
    if ts & TSTAT_HARDERR:   flags.append("HARD ERROR")
    if ts & TSTAT_CORRERR:   flags.append("CORRECTED")
    if ts & TSTAT_EOT:       flags.append("EOT")
    if ts & TSTAT_BLANK:     flags.append("BLANK")
    if ts & TSTAT_LENGTH:    flags.append("OVERRUN")
    print(f"Status: {', '.join(flags) or 'OK'}  ({len(data)} bytes)")
    if data:
        hexdump(data, a.e)

def cmd_skip(ctrl, args):
    p = argparse.ArgumentParser(prog="skip")
    p.add_argument("count", type=int, help="Blocks to skip (negative = backward)")
    a = p.parse_args(args)
    ok_or_die(ctrl.skip(a.count))

def cmd_space(ctrl, args):
    p = argparse.ArgumentParser(prog="space")
    p.add_argument("count", type=int, help="Files to space (negative = backward)")
    a = p.parse_args(args)
    ok_or_die(ctrl.space(a.count))

def cmd_address(ctrl, args):
    p = argparse.ArgumentParser(prog="address")
    p.add_argument("addr", type=lambda x: int(x, 16), help="Address 0-7 (hex)")
    a = p.parse_args(args)
    ok_or_die(ctrl.set_address(a.addr))
    print(f"Address set to {a.addr}")

def cmd_retries(ctrl, args):
    p = argparse.ArgumentParser(prog="retries")
    p.add_argument("count", type=int, help="Retry count 0-9")
    a = p.parse_args(args)
    ok_or_die(ctrl.set_retries(a.count))
    print(f"Retries set to {a.count}")

def cmd_stop(ctrl, args):
    p = argparse.ArgumentParser(prog="stop")
    p.add_argument("mode", help="Number of filemarks, or V for EOV")
    p.add_argument("-e", action="store_true", help="Stop on first error")
    a = p.parse_args(args)
    mode = a.mode
    if mode.upper() != "V":
        try:
            mode = int(mode)
        except ValueError:
            print("Error: mode must be a number or V", file=sys.stderr)
            sys.exit(1)
    ok_or_die(ctrl.set_stop(mode, a.e))
    m_str = "EOV" if (isinstance(mode, str) and mode.upper() == "V") else str(mode)
    print(f"Stop after {m_str} tapemarks{', or first error' if a.e else ''}")

def cmd_debug(ctrl, args):
    p = argparse.ArgumentParser(prog="debug")
    p.add_argument("value", type=lambda x: int(x, 16), help="Hex value")
    a = p.parse_args(args)
    ok_or_die(ctrl.debug_cmd(a.value))

def cmd_1600(ctrl, args):
    ok_or_die(ctrl.set_1600())

def cmd_6250(ctrl, args):
    ok_or_die(ctrl.set_6250())

def cmd_read(ctrl, args):
    p = argparse.ArgumentParser(prog="read")
    p.add_argument("file", help="Output .TAP filename")
    p.add_argument("-n", action="store_true", help="No rewind before/after")
    a = p.parse_args(args)
    print(f"Reading tape to {a.file}...")
    result = ctrl.create_image(a.file, no_rewind=a.n)
    print(f"Done. {result['blocks']} blocks, {result['files']} files, "
          f"{result['bytes']} tape bytes")
    if result.get("aborted"):
        print("(Operation was aborted)")
    if os.path.exists(a.file):
        print(f"Image file: {a.file} ({os.path.getsize(a.file)} bytes)")

def cmd_write(ctrl, args):
    p = argparse.ArgumentParser(prog="write")
    p.add_argument("file", help="Input .TAP filename")
    p.add_argument("-n", action="store_true", help="No rewind before/after")
    a = p.parse_args(args)
    sz = os.path.getsize(a.file)
    print(f"Writing {a.file} ({sz} bytes) to tape...")
    result = ctrl.write_image(a.file, no_rewind=a.n)
    print(f"Done. {result['blocks']} blocks, {result['files']} files, "
          f"{result['bytes']} bytes written to tape")
    if result.get("aborted"):
        print("(Operation was aborted)")

def cmd_info(ctrl, args):
    p = argparse.ArgumentParser(prog="info")
    p.add_argument("file", help=".TAP filename")
    a = p.parse_args(args)
    show_tap_info(a.file)


COMMANDS = {
    "ping":    cmd_ping,
    "status":  cmd_status,
    "init":    cmd_init,
    "rewind":  cmd_rewind,
    "unload":  cmd_unload,
    "dump":    cmd_dump,
    "skip":    cmd_skip,
    "space":   cmd_space,
    "address": cmd_address,
    "retries": cmd_retries,
    "stop":    cmd_stop,
    "debug":   cmd_debug,
    "1600":    cmd_1600,
    "6250":    cmd_6250,
    "read":    cmd_read,
    "write":   cmd_write,
    "info":    cmd_info,
}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Host application for Pertec tape controller",
        usage="%(prog)s [--port PORT] <command> [command-args...]")
    parser.add_argument("--port", "-p", default=None,
                        help="Serial port (default: auto-detect)")
    parser.add_argument("--timeout", "-t", type=int, default=10,
                        help="Serial timeout in seconds (default: 10)")
    parser.add_argument("command", help="Command to execute")
    parser.add_argument("args", nargs=argparse.REMAINDER,
                        help="Command arguments")
    a = parser.parse_args()

    cmd_name = a.command.lower()
    handler = COMMANDS.get(cmd_name)
    if handler is None:
        print(f"Unknown command: {a.command}", file=sys.stderr)
        print(f"Available commands: {', '.join(sorted(COMMANDS))}", file=sys.stderr)
        sys.exit(1)

    # "info" is offline-only, no connection needed
    if cmd_name == "info":
        handler(None, a.args)
        return

    port = a.port
    if port is None:
        port = find_tape_port()
        if port is None:
            print("Error: Could not find tape controller. "
                  "Use --port to specify.", file=sys.stderr)
            sys.exit(1)

    try:
        link = TapeLink(port, timeout=a.timeout)
    except serial.SerialException as e:
        print(f"Error opening port: {e}", file=sys.stderr)
        sys.exit(1)

    ctrl = TapeController(link)

    try:
        handler(ctrl, a.args)
    except KeyboardInterrupt:
        print("\nAborted by user")
        sys.exit(130)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        link.close()


if __name__ == "__main__":
    main()