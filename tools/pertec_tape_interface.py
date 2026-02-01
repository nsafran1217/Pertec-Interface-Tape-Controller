#!/usr/bin/env python3
"""
STM32 PERTEC Tape Interface - Host Application
Replicates the CLI tape utility functionality over USB
"""

import struct
import time
import sys
import os
import serial
import serial.tools.list_ports
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple, Callable

# TAP file format constants
TAP_ERROR_FLAG = 0x80000000
TAP_LENGTH_MASK = 0x00FFFFFF
TAP_EOM = 0xFFFFFFFF

# Protocol constants
CMD_SYNC, RESP_SYNC = 0xAA, 0x55
MAX_PAYLOAD = 4096

class Cmd(IntEnum):
    NOP=0x00; GET_INFO=0x01; GET_STATUS=0x02; RESET=0x03
    SET_CONFIG=0x10; GET_CONFIG=0x11; SET_DENSITY=0x12; SET_ADDRESS=0x13
    REWIND=0x20; UNLOAD=0x21; SKIP_BLOCK=0x22; SKIP_FILE=0x23
    READ_BLOCK=0x30; WRITE_BLOCK=0x31; WRITE_FILEMARK=0x32

class Resp(IntEnum):
    OK=0x00; ERR_UNKNOWN=0x01; ERR_CMD=0x02; ERR_PARAM=0x03
    ERR_OFFLINE=0x04; ERR_TIMEOUT=0x05; ERR_PROTECTED=0x06; ERR_CHECKSUM=0x07
    TAPEMARK=0x10; EOT=0x11; LOADPOINT=0x12; HARDERR=0x13
    CORRERR=0x14; BLANK=0x15; LENGTH=0x16; DATA=0x80

@dataclass
class DeviceInfo:
    protocol_ver: int; fw_major: int; fw_minor: int; fw_patch: int
    capabilities: int; max_payload: int; buffer_size: int

@dataclass
class TapeStatus:
    raw: int; online: bool; ready: bool; loadpoint: bool; eot: bool
    protected: bool; rewinding: bool; filemark: bool; error: bool; position: int

    def __str__(self):
        flags = []
        if self.online: flags.append("Online")
        if self.ready: flags.append("Ready")
        if self.loadpoint: flags.append("Load point")
        if self.eot: flags.append("EOT")
        if self.protected: flags.append("Protected")
        if self.rewinding: flags.append("Rewinding")
        return f"Status: {' '.join(flags) or 'Offline'} (pos={self.position})"

@dataclass
class Config:
    retries: int; stop_tapemarks: int; stop_on_error: bool
    density: int; tape_address: int; timeout_ms: int

class TapeError(Exception):
    def __init__(self, code: int, msg: str = ""):
        self.code = code
        super().__init__(msg or self._translate(code))

    @staticmethod
    def _translate(code):
        return {
            Resp.ERR_OFFLINE: "Drive is offline",
            Resp.ERR_PROTECTED: "Write protected",
            Resp.HARDERR: "Unrecoverable data error",
            Resp.CORRERR: "Corrected data error",
            Resp.TAPEMARK: "Tape mark hit",
            Resp.EOT: "End of tape",
            Resp.BLANK: "Blank tape",
            Resp.LENGTH: "Buffer overrun",
        }.get(code, f"Error 0x{code:02X}")

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = ((crc >> 8) | (crc << 8)) & 0xFFFF
        crc ^= b
        crc ^= (crc & 0xFF) >> 4
        crc ^= (crc << 12) & 0xFFFF
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF
    return crc

class TapeInterface:
    """USB interface to STM32 tape controller"""

    def __init__(self, port: str = None, timeout: float = 5.0):
        self.port = port
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self._seq = 0

    @staticmethod
    def find_device() -> Optional[str]:
        for p in serial.tools.list_ports.comports():
            if p.vid == 0x1209 and p.pid == 0x0001:
                return p.device
        for p in serial.tools.list_ports.comports():
            if 'ACM' in p.device or 'usbmodem' in p.device.lower():
                return p.device
        return None

    def open(self):
        if self.port is None:
            self.port = self.find_device()
        if not self.port:
            raise TapeError(0, "Device not found")
        self.ser = serial.Serial(self.port, 115200, timeout=self.timeout)
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None

    def __enter__(self): self.open(); return self
    def __exit__(self, *a): self.close()

    def _send(self, cmd: Cmd, payload: bytes = b'') -> Tuple[int, bytes]:
        if not self.ser:
            raise TapeError(0, "Not connected")
        self._seq = (self._seq + 1) & 0xFFFF
        hdr = struct.pack('<BBHH', CMD_SYNC, cmd, self._seq, len(payload))
        crc = crc16(hdr) ^ (crc16(payload) if payload else 0)
        self.ser.write(hdr + struct.pack('<H', crc) + payload)

        rhdr = self.ser.read(8)
        if len(rhdr) < 8:
            raise TapeError(0, f"Timeout (got {len(rhdr)} bytes)")
        sync, status, seq, length, rcrc = struct.unpack('<BBHHH', rhdr)
        if sync != RESP_SYNC:
            raise TapeError(0, f"Bad sync: 0x{sync:02X}")
        if seq != self._seq:
            raise TapeError(0, f"Seq mismatch")

        data = self.ser.read(length) if length else b''
        if len(data) < length:
            raise TapeError(0, "Timeout reading payload")

        calc = crc16(rhdr[:6]) ^ (crc16(data) if data else 0)
        if calc != rcrc:
            raise TapeError(0, "CRC error")
        return status, data

    def ping(self) -> bool:
        return self._send(Cmd.NOP)[0] == Resp.OK

    def get_info(self) -> DeviceInfo:
        _, d = self._send(Cmd.GET_INFO)
        v = struct.unpack('<BBBBIHHH', d[:14])
        return DeviceInfo(*v[:7])

    def get_status(self) -> TapeStatus:
        _, d = self._send(Cmd.GET_STATUS)
        raw, ol, rdy, lp, eot, prot, rwd, fm, err, pos = struct.unpack('<HBBBBBBBBL', d[:14])
        return TapeStatus(raw, bool(ol), bool(rdy), bool(lp), bool(eot),
                          bool(prot), bool(rwd), bool(fm), bool(err), pos)

    def get_config(self) -> Config:
        _, d = self._send(Cmd.GET_CONFIG)
        ret, stop, err, dens, addr, tmo = struct.unpack('<BBBBHH', d[:8])
        return Config(ret, stop, bool(err), dens, addr, tmo)

    def set_config(self, cfg: Config):
        payload = struct.pack('<BBBBHH', cfg.retries, cfg.stop_tapemarks,
                              1 if cfg.stop_on_error else 0, cfg.density,
                              cfg.tape_address, cfg.timeout_ms)
        st, _ = self._send(Cmd.SET_CONFIG, payload)
        if st != Resp.OK:
            raise TapeError(st)

    def set_density(self, density: int):
        """Set density: 1=1600 PE, 2=6250 GCR (must be at BOT)"""
        st, _ = self._send(Cmd.SET_DENSITY, bytes([density]))
        if st != Resp.OK:
            raise TapeError(st)

    def rewind(self):
        st, _ = self._send(Cmd.REWIND)
        if st != Resp.OK:
            raise TapeError(st)

    def unload(self):
        st, _ = self._send(Cmd.UNLOAD)
        if st != Resp.OK:
            raise TapeError(st)

    def skip_block(self, count: int = 1):
        """Skip blocks (negative = reverse)"""
        st, _ = self._send(Cmd.SKIP_BLOCK, struct.pack('<h', count))
        if st not in (Resp.OK, Resp.TAPEMARK, Resp.LOADPOINT):
            raise TapeError(st)
        return st

    def skip_file(self, count: int = 1):
        """Skip files (negative = reverse)"""
        st, _ = self._send(Cmd.SKIP_FILE, struct.pack('<h', count))
        if st not in (Resp.OK, Resp.EOT, Resp.LOADPOINT):
            raise TapeError(st)
        return st

    def read_block(self) -> Tuple[bytes, int]:
        """Read one block. Returns (data, flags) where flags: 1=corrected, 2=hard error"""
        st, d = self._send(Cmd.READ_BLOCK)
        if st == Resp.TAPEMARK:
            return b'', 0
        if st == Resp.DATA or st == Resp.OK:
            length, flags = struct.unpack('<IB', d[:5])
            return d[5:5+length], flags
        if st in (Resp.BLANK, Resp.EOT):
            raise TapeError(st)
        raise TapeError(st)

    def write_block(self, data: bytes):
        """Write one block"""
        st, _ = self._send(Cmd.WRITE_BLOCK, data)
        if st != Resp.OK:
            raise TapeError(st)

    def write_filemark(self):
        """Write a tape mark"""
        st, _ = self._send(Cmd.WRITE_FILEMARK)
        if st != Resp.OK:
            raise TapeError(st)

    # High-level operations matching tapeutil.c

    def create_image(self, filename: str, no_rewind: bool = False,
                     stop_tapemarks: int = 2, stop_on_error: bool = False,
                     stop_on_eov: bool = False,
                     progress: Callable = None) -> dict:
        """Read tape to TAP image file (CmdCreateImage equivalent)"""
        stats = {'blocks': 0, 'files': 0, 'bytes': 0, 'errors': 0}

        if not self.get_status().online:
            raise TapeError(Resp.ERR_OFFLINE)

        if not no_rewind:
            self.rewind()

        consecutive_marks = 0
        EOV_ASCII = b'EOV'
        EOV_EBCDIC = bytes([0xC5, 0xD6, 0xE5])

        with open(filename, 'wb') as f:
            while True:
                try:
                    data, flags = self.read_block()
                except TapeError as e:
                    if e.code == Resp.TAPEMARK:
                        # Write tapemark to file
                        f.write(struct.pack('<I', 0))
                        consecutive_marks += 1
                        stats['files'] += 1
                        if progress:
                            progress(stats, f"Tapemark at block {stats['blocks']}")
                        if consecutive_marks >= stop_tapemarks:
                            stats['files'] -= (stop_tapemarks - 1)
                            break
                        continue
                    elif e.code in (Resp.BLANK, Resp.EOT):
                        if progress:
                            progress(stats, "Blank tape or EOT hit")
                        break
                    elif e.code == Resp.HARDERR:
                        stats['errors'] += 1
                        if stop_on_error:
                            if progress:
                                progress(stats, "Stopping at error")
                            break
                        continue
                    else:
                        raise

                consecutive_marks = 0

                # Check for EOV stop condition
                if stop_on_eov and len(data) >= 3:
                    if data[:3] == EOV_ASCII or data[:3] == EOV_EBCDIC:
                        if progress:
                            progress(stats, "Hit EOV--ending")
                        # Still write this record
                        header = len(data)
                        f.write(struct.pack('<I', header))
                        f.write(data)
                        f.write(struct.pack('<I', header))
                        stats['blocks'] += 1
                        stats['bytes'] += len(data)
                        break

                # Build TAP header
                header = len(data)
                if flags & 0x02:  # Hard error
                    header |= TAP_ERROR_FLAG
                    stats['errors'] += 1
                if flags & 0x01:  # Corrected error
                    if progress:
                        progress(stats, f"Corrected error at block {stats['blocks']}")

                # Write record
                f.write(struct.pack('<I', header))
                if data:
                    f.write(data)
                    f.write(struct.pack('<I', header))

                stats['blocks'] += 1
                stats['bytes'] += len(data)

                if progress:
                    progress(stats, None)

            # Write EOM marker
            f.write(struct.pack('<I', TAP_EOM))

        if not no_rewind:
            self.rewind()

        return stats

    def write_image(self, filename: str, no_rewind: bool = False,
                    progress: Callable = None) -> dict:
        """Write TAP image file to tape (CmdWriteImage equivalent)"""
        stats = {'blocks': 0, 'files': 0, 'bytes': 0}

        status = self.get_status()
        if not status.online:
            raise TapeError(Resp.ERR_OFFLINE)
        if status.protected:
            raise TapeError(Resp.ERR_PROTECTED)

        if not no_rewind:
            self.rewind()

        with open(filename, 'rb') as f:
            while True:
                hdr_data = f.read(4)
                if len(hdr_data) < 4:
                    break

                header = struct.unpack('<I', hdr_data)[0]

                if header == TAP_EOM:
                    break

                if header == 0:
                    # Tapemark
                    self.write_filemark()
                    stats['files'] += 1
                    if progress:
                        progress(stats, "Wrote tapemark")
                    continue

                length = header & TAP_LENGTH_MASK
                if length > MAX_PAYLOAD:
                    raise TapeError(0, f"Block too large at {stats['blocks']}")

                data = f.read(length)
                if len(data) < length:
                    raise TapeError(0, f"File truncated at block {stats['blocks']}")

                trailer = struct.unpack('<I', f.read(4))[0]
                if trailer != header:
                    raise TapeError(0, f"Corrupt TAP file at block {stats['blocks']}")

                self.write_block(data)
                stats['blocks'] += 1
                stats['bytes'] += length

                if progress:
                    progress(stats, None)

        if not no_rewind:
            self.rewind()

        return stats


def show_buffer(data: bytes, max_display: int = 256, ebcdic: bool = False):
    """Display buffer contents (hex + ASCII/EBCDIC)"""
    EBCDIC_TO_ASCII = bytes([
        0x00,0x01,0x02,0x03,0x1A,0x09,0x1A,0x7F,0x1A,0x1A,0x1A,0x0B,0x0C,0x0D,0x0E,0x0F,
        0x10,0x11,0x12,0x13,0x1A,0x0A,0x08,0x1A,0x18,0x19,0x1A,0x1A,0x1C,0x1D,0x1E,0x1F,
        0x1A,0x1A,0x1C,0x1A,0x1A,0x0A,0x17,0x1B,0x1A,0x1A,0x1A,0x1A,0x1A,0x05,0x06,0x07,
        0x1A,0x1A,0x16,0x1A,0x1A,0x1E,0x1A,0x04,0x1A,0x1A,0x1A,0x1A,0x14,0x15,0x1A,0x1A,
        0x20,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x2E,0x3C,0x28,0x2B,0x7C,
        0x26,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x21,0x24,0x2A,0x29,0x3B,0x5E,
        0x2D,0x2F,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x2C,0x25,0x5F,0x3E,0x3F,
        0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x60,0x3A,0x23,0x40,0x27,0x3D,0x22,
        0x1A,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,
        0x1A,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,0x70,0x71,0x72,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,
        0x1A,0x7E,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x1A,0x1A,0x1A,0x5B,0x1A,0x1A,
        0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,0x5D,0x1A,0x1A,
        0x7B,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,
        0x7D,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,
        0x5C,0x1A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,
        0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x1A,0x1A,0x1A,0x1A,0x1A,0x1A,
    ])

    display = data[:max_display]
    for i in range(0, len(display), 16):
        chunk = display[i:i+16]
        hexpart = ' '.join(f'{b:02X}' for b in chunk)
        if ebcdic:
            ascpart = ''.join(chr(EBCDIC_TO_ASCII[b]) if 32 <= EBCDIC_TO_ASCII[b] < 127 else '.' for b in chunk)
        else:
            ascpart = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        print(f'{i:04X}: {hexpart:<48} {ascpart}')


def main():
    import argparse

    parser = argparse.ArgumentParser(description='PERTEC Tape Interface')
    parser.add_argument('-p', '--port', help='Serial port')
    parser.add_argument('-v', '--verbose', action='store_true')

    sub = parser.add_subparsers(dest='cmd')
    sub.add_parser('ping', help='Test connection')
    sub.add_parser('info', help='Device info')
    sub.add_parser('status', help='Tape status')
    sub.add_parser('rewind', help='Rewind tape')
    sub.add_parser('unload', help='Unload tape')

    p = sub.add_parser('read', help='Read and display block')
    p.add_argument('-e', '--ebcdic', action='store_true', help='EBCDIC display')

    p = sub.add_parser('skip', help='Skip blocks')
    p.add_argument('count', type=int, nargs='?', default=1)

    p = sub.add_parser('space', help='Skip files')
    p.add_argument('count', type=int, nargs='?', default=1)

    p = sub.add_parser('density', help='Set density')
    p.add_argument('mode', choices=['1600', '6250'])

    p = sub.add_parser('config', help='Show/set config')
    p.add_argument('--retries', type=int)
    p.add_argument('--stop', type=int, help='Stop after N tapemarks')
    p.add_argument('--stop-error', action='store_true')

    p = sub.add_parser('create', help='Read tape to image file')
    p.add_argument('filename', help='Output TAP file')
    p.add_argument('-n', '--no-rewind', action='store_true')
    p.add_argument('-s', '--stop', type=int, default=2, help='Stop after N tapemarks')
    p.add_argument('-e', '--stop-error', action='store_true')
    p.add_argument('-V', '--stop-eov', action='store_true', help='Stop on EOV label')

    p = sub.add_parser('write', help='Write image file to tape')
    p.add_argument('filename', help='Input TAP file')
    p.add_argument('-n', '--no-rewind', action='store_true')

    args = parser.parse_args()
    if not args.cmd:
        parser.print_help()
        return

    def progress(stats, msg):
        if msg:
            print(f"\r{msg:<60}")
        else:
            print(f"\rBlocks: {stats['blocks']:6d}  Files: {stats['files']:3d}  "
                  f"Bytes: {stats['bytes']:10d}", end='', flush=True)

    try:
        with TapeInterface(args.port) as dev:
            if args.cmd == 'ping':
                print("OK" if dev.ping() else "FAIL")

            elif args.cmd == 'info':
                i = dev.get_info()
                print(f"Firmware: v{i.fw_major}.{i.fw_minor}.{i.fw_patch}")
                print(f"Protocol: v{i.protocol_ver}")
                print(f"Max payload: {i.max_payload}, Buffer: {i.buffer_size}")

            elif args.cmd == 'status':
                print(dev.get_status())

            elif args.cmd == 'rewind':
                dev.rewind()
                print("Rewinding...")

            elif args.cmd == 'unload':
                dev.unload()
                print("Unloaded")

            elif args.cmd == 'read':
                try:
                    data, flags = dev.read_block()
                    st = dev.get_status()
                    print(f"{len(data)} bytes in block {st.position}:")
                    if data:
                        show_buffer(data, ebcdic=args.ebcdic)
                except TapeError as e:
                    print(str(e))

            elif args.cmd == 'skip':
                dev.skip_block(args.count)
                print(f"Skipped {args.count} block(s)")

            elif args.cmd == 'space':
                dev.skip_file(args.count)
                print(f"Spaced {args.count} file(s)")

            elif args.cmd == 'density':
                dev.set_density(1 if args.mode == '1600' else 2)
                print(f"Set density to {args.mode}")

            elif args.cmd == 'config':
                cfg = dev.get_config()
                if args.retries is not None:
                    cfg.retries = args.retries
                if args.stop is not None:
                    cfg.stop_tapemarks = args.stop
                if args.stop_error:
                    cfg.stop_on_error = True
                if any([args.retries, args.stop, args.stop_error]):
                    dev.set_config(cfg)
                    print("Configuration updated")
                print(f"Retries: {cfg.retries}")
                print(f"Stop after: {cfg.stop_tapemarks} tapemarks")
                print(f"Stop on error: {cfg.stop_on_error}")

            elif args.cmd == 'create':
                print(f"Reading tape to {args.filename}...")
                stats = dev.create_image(args.filename, args.no_rewind,
                                         args.stop, args.stop_error, args.stop_eov,
                                         progress if args.verbose else None)
                print(f"\n\nFile {args.filename} written.")
                print(f"{stats['blocks']} blocks, {stats['files']} files, "
                      f"{stats['bytes']} bytes, {stats['errors']} errors")

                # Prompt for optional comment (like GetComment in original)
                try:
                    comment = input("\nEnter a comment for this tape, or press Enter for none:\n")
                    if comment.strip():
                        comment_file = args.filename + '.txt'
                        with open(comment_file, 'w') as cf:
                            cf.write(comment.strip() + '\n')
                        print(f"Comment saved to {comment_file}")
                except (EOFError, KeyboardInterrupt):
                    pass

            elif args.cmd == 'write':
                print(f"Writing {args.filename} to tape...")
                stats = dev.write_image(args.filename, args.no_rewind,
                                        progress if args.verbose else None)
                print(f"\n\nFile {args.filename} written to tape.")
                print(f"{stats['blocks']} blocks, {stats['files']} files, "
                      f"{stats['bytes']} bytes")

    except TapeError as e:
        print(f"Error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nAborted")
        return 1

if __name__ == '__main__':
    sys.exit(main() or 0)
