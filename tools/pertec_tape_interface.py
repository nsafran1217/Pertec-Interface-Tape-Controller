#!/usr/bin/env python3
"""
PERTEC Tape Interface - Host Application

Sends commands to MCU and streams TAP file data.
The MCU handles TAP file assembly/parsing.
"""

import struct
import time
import sys
import os
import serial
import serial.tools.list_ports
from typing import Optional, BinaryIO

# Framing constants
FRAME_START = 0xAA
FRAME_END   = 0x55
FRAME_ESC   = 0x1B

# Command codes
CMD_NOP          = 0x00
CMD_STATUS       = 0x01
CMD_REWIND       = 0x02
CMD_UNLOAD       = 0x03
CMD_SKIP         = 0x04
CMD_SPACE        = 0x05
CMD_READ_BLOCK   = 0x06
CMD_SET_DENSITY  = 0x07
CMD_CREATE_IMAGE = 0x10
CMD_WRITE_IMAGE  = 0x11
CMD_ABORT        = 0xFF

# Response codes
RESP_OK        = 0x00
RESP_ERR       = 0x01
RESP_BUSY      = 0x02
RESP_DATA      = 0x10
RESP_DONE      = 0x20
RESP_NEED_DATA = 0x30


class TapeError(Exception):
    pass


class TapeInterface:
    def __init__(self, port: str = None, timeout: float = 10.0):
        self.port = port
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
    
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
            raise TapeError("Device not found")
        self.ser = serial.Serial(self.port, 115200, timeout=self.timeout)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
    
    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None
    
    def __enter__(self):
        self.open()
        return self
    
    def __exit__(self, *a):
        self.close()
    
    def _escape_data(self, data: bytes) -> bytes:
        """Escape special bytes in data"""
        result = bytearray()
        for b in data:
            if b in (FRAME_START, FRAME_END, FRAME_ESC):
                result.append(FRAME_ESC)
                result.append(b ^ 0x20)
            else:
                result.append(b)
        return bytes(result)
    
    def _unescape_data(self, data: bytes) -> bytes:
        """Unescape data"""
        result = bytearray()
        i = 0
        while i < len(data):
            if data[i] == FRAME_ESC and i + 1 < len(data):
                result.append(data[i + 1] ^ 0x20)
                i += 2
            else:
                result.append(data[i])
                i += 1
        return bytes(result)
    
    def _send_packet(self, ptype: int, data: bytes = b''):
        """Send a framed packet"""
        escaped = self._escape_data(data)
        pkt = bytes([FRAME_START, ptype, len(data) & 0xFF, (len(data) >> 8) & 0xFF])
        pkt += escaped
        pkt += bytes([FRAME_END])
        self.ser.write(pkt)
    
    def _recv_packet(self, timeout: float = None) -> tuple:
        """Receive a framed packet. Returns (type, data) or (None, None) on timeout"""
        if timeout:
            old_timeout = self.ser.timeout
            self.ser.timeout = timeout
        
        try:
            # Wait for start byte
            while True:
                b = self.ser.read(1)
                if not b:
                    return (None, None)
                if b[0] == FRAME_START:
                    break
            
            # Read type and length
            hdr = self.ser.read(3)
            if len(hdr) < 3:
                return (None, None)
            
            ptype = hdr[0]
            length = hdr[1] | (hdr[2] << 8)
            
            # Read data until END (accounting for escapes)
            data = bytearray()
            while True:
                b = self.ser.read(1)
                if not b:
                    return (None, None)
                if b[0] == FRAME_END:
                    break
                data.append(b[0])
            
            return (ptype, self._unescape_data(bytes(data)))
        finally:
            if timeout:
                self.ser.timeout = old_timeout
    
    def _send_command(self, cmd: int, params: bytes = b'') -> tuple:
        """Send command and wait for response"""
        self._send_packet(cmd, params)
        return self._recv_packet()
    
    def ping(self) -> bool:
        rtype, _ = self._send_command(CMD_NOP)
        return rtype == RESP_OK
    
    def get_status(self) -> dict:
        rtype, data = self._send_command(CMD_STATUS)
        if rtype != RESP_OK or len(data) < 14:
            return {'error': True}
        
        raw, pos, copied, online, ready, lp, eot, prot, rwd = struct.unpack('<HIIBBBBB', data[:14])
        return {
            'raw_status': raw,
            'position': pos,
            'bytes_copied': copied,
            'online': bool(online),
            'ready': bool(ready),
            'loadpoint': bool(lp),
            'eot': bool(eot),
            'protected': bool(prot),
            'rewinding': bool(rwd),
        }
    
    def rewind(self):
        rtype, _ = self._send_command(CMD_REWIND)
        if rtype != RESP_OK:
            raise TapeError("Rewind failed")
    
    def unload(self):
        rtype, _ = self._send_command(CMD_UNLOAD)
        if rtype != RESP_OK:
            raise TapeError("Unload failed")
    
    def skip(self, count: int):
        params = struct.pack('<h', count)
        rtype, _ = self._send_command(CMD_SKIP, params)
        if rtype != RESP_OK:
            raise TapeError("Skip failed")
    
    def space(self, count: int):
        params = struct.pack('<h', count)
        rtype, _ = self._send_command(CMD_SPACE, params)
        if rtype != RESP_OK:
            raise TapeError("Space failed")
    
    def set_density(self, density: int):
        """Set density: 1=1600, 2=6250"""
        rtype, _ = self._send_command(CMD_SET_DENSITY, bytes([density]))
        if rtype != RESP_OK:
            raise TapeError("Set density failed (must be at BOT)")
    
    def read_block(self) -> tuple:
        """Read single block. Returns (status, data)"""
        rtype, hdr = self._send_command(CMD_READ_BLOCK)
        if rtype != RESP_DATA or len(hdr) < 6:
            raise TapeError("Read block failed")
        
        status = hdr[0] | (hdr[1] << 8)
        length = hdr[2] | (hdr[3] << 8) | (hdr[4] << 16) | (hdr[5] << 24)
        
        # Read the data packet
        if length > 0:
            rtype, data = self._recv_packet()
            if rtype is None:
                raise TapeError("Timeout reading block data")
            return (status, data[:length])
        
        return (status, b'')
    
    def create_image(self, filename: str, stop_tapemarks: int = 2,
                     stop_on_error: bool = False, no_rewind: bool = False,
                     progress=None) -> dict:
        """Read tape and save TAP image to file"""
        stats = {'blocks': 0, 'bytes': 0}
        
        # Send create_image command
        params = bytes([
            stop_tapemarks if stop_tapemarks != 'V' else ord('V'),
            1 if stop_on_error else 0,
            1 if no_rewind else 0
        ])
        self._send_packet(CMD_CREATE_IMAGE, params)
        
        # Wait for OK response (transfer starting)
        rtype, data = self._recv_packet()
        if rtype != RESP_OK:
            raise TapeError("Create image failed to start")
        
        # Open output file
        with open(filename, 'wb') as f:
            # Receive data packets until DONE
            while True:
                rtype, data = self._recv_packet(timeout=30.0)
                
                if rtype is None:
                    raise TapeError("Timeout waiting for data")
                
                if rtype == RESP_DONE:
                    break
                
                if rtype == RESP_DATA:
                    f.write(data)
                    stats['bytes'] += len(data)
                    stats['blocks'] += 1
                    if progress:
                        progress(stats)
                
                elif rtype == RESP_ERR:
                    raise TapeError("Error during read")
        
        return stats
    
    def write_image(self, filename: str, no_rewind: bool = False,
                    progress=None) -> dict:
        """Write TAP image file to tape"""
        stats = {'blocks': 0, 'bytes': 0}
        
        # Check file exists
        if not os.path.exists(filename):
            raise TapeError(f"File not found: {filename}")
        
        file_size = os.path.getsize(filename)
        
        # Send write_image command
        params = bytes([1 if no_rewind else 0])
        self._send_packet(CMD_WRITE_IMAGE, params)
        
        # Wait for OK response (ready for data)
        rtype, data = self._recv_packet()
        if rtype != RESP_OK:
            raise TapeError("Write image failed to start")
        
        # Open and send file
        with open(filename, 'rb') as f:
            while True:
                # Wait for data request
                rtype, req = self._recv_packet(timeout=30.0)
                
                if rtype is None:
                    raise TapeError("Timeout waiting for data request")
                
                if rtype == RESP_DONE:
                    break
                
                if rtype == RESP_ERR:
                    raise TapeError("Error during write")
                
                if rtype == RESP_NEED_DATA:
                    # MCU is requesting data
                    req_len = req[0] | (req[1] << 8) | (req[2] << 16) | (req[3] << 24) if len(req) >= 4 else 4096
                    
                    chunk = f.read(min(req_len, 2048))
                    if chunk:
                        self._send_packet(RESP_DATA, chunk)
                        stats['bytes'] += len(chunk)
                        stats['blocks'] += 1
                        if progress:
                            progress(stats, file_size)
                    else:
                        # End of file
                        self._send_packet(RESP_DONE)
        
        return stats
    
    def abort(self):
        """Send abort command"""
        self._send_packet(CMD_ABORT)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='PERTEC Tape Interface')
    parser.add_argument('-p', '--port', help='Serial port')
    parser.add_argument('-v', '--verbose', action='store_true')
    
    sub = parser.add_subparsers(dest='cmd')
    sub.add_parser('ping', help='Test connection')
    sub.add_parser('status', help='Tape status')
    sub.add_parser('rewind', help='Rewind tape')
    sub.add_parser('unload', help='Unload tape')
    
    p = sub.add_parser('skip', help='Skip blocks')
    p.add_argument('count', type=int, nargs='?', default=1)
    
    p = sub.add_parser('space', help='Space files')
    p.add_argument('count', type=int, nargs='?', default=1)
    
    p = sub.add_parser('density', help='Set density')
    p.add_argument('mode', choices=['1600', '6250'])
    
    p = sub.add_parser('read', help='Read and display one block')
    
    p = sub.add_parser('create', help='Read tape to TAP file')
    p.add_argument('filename', help='Output filename')
    p.add_argument('-n', '--no-rewind', action='store_true')
    p.add_argument('-s', '--stop', type=int, default=2, help='Stop after N tapemarks')
    p.add_argument('-e', '--stop-error', action='store_true')
    p.add_argument('-V', '--stop-eov', action='store_true', help='Stop on EOV label')
    
    p = sub.add_parser('write', help='Write TAP file to tape')
    p.add_argument('filename', help='Input filename')
    p.add_argument('-n', '--no-rewind', action='store_true')
    
    args = parser.parse_args()
    
    if not args.cmd:
        parser.print_help()
        return
    
    def progress_read(stats):
        print(f"\rBlocks: {stats['blocks']:6d}  Bytes: {stats['bytes']:10d}", end='', flush=True)
    
    def progress_write(stats, total):
        pct = stats['bytes'] * 100 // total if total else 0
        print(f"\rProgress: {pct}%  Bytes: {stats['bytes']:10d}", end='', flush=True)
    
    try:
        with TapeInterface(args.port) as dev:
            if args.cmd == 'ping':
                print("OK" if dev.ping() else "FAIL")
            
            elif args.cmd == 'status':
                st = dev.get_status()
                if st.get('error'):
                    print("Error getting status")
                else:
                    flags = []
                    if st['online']: flags.append("Online")
                    if st['ready']: flags.append("Ready")
                    if st['loadpoint']: flags.append("LoadPoint")
                    if st['eot']: flags.append("EOT")
                    if st['protected']: flags.append("Protected")
                    if st['rewinding']: flags.append("Rewinding")
                    print(f"Status: {' '.join(flags) or 'Offline'}")
                    print(f"Position: {st['position']}")
                    print(f"Raw: 0x{st['raw_status']:04X}")
            
            elif args.cmd == 'rewind':
                dev.rewind()
                print("Rewinding...")
            
            elif args.cmd == 'unload':
                dev.unload()
                print("Unloaded")
            
            elif args.cmd == 'skip':
                dev.skip(args.count)
                print(f"Skipped {args.count} block(s)")
            
            elif args.cmd == 'space':
                dev.space(args.count)
                print(f"Spaced {args.count} file(s)")
            
            elif args.cmd == 'density':
                dev.set_density(1 if args.mode == '1600' else 2)
                print(f"Set density to {args.mode}")
            
            elif args.cmd == 'read':
                status, data = dev.read_block()
                print(f"Status: 0x{status:04X}, Length: {len(data)}")
                if data:
                    # Hex dump first 256 bytes
                    for i in range(0, min(len(data), 256), 16):
                        hex_part = ' '.join(f'{b:02X}' for b in data[i:i+16])
                        asc_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data[i:i+16])
                        print(f'{i:04X}: {hex_part:<48} {asc_part}')
            
            elif args.cmd == 'create':
                stop = ord('V') if args.stop_eov else args.stop
                print(f"Reading tape to {args.filename}...")
                stats = dev.create_image(
                    args.filename,
                    stop_tapemarks=stop,
                    stop_on_error=args.stop_error,
                    no_rewind=args.no_rewind,
                    progress=progress_read if args.verbose else None
                )
                print(f"\nDone: {stats['blocks']} packets, {stats['bytes']} bytes")
            
            elif args.cmd == 'write':
                print(f"Writing {args.filename} to tape...")
                stats = dev.write_image(
                    args.filename,
                    no_rewind=args.no_rewind,
                    progress=progress_write if args.verbose else None
                )
                print(f"\nDone: {stats['blocks']} packets, {stats['bytes']} bytes")
    
    except TapeError as e:
        print(f"Error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nAborted")
        return 1

if __name__ == '__main__':
    sys.exit(main() or 0)