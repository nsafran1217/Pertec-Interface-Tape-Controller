#!/usr/bin/env python3
"""
Host-side tape controller client + remote file server.

Usage: run and connect to the device node (e.g. /dev/ttyACM0).
Requires pyserial: pip install pyserial

This program provides a small REPL. Commands correspond to the MCU commands:
STATUS, REWIND, READ <file> [N], WRITE <file> [N], DUMP [E], INIT, ADDRESS <n>,
SKIP <n>, SPACE <n>, UNLOAD, STOP <n|V>, DEBUG <val>

It also implements the remote filesystem protocol used by the MCU:
FSOPEN|R|filename\n -> reply OK|handle\n or ERR|code\n
FSWRITE|handle|len\n followed by len raw bytes from MCU -> reply OK\n
FSREAD|handle|len\n -> send raw bytes back to MCU
FSCLOSE|handle\n -> reply OK\n
"""

import threading
import serial
import sys
import argparse
import time

handles = {}
next_handle = 1
handles_lock = threading.Lock()

class HostServer:
    def __init__(self, port, baud=115200, timeout=1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.thread = threading.Thread(target=self.reader_thread, daemon=True)
        self.thread.start()

    def reader_thread(self):
        global next_handle
        while True:
            try:
                line = self.ser.readline().decode(errors='ignore')
                if not line:
                    continue
                line = line.rstrip('\r\n')
                if line.startswith('FSOPEN|'):
                    # FSOPEN|R|filename
                    parts = line.split('|', 2)
                    if len(parts) == 3:
                        mode = parts[1]
                        fname = parts[2]
                        try:
                            if mode == 'R':
                                f = open(fname, 'rb')
                            else:
                                f = open(fname, 'wb')
                        except Exception as e:
                            self.ser.write(b'ERR|1\n')
                            continue
                        with handles_lock:
                            h = next_handle
                            next_handle += 1
                            handles[h] = f
                        self.ser.write(f'OK|{h}\n'.encode())
                    else:
                        self.ser.write(b'ERR|2\n')
                elif line.startswith('FSWRITE|'):
                    # FSWRITE|<handle>|<len>\n  then read raw bytes
                    parts = line.split('|')
                    if len(parts) >= 3:
                        h = int(parts[1])
                        length = int(parts[2])
                        with handles_lock:
                            f = handles.get(h)
                        if not f:
                            self.ser.write(b'ERR|3\n')
                            continue
                        # read exact bytes
                        toread = length
                        data = bytearray()
                        while toread > 0:
                            chunk = self.ser.read(toread)
                            if not chunk:
                                time.sleep(0.01)
                                continue
                            data.extend(chunk)
                            toread -= len(chunk)
                        try:
                            f.write(data)
                            f.flush()
                            self.ser.write(b'OK\n')
                        except Exception:
                            self.ser.write(b'ERR|4\n')
                    else:
                        self.ser.write(b'ERR|5\n')
                elif line.startswith('FSREAD|'):
                    # FSREAD|<handle>|<len>\n -> send raw bytes
                    parts = line.split('|')
                    if len(parts) >= 3:
                        h = int(parts[1])
                        length = int(parts[2])
                        with handles_lock:
                            f = handles.get(h)
                        if not f:
                            self.ser.write(b'ERR|6\n')
                            continue
                        try:
                            data = f.read(length)
                            if not data:
                                data = b''
                            self.ser.write(data)
                        except Exception:
                            self.ser.write(b'ERR|7\n')
                    else:
                        self.ser.write(b'ERR|8\n')
                elif line.startswith('FSCLOSE|'):
                    parts = line.split('|')
                    if len(parts) >= 2:
                        h = int(parts[1])
                        with handles_lock:
                            f = handles.pop(h, None)
                        if f:
                            try:
                                f.close()
                                self.ser.write(b'OK\n')
                            except Exception:
                                self.ser.write(b'ERR|9\n')
                        else:
                            self.ser.write(b'ERR|10\n')
                    else:
                        self.ser.write(b'ERR|11\n')
                else:
                    # Regular line from MCU (e.g., output) - print to stdout
                    print('[MCU ]', line)
            except Exception as e:
                print('Reader exception:', e)
                time.sleep(0.1)

    def send_command(self, cmd_line):
        if not cmd_line.endswith('\n'):
            cmd_line += '\n'
        self.ser.write(cmd_line.encode())


def repl(server):
    print('Host tape client. Type HELP for commands.')
    while True:
        try:
            line = input('> ')
        except EOFError:
            break
        if not line:
            continue
        if line.upper() == 'QUIT' or line.upper() == 'EXIT':
            break
        if line.upper() == 'HELP':
            print('Commands: STATUS REWIND READ <file> [N] WRITE <file> [N] DUMP <n|E> INIT ADDRESS <n> SKIP <n> SPACE <n> UNLOAD STOP <n|V> DEBUG <val>')
            continue
        # Send raw command line to MCU
        server.send_command(line)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', '-p', required=True, help='Serial port (e.g. /dev/ttyACM0)')
    parser.add_argument('--baud', '-b', type=int, default=115200)
    args = parser.parse_args()

    server = HostServer(args.port, baud=args.baud)
    try:
        repl(server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
