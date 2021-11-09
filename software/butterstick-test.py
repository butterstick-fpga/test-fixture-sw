#!/usr/bin/env python3

# This file is Copyright 2021 (c) Greg Davill <greg.davill@gmail.com>
# License: BSD

from itertools import count
from logging import DEBUG
from struct import unpack
import serial.tools.list_ports
import serial
import subprocess
import sys
import os
import math
import builtins
import statistics
import re
from pyftdi import gpio,jtag
from pyftdi.spi import SpiController
from pyftdi.bits import BitSequence


import sys
import signal
import os
import time
import serial
import threading
import multiprocessing
from multiprocessing import shared_memory
import argparse
import json
import socket

import fcntl



OFLAGS = None

def set_nonblocking(file_handle):
    """Make a file_handle non-blocking."""
    global OFLAGS
    OFLAGS = fcntl.fcntl(file_handle, fcntl.F_GETFL)
    nflags = OFLAGS | os.O_NONBLOCK
    fcntl.fcntl(file_handle, fcntl.F_SETFL, nflags)

def set_blocking(file_handle):
    """Make a file_handle blocking."""
    global OFLAGS
    OFLAGS = fcntl.fcntl(file_handle, fcntl.F_GETFL)
    nflags = OFLAGS | os.O_NONBLOCK
    fcntl.fcntl(file_handle, fcntl.F_SETFL, nflags)



import termios
import pty
class Console:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.default_settings = termios.tcgetattr(self.fd)

    def configure(self):
        settings = termios.tcgetattr(self.fd)
        settings[3] = settings[3] & ~termios.ICANON & ~termios.ECHO
        settings[6][termios.VMIN] = 1
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        set_nonblocking(self.fd)

    def unconfigure(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.default_settings)
        set_blocking(self.fd)

    def getkey(self):
        return os.read(self.fd, 1)

    def escape_char(self, b):
        return False

    def handle_escape(self, b):
        return None

from time import sleep, localtime, strftime




BRIGHTGREEN = '\033[92;1m'
BRIGHTRED = '\033[91;1m'
ENDC = '\033[0m'

serial_log = []
output_log = []

# https://stackoverflow.com/questions/4417546/constantly-print-subprocess-output-while-process-is-running
def execute(command):
    subprocess.check_call(command, stdout=sys.stdout, stderr=sys.stdout)

def log(logtype, message, result=None):
    if logtype == "info":
        ...
        output_log.append(f'INFO: {message}')
        print(f'INFO: {message}')
    elif logtype == "test":
        s = f'TEST: {message:40s}{result}'
        output_log.append(s)

        if result == "OK":
            print(BRIGHTGREEN + s + ENDC)
        if result == "FAIL":
            print(BRIGHTRED + s + ENDC)
            finish("FAIL") # Exit early
    elif logtype == "debug":
        ...
        serial_log.append(message)
        #print(message)


def load_bitstream():
    # Load test-bitstream over JTAG
    print("-- Loading test bitstream into SRAM..")

    # check device type
    cmd = subprocess.Popen(["ecpprog", "-t"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    #(cmd_stdout, cmd_stderr) = cmd.communicate()
    cmd_stdout = b''
    for line in cmd.stdout:
        line = bytes(line, 'ascii')
        cmd_stdout += line
        #print(line.decode(), end='')
    cmd.stdout.close()
    return_code = cmd.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, ["ecpprog", "-t"])

    ecp_device = re.findall("LFE[\w-]+", cmd_stdout.decode('ascii'))

    if len(ecp_device) != 1:
        log("test", f"JTAG ID {ecp_device} != {'LFE5UM5G-85'}", "FAIL")

    if ecp_device[0] != 'LFE5UM5G-85':
        log("test", f"JTAG ID {ecp_device} != {'LFE5UM5G-85'}", "FAIL")

    print(f"JTAG:{ecp_device[0]} Detected")

    test_bitstream = '../gateware/build/butterstick_r1d0/gateware/butterstick_r1d0.bit'

    cmd = subprocess.Popen(["ecpprog", "-S", test_bitstream],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    #(cmd_stdout, cmd_stderr) = cmd.communicate()
    for line in cmd.stdout:
        print(line.decode(), end='')
    cmd.stdout.close()
    return_code = cmd.wait()

load_bitstream()


def test_vccio(voltage, actual):
    for i,read_voltage in enumerate(actual):
        if abs(voltage - read_voltage) < (read_voltage * 0.05):
            log("test", f"voltage-vccio{i}-{voltage} : {read_voltage:0.2f} ({100 * ((read_voltage - voltage) / read_voltage):0.01f}%)", "OK")
        else:
            log("test", f"voltage-vccio{i} {voltage}V != {read_voltage:0.2f}", "FAIL")


check_str = [
    (b'BIOS CRC passed', 'BIOS CRC PASS', 'OK'),
    (b'Memtest OK', 'Memtest OK', 'OK'),
    (b'Memspeed at 0x20000000', 'FLASH Test', 'OK'),
    (b'LiteX minimal demo app', 'RISCV app loaded', 'OK'),
    (b'I/O PORT A passed', 'I/O PORT A', 'OK'),
    (b'I/O PORT A failed', 'I/O PORT A', 'FAIL'),
    (b'I/O PORT B passed', 'I/O PORT B', 'OK'),
    (b'I/O PORT B failed', 'I/O PORT B', 'FAIL'),
    (b'I/O PORT C passed', 'I/O PORT C', 'OK'),
    (b'I/O PORT C failed', 'I/O PORT C', 'FAIL'),
]

class LiteXTerm:
    prompt = b"testrom-cmd>"
    def __init__(self, jtb):
        self.reader_alive = False
        self.writer_alive = False

        self.passed = False

        self.jtb = jtb
        self.detect_buffer = bytes(len(self.prompt))

        self.console = Console()

        signal.signal(signal.SIGINT, self.sigint)
        self.sigint_time_last = 0


    def open(self, port, baudrate):
        if hasattr(self, "port"):
            return
        self.port = serial.serial_for_url(port, baudrate)

    def close(self):
        if not hasattr(self, "port"):
            return
        self.port.close()
        del self.port

    def sigint(self, sig, frame):
        if hasattr(self, "port"):
            self.port.write(b"\x03")
        sigint_time_current = time.time()
        # Exit term if 2 CTRL-C pressed in less than 0.5s.
        if (sigint_time_current - self.sigint_time_last < 0.5):
            self.console.unconfigure()
            jtb.close()
            self.close()
            sys.exit()
        else:
            self.sigint_time_last = sigint_time_current

    def detect_magic(self, data):
        if len(data):
            self.detect_buffer = self.detect_buffer[1:] + data
            return self.detect_buffer[:len(self.prompt)] == self.prompt
        else:
            return False


    def reader(self):
        try:
            test_idx = 0
            log_bytes = bytes([])
            while self.reader_alive:
                c = self.port.read()
                
                # sys.stdout.buffer.write(c)
                # sys.stdout.flush()

                log_bytes += c
                for val in check_str:
                    str,msg,result = val
                    if str in log_bytes:
                        check_str.remove(val)
                        log_bytes = bytes()
                        log("test", msg, result)

                if self.detect_magic(c):
                    test_idx += 1
                    vccio_voltages = [jtb.voltages[3],jtb.voltages[1],jtb.voltages[4]]
                    if test_idx == 1:
                        self.port.write(b'vccio 59700\n')

                    if test_idx == 2:
                        test_vccio(1.20, vccio_voltages)
                        self.port.write(b'vccio 45000\n')

                    if test_idx == 3:
                        test_vccio(1.8, vccio_voltages)
                        self.port.write(b'vccio 26500\n')

                    if test_idx == 4:
                        test_vccio(2.5, vccio_voltages)
                        self.port.write(b'vccio 6500\n')

                    if test_idx == 5:
                        test_vccio(3.3, vccio_voltages)
                        self.port.write(b'eth_phy\n')
                    
                    if test_idx == 6:
                        if b'phy_mdio_read(2)=0022' in log_bytes:
                            log('test', 'PHY_ID0', 'OK')
                        elif b'phy_mdio_read(2)=' in log_bytes:
                            log('test', 'PHY_ID0', 'FAIL')    
                        
                        if b'phy_mdio_read(3)=1622' in log_bytes:
                            log('test', 'PHY_ID1', 'OK')
                        elif b'phy_mdio_read(3)=' in log_bytes:
                            log('test', 'PHY_ID1', 'FAIL')   

                        self.port.write(b'io_test\n')
                    
                    if test_idx == 7:
                        
                        #jtb.close()
                        self.port.write(b'led\n\n')
                    
                    if test_idx == 8:
                        
                        #jtb.close()
                        self.port.write(b'\n\n')

                        self.stop_writer()
                        self.reader_alive = False

                        self.passed = True
            



        except serial.SerialException:
            self.reader_alive = False
            self.console.unconfigure()
            raise

    def start_reader(self):
        self.reader_alive = True
        self.reader_thread = threading.Thread(target=self.reader)
        self.reader_thread.setDaemon(True)
        self.reader_thread.start()

    def stop_reader(self):
        self.reader_alive = False
        self.reader_thread.join()

    def writer(self):
        try:
            while self.writer_alive:
                try:
                    b = self.console.getkey()
                except BlockingIOError:
                    continue

                if b == b"\x03":
                    self.stop()
                elif b == b"\n":
                    self.port.write(b"\x0a")
                elif self.console.escape_char(b):
                    b = self.console.getkey()
                    ansi_seq = self.console.handle_escape(b)
                    self.port.write(ansi_seq)
                else:
                    self.port.write(b)
        
        except:
            self.writer_alive = False
            self.console.unconfigure()
            raise

    def start_writer(self):
        self.writer_alive = True
        self.writer_thread = threading.Thread(target=self.writer)
        self.writer_thread.setDaemon(True)
        self.writer_thread.start()

    def stop_writer(self):
        self.writer_alive = False
        self.writer_thread.join()

    def start(self):
        self.start_reader()
        self.start_writer()

    def stop(self):
        self.reader_alive = False
        self.writer_alive = False

    def join(self, writer_only=False):
        self.writer_thread.join()
        if not writer_only:
            self.reader_thread.join()


class JTAGTestBed:

    spi_clk = 0x02
    spi_cipo = 0x04
    spi_copi = 0x08
    spi_cs = 0x10

    def __init__(self):
        self.j = jtag.JtagEngine()
        self.j.controller.ftdi.log.setLevel(level=DEBUG)
        self.j.configure('ftdi:///1')
        self.j.reset()
        self.j.write_ir(BitSequence(0x32, msb=False, length=8))

        self.jtag_pty_running = True
        self.voltages = [0.0] * 8

    def open(self):
        self.file, self.name = pty.openpty()

        set_nonblocking(self.file)

        self.jtag_pty_thread = threading.Thread(target=self.jtag_pty)
        self.jtag_pty_thread.start()

    def close(self):
        self.jtag_pty_running = False
        self.jtag_pty_thread.join()
        self.j.close()

    def jtag_pty(self):
        while self.jtag_pty_running:
            seq = BitSequence()

            send = False
            r = 0
            try:
                r = os.read(self.file, 1)
                send = True
            except BlockingIOError:
                pass

            for n in range(120):
                val = 0x001
                if send:
                    val |= 0x200 | (ord(r) << 1)
                seq.append(BitSequence(val, msb=False, length=10))
                send = False

            self.j.change_state('pause_dr')
            self.j.change_state('shift_dr')
            seq = self.j.shift_register(seq)

            for n in range(120):
                v = seq[10*n:10*(n+1)]
                v.reverse()
                if v[0]:
                    v.lsr(1)
                    b = v.tobytes(msb=True)
                    #print(b)
                    try:
                        os.write(self.file, bytes(b.decode('ascii')[0], encoding='ascii'))
                    except UnicodeDecodeError:
                        pass

            # Voltage readings
            for i in range(8):
                self.voltages[i] = self.read_adc(i)




    def write_gpio(self, data: int) -> bytes:
        high_data = (data) & 0xFF
        high_dir = (0x1A) & 0xFF
        return bytes([self.j.controller.ftdi.SET_BITS_HIGH, high_data, high_dir])


    def read_gpio(self) -> bytes:
        return bytes([self.j.controller.ftdi.GET_BITS_HIGH])

    # SPI transfer
    def xfer_byte(self,b) -> bytes:
        r = bytes()
        b = BitSequence(b, msb=True, length=8)
        for bit in b:
            if bit:
                r += self.write_gpio(self.spi_copi)
                r += self.write_gpio(self.spi_clk | self.spi_copi)
            else:
                r += self.write_gpio(0)
                r += self.write_gpio(self.spi_clk)
            r += self.read_gpio()
        return r

    def read_adc(self, channel) -> float:
        f = self.j.controller.ftdi


        cmd = bytes()
        cmd += self.write_gpio(0)
        cmd += self.xfer_byte(0x01)
        cmd += self.xfer_byte(0x80 | (channel << 4))
        cmd += self.xfer_byte(0x0)
        cmd += self.write_gpio(0)
        cmd += self.write_gpio(self.spi_cs)

        cmd += bytes([f.SEND_IMMEDIATE])


        f.write_data(cmd)
        data = f.read_data_bytes(24, 4)

        value = 0
        for b in data:
            value = (value << 1) | (b & self.spi_cipo)
        value >>= 2
        value &= 0x00003FF

        return value * (3.33 / 1024.0)


jtb = JTAGTestBed()
term = LiteXTerm(jtb)

sleep(0.2)

jtb.open()

port = os.ttyname(jtb.name)

term.open(port, 1000000)
term.console.configure()
term.start()
#


def finish(result):
    if result == "PASS":
        print(BRIGHTGREEN + """
  ############################
  #          PASS            #
  ############################""" + ENDC)
        return
    if result == "FAIL":
        print(BRIGHTRED + """
  ############################
  #          FAIL            #
  ############################""" + ENDC)

    term.stop()
    
    term.join(True)
    term.console.unconfigure()
    
    term.close()
    jtb.close()


    sys.exit(0)




sleep(0.2)

voltage_rails = [
    (0, "eth_core", 1.2),
#    (1, "vccio1", 3.3),
    (2, "fpga_core", 1.2),
#    (3, "vccio0", 3.3),
#    (4, "vccio2", 1.8),
    (5, "vcc", 3.3),
    (6, "ddr3l", 1.35),
    (7, "fpga_aux", 2.5)
]

for number, name, voltage in voltage_rails:
    read_voltage = jtb.voltages[number]

    if abs(voltage - read_voltage) < (read_voltage * 0.05):
        log("test", f"voltage-{name:10s}: {read_voltage:0.2f} ({100 * ((read_voltage - voltage) / read_voltage):0.01f}%)", "OK")
    else:
        log("test", f"voltage-{name:10s} {voltage} != {read_voltage:0.2f}", "FAIL")



term.join()
jtb.close()
term.console.unconfigure()

try:
    if term.passed:
        # check device type
        execute(["ecpprog", "../prebuilt/butterstick_bootloader.bit"])
        finish('PASS')
        sys.exit(0)

except subprocess.CalledProcessError:
    finish('FAIL')
finish('FAIL')
sys.exit(0)
