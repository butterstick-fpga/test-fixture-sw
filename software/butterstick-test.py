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

    def unconfigure(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.default_settings)

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

def finish(result):
    if result == "PASS":
        print(BRIGHTGREEN + """
  ############################
  #          PASS            #
  ############################""" + ENDC)
    if result == "FAIL":
        print(BRIGHTRED + """
  ############################
  #          FAIL            #
  ############################""" + ENDC)

    # Flush log out to file
    os.makedirs('log', exist_ok=True)

    t = localtime()
    current_time = strftime("%Y-%m-%d-%H:%M:%S", t)

    f= open(f"log/{result}-{current_time}.txt","w+")
    for l in output_log:
        f.write(l + '\n')
    f.write("\r\n-=-=-=-=-= RAW serial log -=-=-=-=-=-=\r\n")
    for l in serial_log:
        f.write(l + '\n')
    f.close()

    sys.exit(0)


def log(logtype, message, result=None):
    if logtype == "info":
        ...
        output_log.append(f'INFO: {message}')
        print(f'INFO: {message}')
    elif logtype == "test":
        s = f'TEST: {message:30s}{result}'
        output_log.append(s)

        if result == "OK":
            print(BRIGHTGREEN + s + ENDC)
        if result == "FAIL":
            print(BRIGHTRED + s + ENDC)
            #finish("FAIL") # Exit early 
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


class LiteXTerm:
    def __init__(self):
        self.reader_alive = False
        self.writer_alive = False

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
            self.close()
            sys.exit()
        else:
            self.sigint_time_last = sigint_time_current

    def reader(self):
        try:
            while self.reader_alive:
                c = self.port.read()
                sys.stdout.buffer.write(c)
                sys.stdout.flush()

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
                b = self.console.getkey()
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


OFLAGS = None

def set_nonblocking(file_handle):
    """Make a file_handle non-blocking."""
    global OFLAGS
    OFLAGS = fcntl.fcntl(file_handle, fcntl.F_GETFL)
    nflags = OFLAGS | os.O_NONBLOCK
    fcntl.fcntl(file_handle, fcntl.F_SETFL, nflags)

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

        self.voltages = [0.0] * 8
    
    def open(self):
        self.file, self.name = pty.openpty()

        set_nonblocking(self.file)

        self.jtag_pty_thread = threading.Thread(target=self.jtag_pty)
        self.jtag_pty_thread.start()
        
       

    def close(self):
        self.jtag_pty_thread.terminate()
        self.j.close()


    def jtag_pty(self):
        i = 0
        while True:
        
            # shift out 2000 bits (this seems to work well with the buffers)
            seq = BitSequence()

            send = False
            r = 0
            try:
                r = os.read(self.file, 1)
                send = True
            except BlockingIOError:
                pass

            for n in range(200):
                val = 0x001 
                if send:
                    val |= 0x200 | (ord(r) << 1)
                seq.append(BitSequence(val, msb=False, length=10))
                send = False

            self.j.change_state('pause_dr')
            self.j.change_state('shift_dr')
            seq = self.j.shift_register(seq)
            #self.j.sync()
            
            
            
            for n in range(200):
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
            self.voltages[i] = self.read_adc(i)
            #print(self.voltages)
            i = (i + 1) % 8

    


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


term = LiteXTerm()

sleep(0.2)

jtb = JTAGTestBed()
jtb.open()
port = os.ttyname(jtb.name)

term.open(port, 1000000)
term.console.configure()
term.start()
#term.join(True)






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

while(True):
    for number, name, voltage in voltage_rails:
        #jtb.read_adc(number)
        #print(jtb.voltages)

        # if abs(voltage - read_voltage) < (read_voltage * 0.05):
        #     log("test", f"{name:10s}: {read_voltage:0.2f} ({100 * ((voltage - read_voltage) / read_voltage):0.01f}%)", "OK")
        # else:
        #     log("test", f"{name:10s} {voltage} != {read_voltage:0.2f}", "FAIL")
        
        sleep(0.05)



finish("PASS")