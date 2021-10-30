#!/usr/bin/env python3

# This file is Copyright 2021 (c) Greg Davill <greg.davill@gmail.com>
# License: BSD

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
    #print(current_time)


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
        s = f'TEST: {message:20s}{result}'
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
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    (cmd_stdout, cmd_stderr) = cmd.communicate()

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
    (cmd_stdout, cmd_stderr) = cmd.communicate()

load_bitstream()



# Bitstream running, connect to it via JTAG

def jtag_uart():
    j = jtag.JtagEngine()
    j.configure('ftdi:///1')
    j.reset()
    j.write_ir(BitSequence(0x32, msb=False, length=8))


    while 1:
        # shift out 2000 bits (this seems to work well with the buffers)
        seq = BitSequence()
        for n in range(200):
            seq.append(BitSequence(0x001, msb=False, length=10))

        j.change_state('shift_dr')
        seq = j.shift_register(seq)
        j.change_state('pause_dr')
        
        for n in range(200):
            v = seq[10*n:10*(n+1)]
            v.reverse()
            if v[0]:
                v.lsr(1)
                b = v.tobytes(msb=True)
                try:
                    print(b.decode('ascii')[0], end='', flush=True)
                    serial_log += [b]
                except:
                    ...
        
        print('', end='', flush=True)    
    j.close()



j = jtag.JtagEngine()
j.configure('ftdi:///1')
j.reset()

f = j.controller.ftdi

spi_clk = 0x02
spi_cipo = 0x04
spi_copi = 0x08
spi_cs = 0x10

def write_gpio(ftdi, data: int) -> bytes:
    low_data = 0 & 0xFF
    low_dir = 0x0b
    high_data = (data) & 0xFF
    high_dir = (0x1A) & 0xFF
    return bytes([ftdi.SET_BITS_HIGH, high_data, high_dir])
                

def read_gpio(ftdi) -> bytes:
    return bytes([ftdi.GET_BITS_HIGH])



# Get GPIO port to manage extra pins, use A*BUS4 as GPO, A*BUS4 as GPI
#gpio = spi.get_gpio()
#gpio.set_direction(0x0200, 0x0200)

# SPI transfer
def xfer_byte(b) -> bytes:
    r = bytes()
    b = BitSequence(b, msb=True, length=8)
    for bit in b:
        if bit:
            r += write_gpio(f, spi_copi)
            r += write_gpio(f, spi_clk | spi_copi)
        else:
            r += write_gpio(f, 0)
            r += write_gpio(f, spi_clk)
        r += read_gpio(f)
    return r

    



def read_adc(channel) -> float:
    cmd = bytes()
    cmd += write_gpio(f, 0)
    cmd += xfer_byte(0x01)
    cmd += xfer_byte(0x80 | (channel << 4))
    cmd += xfer_byte(0x0)
    cmd += write_gpio(f, 0)
    cmd += write_gpio(f, spi_cs)

    cmd += bytes([f.SEND_IMMEDIATE])

    f.write_data(cmd)

    data = f.read_data_bytes(24, 4)
    value = 0
    for b in data:
        value = (value << 1) | (b & spi_cipo)
    value >>= 2
    value &= 0x00003FF
    f.purge_buffers()
    return value * (3.33 / 1024.0)

for i in range(8):
    print(f'{0} -> {read_adc(i):0.2f}')
    ...


finish("PASS")