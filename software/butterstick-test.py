#!/usr/bin/env python3

# This file is Copyright 2021 (c) Greg Davill <greg.davill@gmail.com>
# License: BSD

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


j = jtag.JtagEngine()
j.configure('ftdi:///1')
j.reset()
j.write_ir(BitSequence(0x32, msb=False, length=8))


while 1:
    j.change_state('shift_dr')
    v = j.shift_register(BitSequence(0x201 | (ord('a') << 1), msb=False, length=10))
    j.change_state('pause_dr')
    v.reverse()
    
    if v[9]:
        v.lsr(1)
        b = v.tobytes(msb=True)
        try:
            print(b.decode('ascii')[0], end='')
        except:
            ...
    
j.close()



finish("PASS")