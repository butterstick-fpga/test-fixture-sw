#!/usr/bin/env python3

# This file is Copyright 2021 (c) Greg Davill <greg.davill@gmail.com>
# License: BSD

import subprocess
import sys
import os
import math
import builtins
import statistics

from time import sleep, localtime, strftime

# https://stackoverflow.com/questions/4417546/constantly-print-subprocess-output-while-process-is-running
def execute(command):
    subprocess.check_call(command, stdout=sys.stdout, stderr=sys.stdout)

# https://stackoverflow.com/questions/510357/python-read-a-single-character-from-the-user
class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()


while True:
    # Wait for SPACE, exit on any other input
    print(
"""

-=-=- Hold Button 0 on ButterStick while plugging it in  -=-=- 
-=-=-=-=-= Press SPACE to load over DFU interface =-=-=-=-=-=-
-=-=-=-=-=-=      Any other key to exit         =-=-=-=-=-=-=-
   
""")

    c = getch()

    if c == ' ':
        try:
            execute(["dfu-util", "--alt", "0", "--download", "../prebuilt/example.dfu", "--reset"])
        except:
            ...
    else:
        break