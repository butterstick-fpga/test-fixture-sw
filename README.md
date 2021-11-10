# ButterStick Test Software

This repository contains the software, firmware, and gateware to run some basic hardware tests on the ButterStick r1d0 board. It also loads up a bootloader using the programming-jig once all tests have passed

In this project:
- LiteX project to facilitate tests on the ButterStick FPGA PCB
- Software stack for performing those tests

---

## Usage ##

The tests for these boards are done in two phases. 
- Phase 1 uses the test-jig, and will load the bootloader onto the board, it takes ~60s to run.
- Phase 2 makes use of the bootloader and ensures that it's functional in communicating to a PC over USB.

### Phase 1
1. Locate a linux computer. These tests have been developed on an Ubuntu based machine.
2. Plug in the 'butterstick-programmer' board to your computer. The programmer makes use of an FTDI232H, which should appear without needing drivers.
3. Install python dependencies `pyftdi`
```
python3 -m pip install pyftdi --user
```
4. Enter run `run-test.py`
```
$ cd software
$ python3 run-test.py
```
5. Load a ButterStick board into the test-jig, no extra cables should be attached to the ButterStick, only the test-jig.
6. Press `space` as instructed by the script, the tests will run. Towards the end of the test the LEDs will run through each colour.
7. If the test result in PASS then unload the butterStick and set it aside for Phase 2. Repeat.

### Phase 2

0. Prep: Copy udev rule
```
sudo cp extra/10-butterstick-dfu.rules /dev/udev/rules.d/
```

1. Run `run-dfu.py`
```
$ cd software
$ python run-dfu.py
```
2. While holding BTN0 on the ButterStick plug it in, the LEDs should indicate that it's running.
3. Press `space` and watch the example program load over USB.
4. Check if this completed correctly, and the ButterStick showing blue moving LEDs
5. Ship!