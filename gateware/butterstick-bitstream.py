#!/usr/bin/env python3

# This file is Copyright (c) Greg Davill <greg.davill@gmail.com>
# License: BSD


# This variable defines all the external programs that this module
# relies on.  lxbuildenv reads this variable in order to ensure
# the build will finish without exiting due to missing third-party
# programs.
LX_DEPENDENCIES = ["riscv", "nextpnr-ecp5", "yosys"]

# Import lxbuildenv to integrate the deps/ directory
import lxbuildenv



import os
import shutil
import argparse
import subprocess


from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer


from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.soc.interconnect.csr import *

from litex.soc.cores.clock import *
from litex.soc.cores.clock.common import period_ns
from litex.soc.cores.gpio import GPIOOut, GPIOIn
from litex.soc.cores.spi_flash import SpiFlashDualQuad


from litedram.modules import MT41K256M16
from litedram.phy import ECP5DDRPHY

from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII

from rtl.platform import butterstick_r1d0
from rtl.rgb import Leds
from rtl.vccio import VccIo


# CRG ---------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_init    = ClockDomain()
        self.clock_domains.cd_por     = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys     = ClockDomain()
        self.clock_domains.cd_sys2x   = ClockDomain()
        self.clock_domains.cd_sys2x_i = ClockDomain(reset_less=True)

        # # #

        self.stop  = Signal()
        self.reset = Signal()

        # Clk / Rst
        clk30 = platform.request("clk30")
        rst_n = platform.request("user_btn", 0)
        platform.add_period_constraint(clk30, period_ns(30e6))
        platform.add_period_constraint(ClockSignal('jtag'), period_ns(20e6))

        # Power on reset
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(clk30)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # PLL
        self.submodules.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~por_done | ~rst_n | self.rst)
        pll.register_clkin(clk30, 30e6)
        pll.create_clkout(self.cd_sys2x_i, 2*sys_clk_freq)
        pll.create_clkout(self.cd_init,   25e6)
        self.specials += [
            Instance("ECLKSYNCB",
                i_ECLKI = self.cd_sys2x_i.clk,
                i_STOP  = self.stop,
                o_ECLKO = self.cd_sys2x.clk),
            Instance("CLKDIVF",
                p_DIV     = "2.0",
                i_ALIGNWD = 0,
                i_CLKI    = self.cd_sys2x.clk,
                i_RST     = self.reset,
                o_CDIVX   = self.cd_sys.clk),
            AsyncResetSynchronizer(self.cd_sys,    ~pll.locked | self.reset),
            AsyncResetSynchronizer(self.cd_sys2x,  ~pll.locked | self.reset),
        ]



# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    mem_map = {
        "rom":      0x00000000,  # (default shadow @0x80000000)
        "testrom":  0x08000000,  # (default shadow @0x80000000)
        "sram":     0x10000000,  # (default shadow @0xa0000000)
        "spiflash": 0x20000000,  # (default shadow @0xa0000000)
        "main_ram": 0x40000000,  # (default shadow @0xc0000000)
        "csr":      0xf0000000,  # (default shadow @0xe0000000)
        "usb":      0xf0010000,
    }
    mem_map.update(SoCCore.mem_map)

    interrupt_map = {
        "timer0": 0,
        "uart": 1,
    }
    interrupt_map.update(SoCCore.interrupt_map)

    def __init__(self, sys_clk_freq=int(60e6), toolchain="trellis", **kwargs):
        # Board Revision ---------------------------------------------------------------------------
        revision = kwargs.get("revision", "0.2")
        device = kwargs.get("device", "25F")

        platform = butterstick_r1d0.ButterStickPlatform()

        # Serial -----------------------------------------------------------------------------------
        # platform.add_extension(butterstick_r1d0._uart_debug)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, csr_data_width=32, integrated_rom_size=32*1024, integrated_sram_size=16*1024, uart_name='jtag_uart')        
        
        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = crg = _CRG(platform, sys_clk_freq)

        # VCCIO Control ----------------------------------------------------------------------------
        self.submodules.vccio = VccIo(platform.request("vccio_ctrl"))

        # SPI Flash --------------------------------------------------------------------------------
        from litespi.modules import W25Q128JV
        from litespi.opcodes import SpiNorFlashOpCodes as Codes
        self.add_spi_flash(mode="4x", module=W25Q128JV(Codes.READ_1_1_4), with_master=True)


        # Leds -------------------------------------------------------------------------------------
        led = platform.request("led_rgb_multiplex")
        self.submodules.leds = Leds(led.a, led.c)
        self.add_csr("leds")
        
        # Test rom ---------------------------------------------------------------------------------
        self.add_rom("testrom",
                origin   = self.mem_map['testrom'],
                size     = 32*1024,
                contents = [],
                mode     = 'r',
            )
        self.add_constant("ROM_BOOT_ADDRESS", self.mem_map['testrom'])


        self.submodules.ddrphy = ECP5DDRPHY(
            platform.request("ddram"),
            sys_clk_freq=sys_clk_freq)
        self.comb += self.crg.stop.eq(self.ddrphy.init.stop)
        self.comb += self.crg.reset.eq(self.ddrphy.init.reset)
        self.add_sdram("sdram",
            phy           = self.ddrphy,
            module        = MT41K256M16(sys_clk_freq, "1:2"),
            l2_cache_size = kwargs.get("l2_size", 8192)
        )

        # Ethernet / Etherbone ---------------------------------------------------------------------
        self.submodules.ethphy = LiteEthPHYRGMII(
           clock_pads = self.platform.request("eth_clocks"),
           pads       = self.platform.request("eth"))
        self.add_ethernet(phy=self.ethphy)
        

        # Self Reset -------------------------------------------------------------------------------
        rst = Signal()
        self.submodules.reset = GPIOOut(rst)
        self.comb += platform.request("rst_n").eq(~rst)

        # Buttons ----------------------------------------------------------------------------------
        self.submodules.button = GPIOIn(platform.request("user_btn"))

        #Add GIT repo to the firmware
        git_rev_cmd = subprocess.Popen("git describe --tags --first-parent --always".split(),
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE)
        (git_stdout, _) = git_rev_cmd.communicate()
        self.add_constant('CONFIG_REPO_GIT_DESC',git_stdout.decode('ascii').strip('\n'))

    def PackageTestRom(self, builder):
        self.finalize()

        os.makedirs(builder.output_dir, exist_ok=True)

        # Remove un-needed sw packages
        builder.add_software_package("testrom", "{}/../firmware/testrom".format(os.getcwd()))

        builder._prepare_rom_software()
        builder._generate_includes()
        builder._generate_rom_software(compile_bios=False)

        # patch random file into BRAM
        rom_file = os.path.join(builder.software_dir, "testrom", "demo.bin")
        rom_data = soc_core.get_mem_data(rom_file, self.cpu.endianness)

        # Initialize SoC with with demo data.
        self.testrom.mem.init = rom_data

def CreateFirmwareInit(init, output_file):
    content = ""
    for d in init:
        content += "{:08x}\n".format(d)
    with open(output_file, "w") as o:
        o.write(content)




# Build --------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Build ButterStick test gateware")
    parser.add_argument("--update-firmware",
                        default=False,
                        action='store_true',
                        help="compile firmware and update existing gateware")
    args = parser.parse_args()

    soc = BaseSoC()
    builder = Builder(soc)

    rand_rom = os.path.join(builder.gateware_dir, "rand.data")

    input_config = os.path.join(builder.gateware_dir, f"{soc.platform.name}.config")
    output_config = os.path.join(builder.gateware_dir, f"{soc.platform.name}_patched.config")
    
    # Create rand fill for BRAM
    if (os.path.exists(rand_rom) == False) or (args.update_firmware == False):
        os.makedirs(os.path.join(builder.output_dir, 'software'), exist_ok=True)
        os.makedirs(os.path.join(builder.output_dir, 'gateware'), exist_ok=True)
        os.system(f"ecpbram  --generate {rand_rom} --seed {0} --width {32} --depth {32*1024 // 4}")

        # patch random file into BRAM
        data = []
        with open(rand_rom, 'r') as inp:
            for d in inp.readlines():
                data += [int(d, 16)]
        soc.testrom.mem.init = data

        # Build gateware
        vns = builder.build()
        soc.do_exit(vns)
    
    soc.finalize()
    soc.PackageTestRom(builder)


    testrom_file = "{}/testrom/demo.bin".format(builder.software_dir)
    testrom_init = "{}/testrom/testrom.init".format(builder.software_dir)
    CreateFirmwareInit(get_mem_data(testrom_file, soc.cpu.endianness), testrom_init)

    # Insert Firmware into Gateware
    os.system(f"ecpbram  --input {input_config} --output {output_config} --from {rand_rom} --to {testrom_init}")


    # create compressed config (ECP5 specific)
    output_bitstream = os.path.join(builder.gateware_dir, f"{soc.platform.name}.bit")
    os.system(f"ecppack --freq 38.8 --compress --input {output_config} --bit {output_bitstream}")

if __name__ == "__main__":
    main()
