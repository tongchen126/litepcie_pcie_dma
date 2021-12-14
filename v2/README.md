**This work is based on Litex project. It combines WishboneDMA and PCIeDMA function and enables DMA transferring between Host Address Space and the Wishbone DMA bus of the SoC.**

# Source File
wishbone_dma.py

# Kernel Demo: 
dma.c

This mainly allocates the DMA buffer and informs when the interrupt is triggered(i.e., the DMA process has finished).

# How to use(Add DMA controller to SoC's dma bus):
```
from wishbone_dma import LitePCIe2WishboneDMA,LiteWishbone2PCIeDMA
pcie_host_wb2pcie_dma = LiteWishbone2PCIeDMA(endpoint)
self.submodules.pcie_host_wb2pcie_dma = pcie_host_wb2pcie_dma
self.dma_bus.add_master("pcie_master_wb2pcie",pcie_host_wb2pcie_dma.bus_wr)
pcie_host_pcie2wb_dma = LitePCIe2WishboneDMA(endpoint)
self.submodules.pcie_host_pcie2wb_dma = pcie_host_pcie2wb_dma
self.dma_bus.add_master("pcie_master_pcie2wb",pcie_host_pcie2wb_dma.bus_rd)

'''INTERRUPT'''
from litepcie.core import LitePCIeMSIMultiVector
if with_msi:
    self.check_if_exists(f"{name}_msi")
    msi = LitePCIeMSI()
    setattr(self.submodules, f"{name}_msi", msi)
    self.comb += msi.source.connect(phy.msi)
    self.msis = {}
    
    self.msis["PCIE2WISHBONE"] = pcie_host_pcie2wb_dma.irq
    self.msis["WISHBONE2PCIE"] = pcie_host_wb2pcie_dma.irq

    for i, (k, v) in enumerate(sorted(self.msis.items())):
        self.comb += msi.irqs[i].eq(v)
        self.add_constant(k + "_INTERRUPT", i)

```

# How to start the DMA(Litex commandline prompt):
After building the bitstream, open the 'csr.json' file in the build directory.
Write to the corresponding CSR register to specify Host Addr(PC), SoC bus address, and length.
Then write to the enable CSR register to start the DMA process.
**Writing to the CSR register can be done by Litex pre-boot commandline prompt or in a kernel module.**

Write to the CSR:
![CSR](https://github.com/tongchen126/litepcie_pcie_dma/blob/master/v2/pblaze_demo/CSR.png)

After a successful DMA, a MSI IRQ has been triggered to HOST:
![INTERRUPT Triggered by HOST](https://github.com/tongchen126/litepcie_pcie_dma/blob/master/v2/pblaze_demo/INTERRUPT.png)


# Test
Basic Data integrity has been tested under Pblaze3(a repurposed Kintex7-325t based board), running Vexriscv-smp & Rocket cpu at 100MHz.
