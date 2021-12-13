# Add DMA controller to SoC's dma bus:
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

# Kernel Demo: 
dma.c

# Usage Demo:
After finishing building the bitstream, open the 'csr.json' file in the build directory.
Write to the CSR register to specify Host Addr(PC), SoC bus addr and length, then write to the enable register to start the DMA process.
If 'irq_disable' is set to zero, then a interrupt will be triggered to the PC after finishing the process.
