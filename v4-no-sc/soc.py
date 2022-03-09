    # Add PCIe -------------------------------------------------------------------------------------
    def add_pcie_custom(self, name="pcie", phy=None, ndmas=0, max_pending_requests=8,
        with_dma_buffering = True, dma_buffering_depth=1024,
        with_dma_loopback  = True,
        with_msi           = True):
        # Imports
        from litepcie.core import LitePCIeEndpoint, LitePCIeMSI
        from litepcie.frontend.dma import LitePCIeDMA
        from litepcie.frontend.wishbone import LitePCIeWishboneMaster

        # Checks.
        assert self.csr.data_width == 32

        # Endpoint.
        self.check_if_exists(f"{name}_endpoint")
        endpoint = LitePCIeEndpoint(phy, max_pending_requests=max_pending_requests, endianness=phy.endianness)
        setattr(self.submodules, f"{name}_endpoint", endpoint)

        # MMAP.
        self.check_if_exists(f"{name}_mmap")
        mmap = LitePCIeWishboneMaster(self.pcie_endpoint, base_address=self.mem_map["csr"])
        self.add_wb_master(mmap.wishbone)
        setattr(self.submodules, f"{name}_mmap", mmap)

        from litex.soc.integration.soc import SoCRegion
        from litedram.common import LiteDRAMNativePort
        from litedram.core import LiteDRAMCore
        from litedram.frontend.wishbone import LiteDRAMWishbone2Native
        wb_data_width = 64
        self.submodules.pcie_mem_bus = SoCBusHandler(
            data_width=wb_data_width
        )
        self.pcie_mem_bus.add_region("main_ram",
                                     SoCRegion(origin=self.bus.regions['main_ram'].origin,
                                               size=self.bus.regions['main_ram'].size))
        # Request a LiteDRAM native port.
        port = self.sdram.crossbar.get_port()
        port.data_width = 2 ** int(log2(port.data_width))  # Round to nearest power of 2.

        # Create Wishbone Slave.
        wb_sdram = wishbone.Interface(wb_data_width)
        self.pcie_mem_bus.add_slave("main_ram", wb_sdram)

        litedram_wb = wishbone.Interface(port.data_width)
        self.submodules += wishbone.Converter(wb_sdram, litedram_wb)
        # Wishbone Slave <--> LiteDRAM bridge.
        self.submodules += LiteDRAMWishbone2Native(
            wishbone=litedram_wb,
            port=port,
            base_address=self.pcie_mem_bus.regions["main_ram"].origin)

        from litepcie.frontend.wishbone_dma import LitePCIe2WishboneDMA, LiteWishbone2PCIeDMA, PCIeInterruptTest
        pcie_host_wb2pcie_dma = LiteWishbone2PCIeDMA(endpoint, wb_data_width)
        self.submodules.pcie_host_wb2pcie_dma = pcie_host_wb2pcie_dma
        self.pcie_mem_bus.add_master("pcie_master_wb2pcie", pcie_host_wb2pcie_dma.bus_wr)
        pcie_host_pcie2wb_dma = LitePCIe2WishboneDMA(endpoint, wb_data_width)
        self.submodules.pcie_host_pcie2wb_dma = pcie_host_pcie2wb_dma
        self.pcie_mem_bus.add_master("pcie_master_pcie2wb", pcie_host_pcie2wb_dma.bus_rd)

        self.submodules.pcie_mem_bus_interconnect = wishbone.InterconnectShared(
            masters=list(self.pcie_mem_bus.masters.values()),
            slaves=[(self.pcie_mem_bus.regions[n].decoder(self.pcie_mem_bus), s) for n, s in
                    self.pcie_mem_bus.slaves.items()],
            register=True)

        self.submodules.pcie_irq_test = pcie_irq_test = PCIeInterruptTest()

        from litepcie.core import LitePCIeMSIMultiVector
        if with_msi:
            self.check_if_exists(f"{name}_msi")
            msi = LitePCIeMSIMultiVector()
            setattr(self.submodules, f"{name}_msi", msi)
            self.comb += msi.source.connect(phy.msi)
            self.msis = {}

            self.msis["PCIE2WISHBONE"] = pcie_host_pcie2wb_dma.irq
            self.msis["WISHBONE2PCIE"] = pcie_host_wb2pcie_dma.irq

            self.msis["IRQ1"] = pcie_irq_test.irq1
            self.msis["IRQ2"] = pcie_irq_test.irq2
            self.msis["IRQ3"] = pcie_irq_test.irq3

            for i, (k, v) in enumerate(sorted(self.msis.items())):
                self.comb += msi.irqs[i].eq(v)
                self.add_constant(k + "_INTERRUPT", i)

        # Timing constraints.
        self.platform.add_false_path_constraints(self.crg.cd_sys.clk, phy.cd_pcie.clk)
