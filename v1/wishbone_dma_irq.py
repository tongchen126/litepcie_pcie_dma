#
# This file is part of LitePCIe.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.soc.interconnect import wishbone

from litepcie.common import *

from litex.soc.cores.dma import *

from litepcie.frontend.dma import *


class WishboneDMAReaderMod(Module, AutoCSR):
    """Read data from Wishbone MMAP memory.

    For every address written to the sink, one word will be produced on the source.

    Parameters
    ----------
    bus : bus
        Wishbone bus of the SoC to read from.

    Attributes
    ----------
    sink : Record("address")
        Sink for MMAP addresses to be read.

    source : Record("data")
        Source for MMAP word results from reading.
    """
    def __init__(self, bus, endianness="little", with_csr=False,leds=None):
        assert isinstance(bus, wishbone.Interface)
        self.bus    = bus
        self.sink   = sink   = stream.Endpoint([("address", bus.adr_width, ("last", 1))])
        self.source = source = stream.Endpoint([("data",    bus.data_width)])

        # # #

        data = Signal(bus.data_width)

        self.submodules.fsm = fsm = FSM(reset_state="BUS-READ")
        fsm.act("BUS-READ",
            bus.stb.eq(sink.valid),
            bus.cyc.eq(sink.valid),
            bus.we.eq(0),
            bus.sel.eq(2**(bus.data_width//8)-1),
            bus.adr.eq(sink.address),
            If(bus.stb & bus.ack,
                NextValue(data, bus.dat_r),
                NextState("SOURCE-WRITE")
            )
        )
        fsm.act("SOURCE-WRITE",
            source.valid.eq(1),
            source.last.eq(sink.last),
            source.data.eq(data),
            If(source.ready,
                sink.ready.eq(1),
                NextState("BUS-READ")
            )
        )


        self.base   = Signal(32,reset=0)
        self.length = Signal(32,reset=0)
        self.enable = Signal(reset=0)
        self.done   = Signal(reset=0)

        # # #

        shift   = log2_int(self.bus.data_width//8)
        base    = Signal(self.bus.adr_width)
        offset  = Signal(self.bus.adr_width)
        length  = Signal(self.bus.adr_width)
        self.comb += base.eq(self.base[shift:])
        self.comb += length.eq(self.length[shift:])

        ctrl_fsm = FSM(reset_state="IDLE")
        ctrl_fsm = ResetInserter()(ctrl_fsm)
        self.submodules.ctrl_fsm = ctrl_fsm
        self.test1 = CSRStatus(32,reset=0)
        ctrl_fsm.act("IDLE",
            self.done.eq(0),
            self.sink.valid.eq(0),
            If(self.enable,
                NextValue(offset, 0),
                NextState("RUN"),
               NextValue(self.test1.status, self.test1.status + 1),
            ),
        )
        ctrl_fsm.act("RUN",
            self.sink.valid.eq(1),
            self.sink.last.eq(offset == (length - 1)),
            self.sink.address.eq(base + offset),
            If(self.sink.ready,
                NextValue(offset, offset + 1),
                If(self.sink.last,
                    NextState("DONE")
                )
            )
        )
        ctrl_fsm.act("DONE",
                     self.done.eq(1),
                     NextState("IDLE"),
                     self.sink.valid.eq(0)
        )

# LitePCIeWishboneMaster ---------------------------------------------------------------------------

class LiteWishbone2PCIeDMA(Module,AutoCSR):
    def __init__(self, endpoint,data_width = 32,leds=None):

        port_wr = endpoint.crossbar.get_master_port(write_only=True)
        self.submodules.dma_wr = dma_wr = LitePCIeDMAWriter(
            endpoint=endpoint,
            port=port_wr,
            with_table=False)

        desc_wr = stream.Endpoint(descriptor_layout())
        self.submodules.fifo_wr = fifo_wr = stream.SyncFIFO(descriptor_layout(), 16)

        self.host_addr = host_addr = CSRStorage(32,description="Host ADDR",reset=0)
        self.length = length = CSRStorage(32,description="Length",reset=128)
        self.bus_addr = bus_addr = CSRStorage(32,description="SoC Bus ADDR",reset=0x3000_0000)
        self.wr_enable = wr_enable = CSRStorage(1,description="Write Table Enable",reset=0)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable Wishbone2PCIe IRQ", reset=0)
        self.irq = Signal(reset=0)

        self.bus_wr = wishbone.Interface()
        self.submodules.wb_dma = wb_dma = WishboneDMAReaderMod(self.bus_wr)

        self.comb += [
            desc_wr.connect(fifo_wr.sink),
            fifo_wr.source.connect(dma_wr.desc_sink),
        ]
        
        self.test1 = CSRStatus(32,reset=0)
        self.test2 = CSRStatus(32,reset=0)

        self.submodules.conv_wr = conv_wr = stream.Converter(nbits_from=data_width, nbits_to=endpoint.phy.data_width)
        dma_enable = Signal(reset=0)
        pending = Signal(reset=0)

        self.comb += [wb_dma.enable.eq(dma_enable)]
        self.sync += [
            If(wr_enable.storage & wr_enable.re,
               wb_dma.base.eq(bus_addr.storage),
               wb_dma.length.eq(length.storage),
               dma_enable.eq(1),
               self.test1.status.eq(self.test1.status+1),
               pending.eq(1),
            ).Elif(pending & wb_dma.done,
                   dma_enable.eq(0),
                   self.test2.status.eq(self.test2.status + 1),
                   pending.eq(0),
            ).
            Elif(~pending,
                   dma_enable.eq(0)
            )
        ]
        self.comb += If(pending & wb_dma.done, self.irq.eq(~irq_disable.storage))
        self.comb += [
            wb_dma.source.connect(conv_wr.sink),
            conv_wr.source.connect(dma_wr.sink),
            desc_wr.address.eq(host_addr.storage),
            desc_wr.length.eq(length.storage),
            desc_wr.valid.eq(wr_enable.storage & wr_enable.re),
        ]
        

class LitePCIe2WishboneDMASingleWord(Module, AutoCSR):
    def __init__(self, endpoint, data_width=32, leds=None):
        port = endpoint.crossbar.get_master_port()
        self.host_addr = host_addr = CSRStorage(32, description="Host ADDR", reset=0)
        self.length = length = CSRStorage(32, description="Length", reset=128)
        self.bus_addr = bus_addr = CSRStorage(32, description="SoC Bus ADDR", reset=0x3000_0000)
        self.rd_enable = rd_enable = CSRStorage(1, description="Read Enable", reset=0)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable PCIe2Wishbone IRQ", reset=0)
        self.bus_rd = bus_rd = wishbone.Interface()
        self.irq = Signal(reset=0)
        data = Signal(32,reset=0)
        self.test1 = CSRStatus(32)
        self.test2 = CSRStatus(32)
        self.test3 = CSRStatus(32)
        counter = Signal(32,reset=0)
        wait = Signal(32,reset=0)
        offset = Signal(32,reset=0)
        read_len = Signal(32)
        self.comb += [
            port.source.channel.eq(port.channel),
            port.source.first.eq(1),
            port.source.last.eq(1),
            port.source.adr.eq(host_addr.storage+(offset<<2)),
            port.source.req_id.eq(endpoint.phy.id),
            port.source.tag.eq(0),
            port.source.len.eq(1),
            port.source.dat.eq(0),
        ]

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(rd_enable.storage & rd_enable.re,
                NextState("ISSUE-READ"),
                NextValue(offset,0),
               )
        )
        fsm.act("ISSUE-READ",
            port.source.valid.eq(1),
            port.source.we.eq(0),
            If(port.source.ready,
                NextState("MAPDATA"),
               NextValue(self.test1.status,self.test1.status+1),
            )
        )
        fsm.act("MAPDATA",
            If(port.sink.valid,
               port.sink.ready.eq(1),
               NextValue(data, port.sink.dat[0:32]),
               NextValue(self.test2.status, self.test2.status + 1),
               NextState("WB-WRITE")
            ),
        )
        fsm.act("WB-WRITE",
            bus_rd.stb.eq(1),
            bus_rd.we.eq(1),
            bus_rd.cyc.eq(1),
            bus_rd.dat_w.eq(data),
            bus_rd.sel.eq(0xf),
            bus_rd.adr.eq(offset + (bus_addr.storage >> 2)),
            If(bus_rd.ack,
               NextValue(offset,offset+1),
               NextValue(self.test3.status, self.test3.status + 1),
               If(offset + 1 == (length.storage >> 2),NextState("IDLE"),self.irq.eq(~irq_disable.storage)).Else(NextState("ISSUE-READ"))
            )
        )

class LitePCIe2WishboneMultiWord(Module, AutoCSR):
    def __init__(self, endpoint, data_width=32, leds=None):
        port = endpoint.crossbar.get_master_port()
        self.host_addr = host_addr = CSRStorage(32, description="Host ADDR", reset=0)
        self.length = length = CSRStorage(32, description="Length", reset=128)
        self.bus_addr = bus_addr = CSRStorage(32, description="SoC Bus ADDR", reset=0x3000_0000)
        self.rd_enable = rd_enable = CSRStorage(1, description="Read Enable", reset=0)
        self.irq_disable = irq_disable = CSRStorage(1, description="Disable PCIe2Wishbone IRQ", reset=0)
        self.bus_rd = bus_rd = wishbone.Interface()
        self.irq = Signal(reset=0)
        self.test1 = CSRStatus(32)
        self.test2 = CSRStatus(32)
        self.test3 = CSRStatus(32)

        offset = Signal(32,reset=0)
        data = Signal(endpoint.phy.data_width, reset=0)
        counter = Signal(32,reset=0)
        max_word = endpoint.phy.data_width//data_width
        self.comb += [
            port.source.channel.eq(port.channel),
            port.source.first.eq(1),
            port.source.last.eq(1),
            port.source.adr.eq(host_addr.storage+(offset<<2)),
            port.source.req_id.eq(endpoint.phy.id),
            port.source.tag.eq(0),
            port.source.len.eq(max_word),
            port.source.dat.eq(0),
        ]

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(rd_enable.storage & rd_enable.re,
                NextState("ISSUE-READ"),
                NextValue(offset,0),
               )
        )
        fsm.act("ISSUE-READ",
            port.source.valid.eq(1),
            port.source.we.eq(0),
            If(port.source.ready,
                NextState("MAPDATA"),
               NextValue(self.test1.status,self.test1.status+1),
            )
        )
        fsm.act("MAPDATA",
            If(port.sink.valid,
               port.sink.ready.eq(1),
               NextValue(data, port.sink.dat),
               NextValue(self.test2.status, self.test2.status + 1),
               NextState("WB-WRITE"),
               NextValue(counter, 0),
            ),
        )
        cases = {}
        for i in range(max_word):
            n = i
            cases[i] = bus_rd.dat_w.eq(data[n*data_width:(n+1)*data_width])
        self.comb += Case(counter, cases)

        fsm.act("WB-WRITE",
            bus_rd.stb.eq(1),
            bus_rd.we.eq(1),
            bus_rd.cyc.eq(1),
            bus_rd.sel.eq(0xf),
            bus_rd.adr.eq(counter + offset + (bus_addr.storage >> 2)),
            If(bus_rd.ack,
               NextValue(counter,counter+1),
               If(counter + 1 == max_word,
                    NextValue(offset,offset+max_word),
                    NextValue(self.test3.status, self.test3.status + 1),
                    If(offset + max_word == (length.storage >> 2),NextState("IDLE"),self.irq.eq(~irq_disable.storage)).Else(NextState("ISSUE-READ"))
                )
            )
        )

class PCIeInterruptTest(Module,AutoCSR):
    def __init__(self):
        self.irq1_csr = CSRStorage(1,reset=0)
        self.irq2_csr = CSRStorage(1,reset=0)
        self.irq3_csr = CSRStorage(1,reset=0)
        self.irq1 = Signal()
        self.irq2 = Signal()
        self.irq3 = Signal()
        self.comb += [
            self.irq1.eq(self.irq1_csr.storage & self.irq1_csr.re),
            self.irq2.eq(self.irq2_csr.storage & self.irq2_csr.re),
            self.irq3.eq(self.irq3_csr.storage & self.irq3_csr.re)
        ]
