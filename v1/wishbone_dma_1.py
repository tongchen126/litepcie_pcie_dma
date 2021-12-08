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
# Helpers ------------------------------------------------------------------------------------------

def map_wishbone_dat(address, data, wishbone_dat, qword_aligned=False):
    return [
        If(qword_aligned,
            If(address[2],
                wishbone_dat.eq(data[:32])
            ).Else(
                wishbone_dat.eq(data[32:])
            )
        ).Else(
            wishbone_dat.eq(data[:32])
        )
    ]
    
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
            leds[0].eq(1),
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
            leds[0].eq(0),
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
            leds[1].eq(1),
            If(self.enable,
                NextValue(offset, 0),
                NextState("RUN"),
               NextValue(self.test1.status, self.test1.status + 1),
            ),
        )
        ctrl_fsm.act("RUN",
            leds[1].eq(0),
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
        self.bus_addr = bus_addr = CSRStorage(32,description="SoC Bus ADDR",reset=0)
        self.wr_enable = wr_enable = CSRStorage(1,description="Write Table Enable",reset=0)
        self.data = data = CSRStorage(32,description="Write Data",reset=0x5555_5555)

        self.bus_wr = wishbone.Interface()
        self.submodules.wb_dma = wb_dma = WishboneDMAReaderMod(self.bus_wr,leds=leds[6:])

        self.comb += [
            desc_wr.connect(fifo_wr.sink),
            fifo_wr.source.connect(dma_wr.desc_sink),
        ]
        '''
        self.req_count = CSRStatus(32,description="Request Counter",reset=0)
        self.submodules.conv_wr = conv_wr = stream.Converter(nbits_from=data_width, nbits_to=endpoint.phy.data_width)
        req_count = self.req_count.status#Signal(32,reset=0)
        req_last = Signal()
        req_start = Signal()
        self.comb += [req_last.eq(req_count==(self.length.storage>>2))]
        self.sync += [
            If(wr_enable.storage & wr_enable.re,
               req_start.eq(1),
               req_count.eq(0),
               conv_wr.sink.valid.eq(0),
            ).Elif(req_start & ~req_last & conv_wr.sink.ready,
                conv_wr.sink.valid.eq(1),
                req_count.eq(req_count+1),
            ).Elif(req_start & req_last & conv_wr.sink.ready,
                conv_wr.sink.valid.eq(0),
                req_start.eq(0),
                req_count.eq(0),
            ).Elif(req_start,
                conv_wr.sink.valid.eq(0),
            ).Else(
                #req_count.eq(0),
                conv_wr.sink.valid.eq(0),
                #req_start.eq(0),
            ),
        ]
        self.comb += [
            conv_wr.source.connect(dma_wr.sink),

            conv_wr.sink.first.eq(req_count==endpoint.phy.data_width>>2),
            desc_wr.address.eq(host_addr.storage),
            desc_wr.length.eq(length.storage),
            desc_wr.valid.eq(wr_enable.storage & wr_enable.re),
        ]
        '''
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
        self.comb += [
            wb_dma.source.connect(conv_wr.sink),
            conv_wr.source.connect(dma_wr.sink),
            desc_wr.address.eq(host_addr.storage),
            desc_wr.length.eq(length.storage),
            desc_wr.valid.eq(wr_enable.storage & wr_enable.re),
        ]
        

class LitePCIe2WishboneDMA(Module, AutoCSR):
    def __init__(self, endpoint, data_width=32, leds=None):
        port = endpoint.crossbar.get_master_port()
        self.host_addr = host_addr = CSRStorage(32, description="Host ADDR", reset=0)
        self.length = length = CSRStorage(32, description="Length", reset=128)
        self.bus_addr = bus_addr = CSRStorage(32, description="SoC Bus ADDR", reset=0x3000_0000)
        self.rd_enable = rd_enable = CSRStorage(1, description="Read Enable", reset=0)
        self.bus_rd = bus_rd = wishbone.Interface()
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
            port.source.adr.eq(host_addr.storage+offset<<2),
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
               If(offset + 1 == (length.storage >> 2),NextState("IDLE")).Else(NextState("ISSUE-READ"))
            )
        )

        '''
        self.data = data = CSRStatus(128 * 8, description="DATA", reset=0)
        self.counter = CSRStatus(32, reset=0)
        counter = self.counter.status
        cases = {}
        for i in range(8):
            n = i
            cases[i] = data.status[n * 128:(n + 1) * 128].eq(port.sink.dat)

        self.sync += If(port.sink.valid, Case(counter, cases))

        self.submodules.test_fsm = test_fsm = FSM(reset_state="IDLE")
        test_fsm.act("IDLE",
            NextValue(counter,0),
            If(rd_enable.storage & rd_enable.re,
                NextState("ISSUE-READ"),
            )
        )
        test_fsm.act("ISSUE-READ",
            If(port.sink.valid,
               NextValue(counter,counter+1),
               NextValue(self.test2.status, self.test2.status + 1),
            ),
            If((counter>=7) & port.sink.valid,NextState("IDLE"),port.sink.ready.eq(1),NextValue(self.test3.status, self.test3.status + 1),),
        )
        '''

