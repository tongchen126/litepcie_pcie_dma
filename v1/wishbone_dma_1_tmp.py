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

# LitePCIeWishboneMaster ---------------------------------------------------------------------------

class LitePCIeWishboneDMATest(Module,AutoCSR):
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
      #  self.wr_table_reset = wr_table_reset = CSRStorage(1,description="Write Table Reset",reset=0)
        self.wr_enable = wr_enable = CSRStorage(1,description="Write Table Enable",reset=0)
        self.data = data = CSRStorage(32,description="Write Data",reset=0x5555_5555)
        self.req_count = CSRStatus(32,description="Write Data",reset=0x5555_5555)

        self.bus_wr = wishbone.Interface()
        self.submodules.wb_dma = wb_dma = WishboneDMAReader(self.bus_wr,leds=leds[6:])

        self.comb += [
            desc_wr.connect(fifo_wr.sink),
            fifo_wr.source.connect(dma_wr.desc_sink),
        ]
        '''
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


class LitePCIeWishboneDMATestRead(Module, AutoCSR):
    def __init__(self, endpoint, data_width=32, leds=None):
        port_rd = endpoint.crossbar.get_master_port()
        self.submodules.dma_rd = dma_rd = LitePCIeDMAReader(
            endpoint=endpoint,
            port=port_rd,
            with_table=False)

        desc_rd = stream.Endpoint(descriptor_layout())
        self.submodules.fifo_rd = fifo_rd = stream.SyncFIFO(descriptor_layout(), 16)

        self.host_addr = host_addr = CSRStorage(32, description="Host ADDR", reset=0)
        self.length = length = CSRStorage(32, description="Length", reset=128)
        self.bus_addr = bus_addr = CSRStorage(32, description="SoC Bus ADDR", reset=0x3000_0000)
        self.rd_enable = rd_enable = CSRStorage(1, description="Read Enable", reset=0)

        self.bus_rd = wishbone.Interface()
        self.submodules.wb_dma = wb_dma = WishboneDMAWriter(self.bus_rd)

        self.comb += [
            desc_rd.connect(fifo_rd.sink),
            fifo_rd.source.connect(dma_rd.desc_sink),
        ]

        self.test1 = CSRStatus(32, reset=0)
        self.test2 = CSRStatus(32, reset=0)

        self.submodules.conv_rd = conv_rd = stream.Converter(nbits_from=endpoint.phy.data_width,nbits_to=data_width)
        dma_enable = Signal(reset=0)
        pending = Signal(reset=0)
        '''
        self.comb += [wb_dma.enable.eq(dma_enable)]
        self.sync += [
            If(rd_enable.storage & rd_enable.re,
               wb_dma.base.eq(bus_addr.storage),
               wb_dma.length.eq(length.storage),
               dma_enable.eq(1),
               self.test1.status.eq(self.test1.status + 1),
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
        '''
        self.comb += [
            dma_rd.start.eq(rd_enable.storage & rd_enable.re),
            dma_rd.source.connect(conv_rd.sink),
            conv_rd.source.connect(wb_dma.sink),
            wb_dma.sink.ready.eq(conv_rd.source.valid),
            desc_rd.address.eq(host_addr.storage),
            desc_rd.length.eq(length.storage),
            desc_rd.valid.eq(rd_enable.storage & rd_enable.re),
        ]


        self.data = data = CSRStatus(128*8, description="DATA", reset=0)
        counter = Signal(32,reset=0)
        data_reading = Signal(reset=0)
        cases = {}
        for i in range(8):
            n = i
            cases[i] = data.status[n*128:(n+1)*128].eq(dma_rd.source.data)
            
        self.comb += If(data_reading & dma_rd.source.valid & dma_rd.source.ready, Case(counter, cases))

        self.sync += [
            If(rd_enable.storage & rd_enable.re,
               data_reading.eq(1),
               counter.eq(0),
            ).Elif(data_reading & dma_rd.source.valid & dma_rd.source.ready,
                counter.eq(counter+1),
                If(counter==7,data_reading.eq(0))
            )
        ]


class LitePCIeWishboneDMATestRead2(Module, AutoCSR):
    def __init__(self, endpoint, data_width=32, leds=None):
        port = endpoint.crossbar.get_master_port()
        self.host_addr = host_addr = CSRStorage(32, description="Host ADDR", reset=0)
        self.length = length = CSRStorage(32, description="Length", reset=128)
        self.bus_addr = bus_addr = CSRStorage(32, description="SoC Bus ADDR", reset=0x3000_0000)
        self.rd_enable = rd_enable = CSRStorage(1, description="Read Enable", reset=0)
        self.bus_rd = bus_rd = wishbone.Interface()
        #self.submodules.wb_dma = wb_dma = WishboneDMAWriter(self.bus_rd, leds=leds[6:])
        #self.submodules.conv_rd = conv_rd = stream.Converter(nbits_from=endpoint.phy.data_width, nbits_to=data_width)
        #dma_enable = Signal(reset=0)
        #self.comb += [wb_dma.enable.eq(dma_enable)]
        #first = Signal()
        #last = Signal()
        #data_fifo = SyncFIFO(dma_layout(endpoint.phy.data_width), 64, buffered=True)
        self.comb += [
            #data_fifo.source.connect(conv_rd.sink),
            #conv_rd.source.connect(wb_dma.sink),
            #conv_rd.source.ready(1),
            #port.sink.connect(data_fifo.sink, keep={"valid", "ready"}),
            #data_fifo.sink.data.eq(port.sink.dat),
            #data_fifo.sink.first.eq(port.sink.first),
            #data_fifo.sink.last.eq(port.sink.last),
        ]
        data = Signal(32,reset=0)
        self.test1 = CSRStatus(32)
        self.test2 = CSRStatus(32)
        self.test3 = CSRStatus(32)
        counter = Signal(32,reset=0)
        wait = Signal(32,reset=0)
        offset = Signal(32,reset=0)
        read_len = Signal(32)
        self.comb += [
            #first.eq(read_len == 0),
            #last.eq((read_len + 1)== length.storage[4:]),
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
            #NextValue(read_len, 0),
            If(rd_enable.storage & rd_enable.re,
                NextState("ISSUE-READ"),
                NextValue(offset,0),
               )
        )
        fsm.act("ISSUE-READ",
            #dma_enable.eq(1),
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
class LitePCIeWishboneDMATestRead3(Module, AutoCSR):
    def __init__(self, endpoint, data_width=32, leds=None):
        port = endpoint.crossbar.get_master_port()
        self.host_addr = host_addr = CSRStorage(32, description="Host ADDR", reset=0)
        self.length = length = CSRStorage(32, description="Length", reset=128)
        self.bus_addr = bus_addr = CSRStorage(32, description="SoC Bus ADDR", reset=0x3000_0000)
        self.rd_enable = rd_enable = CSRStorage(1, description="Read Enable", reset=0)
        self.bus_rd = bus_rd = wishbone.Interface()
        #self.submodules.wb_dma = wb_dma = WishboneDMAWriter(self.bus_rd, leds=leds[6:])
        #self.submodules.conv_rd = conv_rd = stream.Converter(nbits_from=endpoint.phy.data_width, nbits_to=data_width)
        #dma_enable = Signal(reset=0)
        #self.comb += [wb_dma.enable.eq(dma_enable)]
        #first = Signal()
        #last = Signal()
        #data_fifo = SyncFIFO(dma_layout(endpoint.phy.data_width), 64, buffered=True)
        self.comb += [
            #data_fifo.source.connect(conv_rd.sink),
            #conv_rd.source.connect(wb_dma.sink),
            #conv_rd.source.ready(1),
            #port.sink.connect(data_fifo.sink, keep={"valid", "ready"}),
            #data_fifo.sink.data.eq(port.sink.dat),
            #data_fifo.sink.first.eq(port.sink.first),
            #data_fifo.sink.last.eq(port.sink.last),
        ]

        data = Signal(endpoint.phy.data_width,reset=0)
        max_word = endpoint.phy.data_width//32
        self.test1 = CSRStatus(32)
        self.test2 = CSRStatus(32)
        self.test3 = CSRStatus(32)
        counter = Signal(32,reset=0)
        wait = Signal(32,reset=0)
        offset = Signal(32,reset=0)
        read_len = Signal(32)
        self.comb += [
            #first.eq(read_len == 0),
            #last.eq((read_len + 1)== length.storage[4:]),
            port.source.channel.eq(port.channel),
            port.source.first.eq(1),
            port.source.last.eq(1),
            port.source.adr.eq(host_addr.storage+offset<<2),
            port.source.req_id.eq(endpoint.phy.id),
            port.source.tag.eq(0),
            port.source.len.eq(max_word),
            port.source.dat.eq(0),
        ]

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            #NextValue(read_len, 0),
            If(rd_enable.storage & rd_enable.re,
                NextState("ISSUE-READ"),
                NextValue(offset,0),
               )
        )
        fsm.act("ISSUE-READ",
            #dma_enable.eq(1),
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
               NextValue(counter,0),
            ),
        )
        cases = {}
        for i in range(max_word):
            n = max_word - 1 - i
            cases[i] = bus_rd.dat_w.eq(data[n*32:(n+1)*32])
        self.comb += Case(counter, cases)

        fsm.act("WB-WRITE",
            bus_rd.stb.eq(1),
            bus_rd.we.eq(1),
            bus_rd.cyc.eq(1),
            #bus_rd.dat_w.eq(data[counter*32:(counter+1)*32]),
            bus_rd.sel.eq(0xf),
            bus_rd.adr.eq(counter + offset + (bus_addr.storage >> 2)),
            If(bus_rd.ack,
               NextValue(counter,counter+1),
               If(counter + 1==max_word,
                    NextValue(offset,offset+4),
                    NextValue(self.test3.status, self.test3.status + 1),
                    If(offset + 4 == (length.storage >> 2),NextState("IDLE")).Else(NextState("ISSUE-READ"))
                )
            )
        )



        