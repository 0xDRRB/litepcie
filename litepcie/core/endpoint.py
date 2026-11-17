#
# This file is part of LitePCIe.
#
# Copyright (c) 2015-2023 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr    import *

from litepcie.tlp.depacketizer   import LitePCIeTLPDepacketizer
from litepcie.tlp.packetizer     import LitePCIeTLPPacketizer
from litepcie.core.crossbar      import LitePCIeCrossbar
from litepcie.core.expansion_rom import LitePCIeExpansionROM

# LitePCIe Endpoint --------------------------------------------------------------------------------

class LitePCIeEndpoint(LiteXModule):
    def __init__(self, phy, max_pending_requests=4, address_width=32, endianness="big",
        cmp_bufs_buffered  = True,
        with_ptm           = False,
        with_configuration = False,
        address_mask       = None,
    ):
        self.phy                  = phy
        self.max_pending_requests = max_pending_requests

        # The PHY is a module of the SoC with its own CSRs: do not collect them a second time here.
        self.autocsr_exclude = {"phy"}

        # # #

        # Parameters.
        optional_packetizer_capabilities   = []
        optional_depacketizer_capabilities = []
        if with_ptm:
            optional_packetizer_capabilities   = ["PTM"]
            optional_depacketizer_capabilities = ["PTM", "CONFIGURATION"]
        if with_configuration:
            optional_packetizer_capabilities += ["CONFIGURATION"]
            if "CONFIGURATION" not in optional_depacketizer_capabilities:
                optional_depacketizer_capabilities += ["CONFIGURATION"]
        # Default to PHY BAR0 aperture mask for memory requests.
        if address_mask is None:
            address_mask = getattr(phy, "bar0_mask", 0)

        # TLP Packetizer / Depacketizer ------------------------------------------------------------

        if hasattr(phy, "sink") and hasattr(phy, "source"):
            # Shared Request/Completion channels
            self.depacketizer = depacketizer = LitePCIeTLPDepacketizer(
                data_width   = phy.data_width,
                endianness   = endianness,
                address_mask = address_mask,
                capabilities = ["REQUEST", "COMPLETION"] + optional_depacketizer_capabilities,
            )
            self.packetizer = packetizer = LitePCIeTLPPacketizer(
                data_width    = phy.data_width,
                endianness    = endianness,
                address_width = address_width,
                capabilities  = ["REQUEST", "COMPLETION"] + optional_packetizer_capabilities,
            )
            self.comb += [
                phy.source.connect(depacketizer.sink),
                packetizer.source.connect(phy.sink)
            ]
            req_source = depacketizer.req_source
            cmp_sink   = packetizer.cmp_sink
            req_sink   = packetizer.req_sink
            cmp_source = depacketizer.cmp_source
        else:
            if with_ptm and not getattr(phy, "with_ptm", False):
                raise ValueError("PTM requires a PHY with with_ptm=True")
            # Separate Request/Completion channels
            self.cmp_depacketizer = cmp_depacketizer = LitePCIeTLPDepacketizer(
                data_width   = phy.data_width,
                endianness   = endianness,
                address_mask = address_mask,
                capabilities = ["COMPLETION"] + (["CONFIGURATION"] if with_configuration else []),
            )
            self.req_depacketizer = req_depacketizer = LitePCIeTLPDepacketizer(
                data_width   = phy.data_width,
                endianness   = endianness,
                address_mask = address_mask,
                capabilities = ["REQUEST"],
            )
            self.cmp_packetizer = cmp_packetizer = LitePCIeTLPPacketizer(
                data_width    = phy.data_width,
                endianness    = endianness,
                address_width = address_width,
                capabilities  = ["COMPLETION"],
            )
            self.req_packetizer = req_packetizer = LitePCIeTLPPacketizer(
                data_width    = phy.data_width,
                endianness    = endianness,
                address_width = address_width,
                capabilities  = ["REQUEST"] + optional_packetizer_capabilities,
            )
            self.comb += [
                phy.cmp_source.connect(cmp_depacketizer.sink),
                phy.req_source.connect(req_depacketizer.sink),
                cmp_packetizer.source.connect(phy.cmp_sink),
                req_packetizer.source.connect(phy.req_sink),
            ]
            req_source = req_depacketizer.req_source
            cmp_sink   = cmp_packetizer.cmp_sink
            req_sink   = req_packetizer.req_sink
            cmp_source = cmp_depacketizer.cmp_source

        # Crossbar ---------------------------------------------------------------------------------

        self.crossbar = crossbar = LitePCIeCrossbar(
            data_width           = phy.data_width,
            address_width        = address_width,
            max_pending_requests = max_pending_requests,
            cmp_bufs_buffered    = cmp_bufs_buffered,
            with_configuration   = with_configuration,
        )
        self.tlp_ctrl_rst = crossbar.tlp_ctrl_rst

        # Slave: HOST initiates the transactions ---------------------------------------------------
        #
        # Kept for do_finalize() below (see add_expansion_rom()): the final
        # wiring of this section depends on whether an Expansion ROM
        # responder was attached, which can only be known once the whole
        # target script has run -- do_finalize() is Migen/LiteX's hook for
        # exactly that ("compose now, wire at build time").
        self._req_source = req_source
        self._cmp_sink    = cmp_sink

        # No Expansion ROM by default: behavior/wiring is unchanged for
        # existing users unless add_expansion_rom() is called below.
        self.expansion_rom = None

        # Master: FPGA initiates the transactions --------------------------------------------------

        self.comb += [
            crossbar.phy_master.source.connect(req_sink),
            cmp_source.connect(crossbar.phy_master.sink),
        ]

    def add_expansion_rom(self, size, init=None, bar_hit_bit=6):
        """
        Attach a read-only PCIe Expansion ROM responder to this Endpoint.

        Must be called before the design is built (do_finalize() below does
        the actual wiring), but can be called any time after constructing
        the Endpoint -- e.g. from a target script, after self.add_pcie(...)
        has created it -- following LiteX's usual "compose from outside"
        pattern (see e.g. PHY.add_ltssm_tracer()).

        `size` is the Expansion ROM's size in bytes; it must match (or be
        <=) the size configured on the PHY's Vivado IP
        (CONFIG.Expansion_Rom_Size/Scale on S7PCIEPHY). `init`, if given, is
        the ROM's content as a list of 32-bit words (defaults to all-zero).

        `bar_hit_bit` selects which bit of the PHY's `bar_hit` sideband
        signal identifies the Expansion ROM aperture: on Xilinx 7-Series
        (S7PCIEPHY), bit 6 of rx_bar_hit is the Expansion ROM, following
        BAR0-5 on bits 0-5 (PG054). Requires a PHY that actually drives
        `bar_hit` -- currently only S7PCIEPHY with a 128-bit PCIe interface
        does (see phy/s7pciephy.py).
        """
        assert self.expansion_rom is None, "add_expansion_rom() can only be called once."
        self.expansion_rom = LitePCIeExpansionROM(
            data_width = self.phy.data_width,
            size       = size,
            init       = init,
        )
        self.comb += self.expansion_rom.id.eq(self.phy.id)
        self._expansion_rom_bar_hit_bit = bar_hit_bit

    def do_finalize(self):
        req_source = self._req_source
        cmp_sink   = self._cmp_sink
        crossbar   = self.crossbar

        if self.expansion_rom is None:
            # Default behavior, unchanged: everything goes to the Crossbar.
            self.comb += [
                req_source.connect(crossbar.phy_slave.sink),
                crossbar.phy_slave.source.connect(cmp_sink),
            ]
        else:
            # Fork: Requests hitting the Expansion ROM BAR go to the
            # Expansion ROM responder; everything else keeps going through
            # the Crossbar exactly as before (BAR0/CSR/DMA are unaffected).
            is_expansion_rom_req = Signal()
            self.comb += is_expansion_rom_req.eq(req_source.bar_hit[self._expansion_rom_bar_hit_bit])
            self.comb += [
                If(is_expansion_rom_req,
                    req_source.connect(self.expansion_rom.sink),
                ).Else(
                    req_source.connect(crossbar.phy_slave.sink),
                ),
            ]
            # Completions from both sources are merged back into cmp_sink
            # with a plain priority mux -- deliberately *not* the
            # round-robin Arbiter used elsewhere (e.g. LitePCIeTLPPacketizer,
            # tlp/packetizer.py) to merge REQUEST/COMPLETION/CONFIGURATION TX
            # streams.
            #
            # This is not real arbitration between independent concurrent
            # producers (which is what RoundRobin fairness is for) -- by
            # construction (see the Request-side fork above), a given
            # Request only ever goes down one branch, so at most one of the
            # two Completion sources is ever active for it. A plain mux
            # reflects that directly and is simpler to review than an
            # RoundRobin-based Arbiter would be for this specific shape. On
            # genuine overlap with unrelated Crossbar traffic (e.g. a DMA
            # burst still completing when an Expansion ROM read also
            # completes), the Crossbar side simply sees `ready` deasserted
            # for a cycle and stalls -- no data is ever lost or spliced,
            # only possibly delayed by a beat.
            #
            # (Earlier in development, an Arbiter was used here and swapped
            # for this mux while chasing a hardware bug that turned out to
            # be unrelated -- host-side PCI_COMMAND Memory Space Enable
            # being unset, not this code. The Arbiter was never confirmed
            # to be broken in practice; the mux is kept because it is the
            # better fit for this fork/rejoin shape on its own merits, not
            # because it fixed anything.)
            self.comb += [
                If(self.expansion_rom.source.valid,
                    self.expansion_rom.source.connect(cmp_sink),
                ).Else(
                    crossbar.phy_slave.source.connect(cmp_sink),
                ),
            ]
