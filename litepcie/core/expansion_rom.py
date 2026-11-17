#
# This file is part of LitePCIe.
#
# Copyright (c) 2026 Denis Bodor
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litepcie.common import *

# LitePCIeExpansionROM ------------------------------------------------------------------------------

class LitePCIeExpansionROM(LiteXModule):
    """
    Read-only responder for the PCIe Expansion ROM BAR.

    This is deliberately kept outside of the Crossbar/DMA/Wishbone machinery
    used for BAR0: the Expansion ROM is a small, fixed-content, read-only
    aperture that is only ever accessed a handful of times (by the host's
    BIOS/UEFI firmware during PCI enumeration/boot), so a lightweight
    self-contained responder is simpler than adding a permanent Crossbar
    port for it.

    This module is meant to be forked out of LitePCIeEndpoint's main Request
    stream *before* the Crossbar, based on the `bar_hit` sideband signal --
    see LitePCIeEndpoint.add_expansion_rom() in core/endpoint.py, which wires
    it up.

    Only Memory Read Requests are answered with a Completion. Memory Write
    Requests are silently discarded: the ROM is read-only, and PCIe Memory
    Writes are posted (no Completion is ever expected for them, on real
    BAR0 targets either), so simply dropping them is correct, not a
    shortcut.
    """
    def __init__(self, data_width, size, init=None):
        self.sink   = sink   = stream.Endpoint(request_layout(data_width))
        self.source = source = stream.Endpoint(completion_layout(data_width))

        # Completer ID (Bus/Device/Function): driven from the outside with
        # the PHY's `id` signal -- same convention as other Completion
        # sources in LitePCIe (see frontend/wishbone.py's
        # LitePCIeWishboneMaster, `port.source.cmp_id.eq(endpoint.phy.id)`).
        self.id = Signal(16)

        # # #

        # Memory word = 1 Dword (32-bit), matching PCIe's native access
        # granularity (Requests are always expressed as a Dword count).
        depth = size//4
        if init is None:
            init = [0]*depth
        assert len(init) <= depth, "ROM content does not fit in the requested Expansion ROM size."

        mem  = Memory(32, depth, init=init)
        port = mem.get_port()
        self.specials += mem, port

        # Byte-accurate Completion accounting -------------------------------------------------------
        #
        # Adapted from frontend/wishbone.py's LitePCIeWishboneMaster, which
        # implements the same PCIe Read-Completion byte-counting rules for
        # BAR0 accesses; kept as close as possible to that code on purpose,
        # to make the two easy to compare/review side by side.

        request_adr      = Signal(32)
        request_len      = Signal(10)
        request_first_be = Signal(4)
        request_last_be  = Signal(4)
        request_req_id   = Signal(16)
        request_tc       = Signal(3)
        request_attr     = Signal(2)
        request_tag      = Signal(8)
        dword_index      = Signal(10)
        byte_count       = Signal(12)
        read_dat         = Signal(32)

        first_byte_count   = Signal(3)
        last_byte_count    = Signal(3)
        request_byte_count = Signal(12)
        self.comb += [
            first_byte_count.eq(sum(sink.first_be[n] for n in range(4))),
            last_byte_count.eq(sum(sink.last_be[n] for n in range(4))),
            If(sink.len == 1,
                # PCIe encodes a zero-length Memory Read as a one-Dword request with all First
                # Byte Enables cleared. Its successful Completion still carries one Dword and a
                # Byte Count of one.
                request_byte_count.eq(Mux(first_byte_count == 0, 1, first_byte_count)),
            ).Else(
                request_byte_count.eq(first_byte_count + last_byte_count + 4*(sink.len - 2)),
            ),
        ]

        current_be = Signal(4)
        self.comb += If(request_len == 1,
            current_be.eq(request_first_be),
        ).Elif(dword_index == 0,
            current_be.eq(request_first_be),
        ).Elif(dword_index == (request_len - 1),
            current_be.eq(request_last_be),
        ).Else(
            current_be.eq(0xf),
        )

        current_byte_count   = Signal(3)
        current_lower_offset = Signal(2)
        self.comb += [
            current_byte_count.eq(sum(current_be[n] for n in range(4))),
            If(current_be == 0,
                current_lower_offset.eq(0),
            ).Elif(current_be[0],
                current_lower_offset.eq(0),
            ).Elif(current_be[1],
                current_lower_offset.eq(1),
            ).Elif(current_be[2],
                current_lower_offset.eq(2),
            ).Else(
                current_lower_offset.eq(3),
            ),
        ]

        # Responder FSM --------------------------------------------------------------------------------

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(sink.valid & sink.first,
                If(sink.we,
                    NextState("DROP-WRITE")
                ).Else(
                    NextValue(request_adr,      sink.adr),
                    NextValue(request_len,      sink.len),
                    NextValue(request_first_be, sink.first_be),
                    NextValue(request_last_be,  sink.last_be),
                    NextValue(request_req_id,   sink.req_id),
                    NextValue(request_tc,       sink.tc),
                    NextValue(request_attr,     sink.attr),
                    NextValue(request_tag,      sink.tag),
                    NextValue(dword_index,      0),
                    NextValue(byte_count,       request_byte_count),
                    NextState("SETUP-READ")
                )
            ).Else(
                sink.ready.eq(1)
            )
        )
        fsm.act("DROP-WRITE",
            # Expansion ROM is read-only: discard Write data/beats. PCIe
            # Memory Writes are posted, so no Completion is sent back.
            sink.ready.eq(1),
            If(sink.valid & sink.last,
                NextState("IDLE")
            )
        )
        fsm.act("SETUP-READ",
            # Present this Dword's address to the (synchronous, 1-cycle
            # read latency) BRAM read port.
            port.adr.eq(request_adr[2:] + dword_index),
            NextState("READ-DATA")
        )
        fsm.act("READ-DATA",
            # port.dat_r is now valid: this is exactly one cycle after
            # SETUP-READ presented the address, matching the Memory's
            # synchronous (registered) read port latency.
            NextValue(read_dat, port.dat_r),
            NextState("ISSUE-READ-COMPLETION")
        )
        self.comb += [
            source.first.eq(1),
            source.last.eq(1),
            source.len.eq(1),
            source.byte_count.eq(byte_count),
            source.err.eq(0),
            source.tag.eq(request_tag),
            source.adr.eq(request_adr + 4*dword_index + current_lower_offset),
            source.cmp_id.eq(self.id),
            source.req_id.eq(request_req_id),
            source.tc.eq(request_tc),
            source.attr.eq(request_attr),
            source.dat.eq(read_dat),
        ]
        fsm.act("ISSUE-READ-COMPLETION",
            source.valid.eq(1),
            If(source.ready,
                If(dword_index == (request_len - 1),
                    sink.ready.eq(1),
                    NextState("IDLE"),
                ).Else(
                    NextValue(dword_index, dword_index + 1),
                    NextValue(byte_count, byte_count - current_byte_count),
                    NextState("SETUP-READ"),
                )
            )
        )
