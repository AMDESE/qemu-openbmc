/*
 * ASPEED AST2700 eSPI Controller
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef ASPEED_ESPI_H
#define ASPEED_ESPI_H

#include "hw/core/sysbus.h"
#include "hw/misc/aspeed_tafsb.h"
#include "hw/misc/aspeed_lpc.h"
#include "hw/char/aspeed_vuart.h"

/*
 * Largest payload a cycle can carry. A length field of zero means this,
 * not zero: the field is twelve bits and has to encode 4096.
 */
#define ASPEED_ESPI_MAX_PLD     4096

/*
 * Widest header a peripheral queue stages ahead of its payload: an 8-byte
 * address for a 64-bit memory cycle. Message cycles use a 5-byte header
 * instead, but the queue buffer has to fit whichever is staged.
 */
#define ASPEED_ESPI_PERIF_HDR_MAX 8

#define TYPE_ASPEED_ESPI "aspeed.espi"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedESPIState, ASPEED_ESPI)

/*
 * The block occupies 4 KiB of address space. Only a scattered subset of the
 * offsets is implemented; the gaps do not read as zero and do not read as
 * all-ones, but return one of four band signatures. See aspeed_espi_gap().
 */
#define ASPEED_ESPI_SIZE        0x1000
#define ASPEED_ESPI_NR_REGS     (ASPEED_ESPI_SIZE >> 2)

/* Number of Mapped Memory Buffer Interface instances the block can expose. */
#define ASPEED_ESPI_MMBI_MAX    8

/* Peripheral channel Rx address-translation windows, ESPI140-16C. */
#define ASPEED_ESPI_PERIF_NR_WINDOWS 2

/*
 * The four eSPI channels: peripheral, virtual wire, out-of-band and flash.
 * The channel register blocks share a layout at this level and sit 0x100
 * apart starting at 0x100.
 */
#define ASPEED_ESPI_NR_CHAN     4

#define ASPEED_ESPI_CHAN_PERIF  0
#define ASPEED_ESPI_CHAN_VW     1
#define ASPEED_ESPI_CHAN_OOB    2
#define ASPEED_ESPI_CHAN_FLASH  3

/*
 * Per-channel description. Only the pieces that genuinely differ between
 * channels live here; everything else is derived from the block base.
 */
typedef struct AspeedESPIChannelInfo {
    const char *name;
    hwaddr      base;       /* first offset of the channel register block */
    uint32_t    sw_rdy;     /* SW_RDY bit in the channel control register */
    uint32_t    ctrl_reset; /* reset value of the channel control register */
    uint32_t    sts_reset;  /* reset value of the channel status register */
    uint32_t    ctrl_ro;    /* control bits the channel drives, not software */
} AspeedESPIChannelInfo;

/*
 * How the target reaches its controller, for state it changed on its own
 * rather than in answer to a request: a virtual wire it drove, a status
 * word the controller has not seen yet (Alert, on hardware), or a
 * peripheral cycle it originated as bus master.
 *
 * None of these fire while a controller request is being serviced. Every
 * response already carries the status word, and calling out mid-request
 * would mean reaching the controller while still owing it that reply.
 * pc_notify is fire-and-forget: unlike a flash Tx trigger it leaves
 * nothing to poll for later, the cycle has already left.
 */
typedef struct AspeedESPINotifier {
    /* One virtual wire group as it goes on the eSPI bus: index and data */
    void (*vw_notify)(void *opaque, uint8_t index, uint8_t data);
    void (*alert_notify)(void *opaque, uint32_t espi_status);
    void (*pc_notify)(void *opaque, uint8_t cyc, uint8_t tag, uint64_t addr,
                      uint32_t len, const uint8_t *data, uint8_t msg_code);
} AspeedESPINotifier;

/* Mutable per-channel state, indexed by channel number. */
typedef struct AspeedESPIChannel {
    bool     sw_ready;      /* software has armed the channel */
    uint32_t status;        /* contribution to the shared eSPI Status word */
} AspeedESPIChannel;

/*
 * A single cycle-carrying queue: one payload buffer plus the header fields
 * that travel with it. @pos is the FIFO cursor -- the next byte to hand to
 * software on an Rx queue, the next free byte on a Tx queue. @pending is
 * false exactly when the queue is free to accept another cycle; unlike the
 * flash channel's counted non-posted queue, each of these is a single slot.
 */
typedef struct AspeedESPIQueue {
    uint8_t  buf[ASPEED_ESPI_MAX_PLD + ASPEED_ESPI_PERIF_HDR_MAX];
    uint32_t len;
    uint32_t pos;
    uint8_t  cyc;
    uint8_t  tag;
    bool     pending;
} AspeedESPIQueue;

/*
 * IRQ lines, in sysbus_init_irq() order: index 0 is the eSPI interrupt,
 * index 1 is raised only by MMBI.
 */
#define ASPEED_ESPI_IRQ_MAIN    0
#define ASPEED_ESPI_IRQ_MMBI    1
#define ASPEED_ESPI_NR_IRQS     2

/*
 * Input GPIO carrying the eSPI reset signal from the host controller, either
 * the dedicated pin or an in-band reset cycle. High means reset asserted.
 */
#define ASPEED_ESPI_RESET_GPIO  "espi-reset"

/*
 * Input GPIO carrying the linked VUART's host-side SIRQ line. Level, not
 * edge: high means the line is asserted.
 */
#define ASPEED_ESPI_VUART_SIRQ_GPIO  "vuart-sirq"

/*
 * Input GPIO carrying the PME output of the linked LPC controller, which
 * drives the PME# virtual wire. High means PME# asserted.
 */
#define ASPEED_ESPI_PME_GPIO  "pme"

struct AspeedESPIState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;
    qemu_irq irq[ASPEED_ESPI_NR_IRQS];

    uint32_t regs[ASPEED_ESPI_NR_REGS];

    AspeedESPIChannel chan[ASPEED_ESPI_NR_CHAN];

    /*
     * Shared eSPI Status word, recomposed from every channel whenever
     * channel state changes, and the copy last reported to the host. The
     * controller signals Alert when the two differ.
     */
    uint32_t status;
    uint32_t status_shadow;

    /*
     * Flash channel mapping window, kept decoded from the target address and
     * mask register pairs so cycle handling does not reassemble them.
     */
    struct {
        uint64_t taddr;
        uint64_t mask;

        /*
         * Staging for the cycle currently handed to software and the one
         * being taken back from it. The FIFO registers walk these a byte at
         * a time; in DMA mode they are copied whole.
         */
        uint8_t  rx_buf[ASPEED_ESPI_MAX_PLD];
        uint32_t rx_len;
        uint32_t rx_pos;
        uint8_t  tx_buf[ASPEED_ESPI_MAX_PLD];
        uint32_t tx_len;
        uint32_t tx_pos;

        /*
         * The cycle type and tag software queued its completion under, and
         * whether that completion is still waiting for the controller to
         * collect. tx_len alone cannot tell "nothing queued" apart from "a
         * genuine 0-length completion" (e.g. FLASH_SUC_CMPLT), hence the
         * separate flag.
         */
        uint8_t  tx_cyc;
        uint8_t  tx_tag;
        bool     tx_pending;

        /* Non-posted requests accepted but not yet completed */
        uint32_t np_pending;
    } flash;

    /*
     * Peripheral channel Rx address-translation windows, decoded like the
     * flash channel's mapping window above but not migrated -- pure
     * derivations of regs[], recomputed post-load instead. Each window is
     * independent and separately enabled.
     */
    struct {
        struct {
            uint64_t src;
            uint64_t tgt;
            uint64_t mask;
            bool     enabled;
        } window[ASPEED_ESPI_PERIF_NR_WINDOWS];

        /*
         * Messages and completions for the BMC's own outbound non-posted
         * reads -- software has no separate Non-Posted Rx dataport, so both
         * arrive here. Memory reads and writes never do: they are serviced
         * in hardware through the windows above and are never queued.
         */
        AspeedESPIQueue rx;

        /*
         * Staging for a cycle the target originates as bus master. Unlike
         * the Rx side these are fire-and-forget: once handed to the
         * notifier the trigger self-clears and nothing is left to poll.
         */
        AspeedESPIQueue pc_tx;

        /*
         * A Non-Posted Tx cycle is a memory read request: an address and a
         * length, no payload. Giving it a full 4 KiB AspeedESPIQueue would
         * migrate 4 KiB to carry 8 bytes, so it gets its own small buffer
         * instead -- large enough for the widest (64-bit) address.
         */
        struct {
            uint8_t  buf[8];
            uint32_t pos;
        } np_tx;
    } perif;

    /* TAFS/SAFS bridge the flash channel's hardware servicing goes through */
    AspeedTAFSBState *tafsb;

    /* LPC block that the peripheral channel's I/O cycles feed to */
    AspeedLPCState *lpc;

    /*
     * The VUART, if any, whose host side this controller decodes through the
     * peripheral channel's short I/O cycles
     */
    AspeedVUARTState *vuart;

    /* Memory the DMA descriptors point into */
    MemoryRegion *dma_mr;
    AddressSpace dma_as;
    /* Bus address the DMA region is mapped at, subtracted to get an offset. */
    uint64_t dma_base;

    /* Depth of the non-posted flash request queue */
    uint32_t np_queue_depth;

    /* Set while servicing a controller request, to suppress notifying back */
    bool in_ctrl_request;

    /*
     * Last level the linked VUART's host-side SIRQ line drove, and whether
     * it changed while in_ctrl_request suppressed delivery. SIRQ is a level,
     * not an edge, so it is safe -- and per the comment at
     * aspeed_espi_end_ctrl_request(), necessary -- to defer re-announcing it
     * rather than drop it.
     */
    bool sirq_level;
    bool pme_level;
    bool sirq_dirty;

    const AspeedESPINotifier *notifier;
    void                     *notifier_opaque;

    /*
     * Capability registers. The hardware fixes these at reset, so they are
     * read-only to the guest but settable by the board. The defaults are in
     * aspeed_espi.c.
     */
    uint32_t dev_id;
    uint32_t cap_gen;
    uint32_t cap_ch0;
    uint32_t cap_ch1;
    uint32_t cap_ch2;
    uint32_t cap_ch3_0;
    uint32_t cap_ch3_1;
};

/*
 * Offer a flash cycle to the target. Returns false if the channel is not
 * ready or its non-posted queue is full, which is the controller's cue to
 * retry later. This is the entry point for the host side; nothing inside
 * the model calls it.
 */
bool aspeed_espi_flash_cycle(AspeedESPIState *s, uint8_t cyc, uint8_t tag,
                             uint32_t len, uint32_t addr,
                             const uint8_t *data, uint8_t *rsp_cyc,
                             uint8_t *rsp_data, uint32_t *rsp_len);

/*
 * Offer a peripheral-channel cycle to the target: memory reads/writes,
 * message and completion cycles, and short I/O cycles (KCS/PCC) all come
 * through here. Memory reads and writes are serviced entirely in hardware
 * through the mapping windows; the others post to the shared Rx queue or
 * route to the LPC block. Returns false if the channel is not ready.
 */
bool aspeed_espi_perif_cycle(AspeedESPIState *s, uint8_t cyc, uint8_t tag,
                             uint64_t addr, uint32_t len,
                             const uint8_t *data, uint8_t msg_code,
                             uint8_t *rsp_cyc, uint8_t *rsp_data,
                             uint32_t *rsp_len);

/*
 * Collect the completion software queued for a cycle it serviced
 * asynchronously. False means nothing is queued yet; the controller polls.
 */
bool aspeed_espi_flash_get_completion(AspeedESPIState *s, uint8_t *rsp_cyc,
                                      uint8_t *rsp_tag, uint8_t *rsp_data,
                                      uint32_t *rsp_len);

/*
 * Read the shared eSPI Status word. Reading it is what the controller does
 * to collect the target's state, so this also clears the alert condition by
 * marking the current word as the one the controller has seen.
 */
uint32_t aspeed_espi_get_status(AspeedESPIState *s);

/* True while the controller has yet to read the current status word. */
bool aspeed_espi_alert_pending(AspeedESPIState *s, uint32_t *status);

/*
 * The status word without the acknowledgement, for a response that carries it
 * as context rather than because the controller asked for status.
 */
uint32_t aspeed_espi_peek_status(AspeedESPIState *s);

/*
 * Drop the completion state a hardware-serviced read leaves behind. The
 * caller does this once it has peeked the status word for that read's own
 * response.
 */
void aspeed_espi_flash_read_done(AspeedESPIState *s);

/*
 * Move a virtual wire system event. @evt1 selects the second event register,
 * which is the one carrying the generic platform wires; the first carries the
 * standard ones. Also for the host side.
 */
void aspeed_espi_vw_set_event(AspeedESPIState *s, bool evt1, uint32_t val);
uint32_t aspeed_espi_vw_get_event(AspeedESPIState *s, bool evt1);

/*
 * Configuration cycles from the controller. @cfg_addr is an eSPI
 * configuration address, not a register offset; the two are related but not
 * the same, and only some addresses exist.
 */
uint32_t aspeed_espi_get_config(AspeedESPIState *s, uint16_t cfg_addr);
bool aspeed_espi_set_config(AspeedESPIState *s, uint16_t cfg_addr,
                            uint32_t val);

/* Install the path back to the controller. Passing NULL detaches it. */
void aspeed_espi_set_notifier(AspeedESPIState *s,
                              const AspeedESPINotifier *notifier,
                              void *opaque);

#endif /* ASPEED_ESPI_H */
