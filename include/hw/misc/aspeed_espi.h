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

/*
 * Largest payload a cycle can carry. A length field of zero means this,
 * not zero: the field is twelve bits and has to encode 4096.
 */
#define ASPEED_ESPI_MAX_PLD     4096

#define TYPE_ASPEED_ESPI "aspeed.espi"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedESPIState, ASPEED_ESPI)

/*
 * The block occupies 4 KiB of address space. Only a scattered subset of the
 * offsets is implemented; the gaps do not read as zero and do not read as
 * all-ones, but return one of four band signatures. See aspeed_espi_gap().
 */
#define ASPEED_ESPI_SIZE        0x1000
#define ASPEED_ESPI_NR_REGS     (ASPEED_ESPI_SIZE >> 2)

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
 * rather than in answer to a request: a virtual wire it drove, or a status
 * word the controller has not seen yet (Alert, on hardware).
 *
 * Neither fires while a controller request is being serviced. Every
 * response already carries the status word, and calling out mid-request
 * would mean reaching the controller while still owing it that reply.
 */
typedef struct AspeedESPINotifier {
    /* One virtual wire group as it goes on the eSPI bus: index and data */
    void (*vw_notify)(void *opaque, uint8_t index, uint8_t data);
    void (*alert_notify)(void *opaque, uint32_t espi_status);
} AspeedESPINotifier;

/* Mutable per-channel state, indexed by channel number. */
typedef struct AspeedESPIChannel {
    bool     sw_ready;      /* software has armed the channel */
    uint32_t status;        /* contribution to the shared eSPI Status word */
} AspeedESPIChannel;

/* IRQ lines, in sysbus_init_irq() order: index 0 is the eSPI interrupt. */
#define ASPEED_ESPI_IRQ_MAIN    0
#define ASPEED_ESPI_NR_IRQS     1

/*
 * Input GPIO carrying the eSPI reset signal from the host controller, either
 * the dedicated pin or an in-band reset cycle. High means reset asserted.
 */
#define ASPEED_ESPI_RESET_GPIO  "espi-reset"

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

    /* TAFS/SAFS bridge the flash channel's hardware servicing goes through */
    AspeedTAFSBState *tafsb;

    /* Memory the DMA descriptors point into */
    MemoryRegion *dma_mr;
    AddressSpace dma_as;
    /* Bus address the DMA region is mapped at, subtracted to get an offset. */
    uint64_t dma_base;

    /* Depth of the non-posted flash request queue */
    uint32_t np_queue_depth;

    /* Set while servicing a controller request, to suppress notifying back */
    bool in_ctrl_request;

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
