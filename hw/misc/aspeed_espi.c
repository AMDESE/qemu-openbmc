/*
 * ASPEED AST2700 eSPI Controller
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The eSPI block is the target side of the Enhanced Serial Peripheral
 * Interface. It presents four channels to the host controller: peripheral
 * (0), virtual wire (1), out-of-band (2) and flash (3). This models the
 * global and configuration registers, the virtual wire channel, and the
 * flash channel, whose cycles are serviced by software or, through a TAFS/SAFS
 * bridge, by hardware. The peripheral and out-of-band channel registers are
 * storage only.
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/aspeed_espi.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TO_REG(offset) ((offset) >> 2)

/* Global registers */
#define ESPI_CTRL               0x000
#define   ESPI_CTRL_WMASK           MAKE_64BIT_MASK(0, 9)
#define ESPI_STS                0x004
#define   ESPI_STS_ABORT_CNT        MAKE_64BIT_MASK(28, 4)
#define   ESPI_STS_RESET_CNT        MAKE_64BIT_MASK(16, 8)
#define   ESPI_STS_RESET_CNT_CLEAR  0xea
#define ESPI_INT_STS            0x008
#define   ESPI_INT_STS_RST_DEASSERT BIT(31)
#define   ESPI_INT_STS_RST_ASSERT   BIT(30)
#define   ESPI_INT_STS_CHAN_MASK    MAKE_64BIT_MASK(0, ASPEED_ESPI_NR_CHAN)
#define ESPI_INT_EN             0x00c
#define   ESPI_INT_EN_RST_DEASSERT  BIT(31)
#define   ESPI_INT_EN_RST_ASSERT    BIT(30)
#define ESPI_DEV_ID             0x010
#define ESPI_CAP_GEN            0x014
#define   ESPI_CAP_GEN_RTC          BIT(29)
#define ESPI_CAP_CH0            0x018
#define ESPI_CAP_CH1            0x01c
#define ESPI_CAP_CH2            0x020
#define ESPI_CAP_CH3_0          0x024
#define ESPI_CAP_CH3_1          0x028
#define ESPI_DEV_STS            0x030
#define ESPI_DBG_CMD            0x040
#define ESPI_DBG_RES            0x044
#define ESPI_CH_ACC_CTRL        0x04c
#define ESPI_CH_ACC_OFST2       0x054
#define ESPI_WPROT0             0x0f8
#define ESPI_WPROT1             0x0fc

/* Per-channel register blocks, 0x100 apart */
#define ESPI_CH0_BASE           0x100
#define ESPI_CH1_BASE           0x200
#define ESPI_CH2_BASE           0x300

/*
 * Virtual wire channel (channel 1). The channel carries system events rather
 * than data: level changes on named wires, delivered in two event registers.
 */
#define   ESPI_CH1_CTRL_READY       BIT(0)   /* driven by the channel */
/*
 * Channel interrupt status bits. On the AST2700 GPIO is bit 1 and System
 * Event 1 bit 2, and each is cleared by writing one to the other's position.
 */
#define   ESPI_CH1_INT_SYS_EVT0     BIT(0)
#define   ESPI_CH1_INT_GPIO         BIT(1)
#define   ESPI_CH1_INT_SYS_EVT1     BIT(2)
#define   ESPI_CH1_INT_GP50         BIT(3)
/*
 * The two System Event registers. 4, 8 and 0x10 bytes on from each sit the
 * interrupt enable, the three trigger type registers and the interrupt
 * status for the same wires.
 */
#define ESPI_CH1_EVT0           0x210
#define ESPI_CH1_EVT1           0x230
#define ESPI_EVT_INT_EN         0x04
#define ESPI_EVT_INT_T0         0x08
#define ESPI_EVT_INT_T1         0x0c
#define ESPI_EVT_INT_T2         0x10
#define ESPI_EVT_INT_STS        0x14
/*
 * System Event 0 bits software drives towards the controller: host reset
 * and CPU init acknowledge, boot status and errors, OOB reset acknowledge.
 * The rest are the controller's or are driven by other blocks.
 */
#define   ESPI_CH1_EVT0_WMASK       0x0cf10000
#define   ESPI_CH1_EVT0_INT_EN_MASK MAKE_64BIT_MASK(0, 16)
/*
 * System Event 1: software drives the BMC generic byte and Suspend
 * Acknowledge; the controller drives the rest, including its own generic
 * byte.
 */
#define   ESPI_CH1_EVT1_WMASK       0xff100000
#define   ESPI_CH1_EVT1_INT_EN_MASK MAKE_64BIT_MASK(0, 20)
#define ESPI_CH3_BASE           0x400

/* Virtual wire encoding on the eSPI bus */
#define ESPI_VW_VALID(wire)     BIT((wire) + 4)

/* Flash channel (channel 3) */
#define ESPI_CH3_CTRL           0x400
#define   ESPI_CH3_CTRL_TX_RST      BIT(31)
#define   ESPI_CH3_CTRL_RX_RST      BIT(30)
#define   ESPI_CH3_CTRL_TX_DMA_EN   BIT(17)
#define   ESPI_CH3_CTRL_RX_DMA_EN   BIT(16)
#define   ESPI_CH3_CTRL_SAFS_HW_RST BIT(15)  /* active low */
#define   ESPI_CH3_CTRL_EDAF_MODE   MAKE_64BIT_MASK(8, 2)
#define   ESPI_CH3_CTRL_READY       BIT(4)   /* driven by the channel */
#define ESPI_CH3_STS_ERASE          MAKE_64BIT_MASK(12, 3)
#define ESPI_CH3_INT_STS_RX_CMPLT   BIT(0)
#define ESPI_CH3_INT_STS_TX_CMPLT   BIT(1)
#define ESPI_CH3_INT_STS_TX_ERR     BIT(4)
#define ESPI_CH3_RX_DMAL        0x410
#define ESPI_CH3_RX_CTRL        0x418
#define ESPI_CH3_RX_DATA        0x41c
#define ESPI_CH3_TX_DMAL        0x420
#define ESPI_CH3_TX_CTRL        0x428
#define ESPI_CH3_TX_DATA        0x42c
#define ESPI_CH3_EDAF_TADDRL    0x430
#define ESPI_CH3_EDAF_TADDRH    0x434
#define ESPI_CH3_EDAF_MASKL     0x438
#define ESPI_CH3_EDAF_MASKH     0x43c
/* Only the top byte of the low target address and mask words is writable */
#define   ESPI_CH3_EDAF_LO_WMASK    MAKE_64BIT_MASK(24, 8)

/*
 * Cycle descriptor, shared by the receive and transmit control registers of
 * the flash and out-of-band channels.
 */
#define ESPI_CYC_SERV_PEND      BIT(31)
#define ESPI_CYC_TRIG_PEND      BIT(31)
#define ESPI_CYC_LEN            MAKE_64BIT_MASK(12, 12)
#define ESPI_CYC_TAG            MAKE_64BIT_MASK(8, 4)
#define ESPI_CYC_TYPE           MAKE_64BIT_MASK(0, 8)

/*
 * How the flash channel services cycles. In mixed mode the hardware answers
 * reads on its own and leaves writes and erases to software, which is why a
 * platform with no software agent still boots: booting only reads.
 */
#define ESPI_EDAF_MODE_MIX      0
#define ESPI_EDAF_MODE_SW       1
#define ESPI_EDAF_MODE_HW       2

/* Flash channel cycle types */
#define ESPI_FLASH_READ                 0x00
#define ESPI_FLASH_WRITE                0x01
#define ESPI_FLASH_ERASE                0x02
#define ESPI_FLASH_SUC_CMPLT            0x06
#define ESPI_FLASH_SUC_CMPLT_D_MIDDLE   0x09
#define ESPI_FLASH_SUC_CMPLT_D_FIRST    0x0b
#define ESPI_FLASH_UNSUC_CMPLT_LAST     0x0c
#define ESPI_FLASH_SUC_CMPLT_D_LAST     0x0d
#define ESPI_FLASH_UNSUC_CMPLT_ONLY     0x0e
#define ESPI_FLASH_SUC_CMPLT_D_ONLY     0x0f

/*
 * Erase length encoding for a target-attached flash configuration. The value
 * selects a size rather than counting bytes, and the encoding differs from
 * the controller-attached one, so it must not be shared with it.
 */
static const uint32_t espi_erase_size_kb[] = { 4, 32, 64, 128 };

/*
 * The capability register advertises erase sizes as a bitmap in bits [15:8],
 * where bit N means 2^N KiB.
 */
#define ESPI_ERASE_BITMAP_SHIFT 8

/*
 * Offsets within a channel register block. The layout is common to all four
 * channels; only the SW_RDY bit position in the control register differs.
 */
#define ESPI_CHAN_CTRL          0x00
#define ESPI_CHAN_STS           0x04
/*
 * Every channel's status register mirrors two things the channel does not
 * own: whether the controller has enabled the link at all, and whether it
 * has enabled this channel.
 */
#define   ESPI_CHAN_STS_CTRL_EN     BIT(0)
#define   ESPI_CHAN_STS_CHAN_EN     BIT(4)
#define ESPI_CHAN_INT_STS       0x08
#define ESPI_CHAN_INT_EN        0x0c

/*
 * eSPI Status word bits. In SAFS the target owns the non-posted queue and
 * produces completions, so NP_FREE reports queue space and C_AVAIL reports a
 * completion waiting. The virtual wire channel and the flash channel's
 * completion queue always have room, and the word comes up with only those.
 */
#define ESPI_STATUS_VW_FREE         BIT(2)
#define ESPI_STATUS_FLASH_C_FREE    BIT(8)
#define ESPI_STATUS_FLASH_NP_FREE   BIT(9)
#define ESPI_STATUS_FLASH_C_AVAIL   BIT(12)
#define ESPI_STATUS_IDLE            (ESPI_STATUS_VW_FREE | \
                                     ESPI_STATUS_FLASH_C_FREE)

/*
 * SW_RDY lives at a different bit in each channel control register, and each
 * channel's control and status registers come up with a reset value of
 * the channel's own.
 *
 * The two topmost control bits of every channel are the Tx and Rx queue
 * software resets, and they are active low: clearing one holds that queue in
 * reset, setting it releases the queue. They come up released.
 */
static const AspeedESPIChannelInfo
aspeed_espi_chan_info[ASPEED_ESPI_NR_CHAN] = {
    [ASPEED_ESPI_CHAN_PERIF] = {
        .name = "peripheral", .base = ESPI_CH0_BASE, .sw_rdy = BIT(1),
        .ctrl_reset = 0xf0000350, .sts_reset = 0xfd000001,
    },
    [ASPEED_ESPI_CHAN_VW] = {
        .name = "virtual wire", .base = ESPI_CH1_BASE, .sw_rdy = BIT(1),
        .ctrl_reset = 0x800001f0, .sts_reset = 0x00000001,
        .ctrl_ro = ESPI_CH1_CTRL_READY,
    },
    [ASPEED_ESPI_CHAN_OOB] = {
        .name = "out-of-band", .base = ESPI_CH2_BASE, .sw_rdy = BIT(4),
        .ctrl_reset = 0xc0000001, .sts_reset = 0x00000001,
    },
    [ASPEED_ESPI_CHAN_FLASH] = {
        .name = "flash", .base = ESPI_CH3_BASE, .sw_rdy = BIT(5),
        .ctrl_reset = 0xc0008201, .sts_reset = 0x00001001,
        .ctrl_ro = ESPI_CH3_CTRL_READY,
    },
};

/*
 * SAFS filter block: a control register, an interrupt register, and sixteen
 * start and end address pairs. A region covers an address range at 16 KiB
 * granularity and gates reads and writes within it separately; the control
 * register decides whether a region describes what is denied or what is
 * permitted, and carries the size of the shared flash.
 */
#define ESPI_FLTR_CTRL          0x500
#define   ESPI_FLTR_CTRL_SIZE       MAKE_64BIT_MASK(16, 9)
#define   ESPI_FLTR_CTRL_SIZE_SHIFT 19
#define   ESPI_FLTR_CTRL_ALLOW      BIT(0)
#define ESPI_FLTR_INT           0x504
#define ESPI_FLTR_FIRST         0x510
#define ESPI_FLTR_LAST          0x58c
#define ESPI_FLTR_NR_REGIONS    16
#define   ESPI_FLTR_ADDR            MAKE_64BIT_MASK(14, 18)
#define   ESPI_FLTR_WRITE           BIT(2)
#define   ESPI_FLTR_READ            BIT(1)
#define   ESPI_FLTR_VALID           BIT(0)
#define ESPI_FLTR_WPROT0        0x5f8
#define ESPI_FLTR_WPROT1        0x5fc

/*
 * The AST2700 comes up with a shared flash size of 16 MiB in the filter
 * control register, and with every region's start address all ones and its
 * read and write control set.
 */
#define ESPI_FLTR_CTRL_RESET    0x00200000
#define ESPI_FLTR_SADDR_RESET   0xffffc006
#define ESPI_FLTR_EADDR_RESET   0x00000000

/*
 * Reset values of the global block. Capability registers are fixed at reset,
 * so each is a property a board can override. 0x6401 in the second flash
 * channel capability register advertises 4, 32 and 64 KiB erase.
 */
/* Set while the controller has the link enabled */
#define   ESPI_STS_CTRL_EN          BIT(0)

#define ESPI_CTRL_RESET         0x00000006
#define ESPI_STS_RESET          0x00000001
#define ESPI_DBG_CMD_RESET      0x0000ff00
#define ESPI_DBG_RES_RESET      0x000000ff
#define ESPI_CH_ACC_CTRL_RESET  0x00000007
#define ESPI_DEV_ID_RESET       0x00000001
#define ESPI_CAP_GEN_RESET      0x230c000f
#define ESPI_CAP_CH0_RESET      0x00001110
#define ESPI_CAP_CH1_RESET      0x00000700
#define ESPI_CAP_CH2_RESET      0x00000110
#define ESPI_CAP_CH3_0_RESET    0x00031924
#define ESPI_CAP_CH3_1_RESET    0x00006401
#define ESPI_DEV_STS_RESET      0x01040104

/*
 * Flash channel mapping window. The mask reaches 40 bits, so its reset
 * describes a window rather than an open one.
 */
#define ESPI_CH3_TADDRH_RESET   0x00000020
#define ESPI_CH3_MASKL_RESET    0xff000000
#define ESPI_CH3_MASKH_RESET    0xffffffff

/* Virtual wire system event register 0 */
#define ESPI_CH1_EVT0_RESET     0x070c0600

static bool aspeed_espi_offset_implemented(hwaddr offset)
{
    /* Global block */
    if (offset <= ESPI_CAP_CH3_1 ||
        (offset >= ESPI_DEV_STS && offset <= ESPI_DBG_RES) ||
        (offset >= ESPI_CH_ACC_CTRL && offset <= ESPI_CH_ACC_OFST2) ||
        (offset >= ESPI_WPROT0 && offset <= ESPI_WPROT1)) {
        return true;
    }

    /* Virtual wire channel */
    if ((offset >= ESPI_CH1_BASE && offset <= 0x224) ||
        (offset >= 0x230 && offset <= 0x244) ||
        (offset >= 0x250 && offset <= 0x268) ||
        (offset >= 0x270 && offset <= 0x288)) {
        return true;
    }

    /* Peripheral channel control, status and interrupt registers */
    if (offset >= ESPI_CH0_BASE && offset <= 0x10c) {
        return true;
    }

    /* Out-of-band channel, including its descriptor ring pointers */
    if (offset >= ESPI_CH2_BASE && offset <= 0x348) {
        return true;
    }

    /* Flash channel, including the eDAF mapping window */
    if (offset >= ESPI_CH3_BASE && offset <= 0x43c) {
        return true;
    }

    /* SAFS filter */
    if ((offset >= ESPI_FLTR_FIRST && offset <= ESPI_FLTR_LAST) ||
        (offset >= ESPI_FLTR_WPROT0 && offset <= ESPI_FLTR_WPROT1)) {
        return true;
    }

    /* Per-channel write-protect pairs sit at the top of each block */
    switch (offset) {
    case 0x1f8 ... 0x1fc:
    case 0x2f8 ... 0x2fc:
    case 0x3f8 ... 0x3fc:
    case 0x4f8 ... 0x4fc:
    case ESPI_FLTR_CTRL:
    case ESPI_FLTR_INT:
        return true;
    default:
        return false;
    }
}

/*
 * Registers the guest may not write: the capability registers, fixed at
 * reset, and the status and debug result registers. On the AST2700 the
 * second flash channel capability register is writable, so software can
 * narrow the erase sizes it advertises.
 */
static bool aspeed_espi_offset_ro(hwaddr offset)
{
    switch (offset) {
    case ESPI_DEV_ID:
    case ESPI_CAP_CH0:
    case ESPI_CAP_CH1:
    case ESPI_CAP_CH2:
    case ESPI_CAP_CH3_0:
    case ESPI_DEV_STS:
    case ESPI_DBG_CMD:
    case ESPI_DBG_RES:
        return true;
    default:
        return false;
    }
}

static void aspeed_espi_flash_update_status(AspeedESPIState *s);

/*
 * Configuration space, as the controller addresses it.  Each entry is
 * mirrored by one register in the block, which is how software reads back
 * what the controller negotiated.
 */
#define ESPI_CFG_DEV_ID         0x04
#define ESPI_CFG_GEN            0x08
#define ESPI_CFG_CH0            0x10
#define ESPI_CFG_CH1            0x20
#define ESPI_CFG_CH2            0x30
#define ESPI_CFG_CH3            0x40
#define ESPI_CFG_CH3_1          0x44

/* Common to every channel's configuration dword */
#define ESPI_CFG_CHAN_ENABLE    BIT(0)
/* The erase size the two ends settle on, in the flash channel's config */
#define ESPI_CFG_CH3_ERASE      MAKE_64BIT_MASK(2, 3)
#define ESPI_CFG_CHAN_READY     BIT(1)

/* Backing register for a configuration address, or -1 if it has none */
static int aspeed_espi_config_reg(uint16_t cfg_addr)
{
    switch (cfg_addr) {
    case ESPI_CFG_DEV_ID:
        return ESPI_DEV_ID;
    case ESPI_CFG_GEN:
        return ESPI_CAP_GEN;
    case ESPI_CFG_CH0:
        return ESPI_CAP_CH0;
    case ESPI_CFG_CH1:
        return ESPI_CAP_CH1;
    case ESPI_CFG_CH2:
        return ESPI_CAP_CH2;
    case ESPI_CFG_CH3:
        return ESPI_CAP_CH3_0;
    case ESPI_CFG_CH3_1:
        return ESPI_CAP_CH3_1;
    default:
        return -1;
    }
}

/* Channel a configuration address belongs to, or -1 for the global ones */
static int aspeed_espi_config_chan(uint16_t cfg_addr)
{
    switch (cfg_addr) {
    case ESPI_CFG_CH0:
        return ASPEED_ESPI_CHAN_PERIF;
    case ESPI_CFG_CH1:
        return ASPEED_ESPI_CHAN_VW;
    case ESPI_CFG_CH2:
        return ASPEED_ESPI_CHAN_OOB;
    case ESPI_CFG_CH3:
        return ASPEED_ESPI_CHAN_FLASH;
    default:
        return -1;
    }
}

/*
 * A channel reports itself ready once the controller has enabled it and
 * software has armed it.  The controller polls this bit after enabling a
 * channel, so it has to track both halves.
 */
/*
 * Rebuild the fields of a channel's control and status registers that the
 * channel drives rather than software: whether the controller has enabled the
 * link and this channel, and for the flash channel the negotiated erase size.
 * Keeping this the only writer is what stops a guest write leaving a stale
 * value in a field the guest does not own.
 */
static void aspeed_espi_chan_refresh(AspeedESPIState *s, int chan)
{
    const AspeedESPIChannelInfo *info = &aspeed_espi_chan_info[chan];
    int cfg_reg = aspeed_espi_config_reg(ESPI_CFG_CH0 + (chan << 4));
    uint32_t sts = s->regs[TO_REG(info->base + ESPI_CHAN_STS)];
    uint32_t cfg;

    if (cfg_reg < 0) {
        return;
    }
    cfg = s->regs[TO_REG(cfg_reg)];

    sts &= ~(ESPI_CHAN_STS_CTRL_EN | ESPI_CHAN_STS_CHAN_EN);
    if (s->regs[TO_REG(ESPI_STS)] & ESPI_STS_CTRL_EN) {
        sts |= ESPI_CHAN_STS_CTRL_EN;
    }
    if (cfg & ESPI_CFG_CHAN_ENABLE) {
        sts |= ESPI_CHAN_STS_CHAN_EN;
    }

    if (chan == ASPEED_ESPI_CHAN_VW) {
        uint32_t ctrl = s->regs[TO_REG(info->base + ESPI_CHAN_CTRL)];

        ctrl &= ~ESPI_CH1_CTRL_READY;
        if (cfg & ESPI_CFG_CHAN_READY) {
            ctrl |= ESPI_CH1_CTRL_READY;
        }
        s->regs[TO_REG(info->base + ESPI_CHAN_CTRL)] = ctrl;
    }

    if (chan == ASPEED_ESPI_CHAN_FLASH) {
        uint32_t ctrl = s->regs[TO_REG(info->base + ESPI_CHAN_CTRL)];

        /* Both registers report the erase size in the same encoding */
        sts &= ~ESPI_CH3_STS_ERASE;
        sts |= (cfg & ESPI_CFG_CH3_ERASE) << 10;

        ctrl &= ~ESPI_CH3_CTRL_READY;
        if (cfg & ESPI_CFG_CHAN_READY) {
            ctrl |= ESPI_CH3_CTRL_READY;
        }
        s->regs[TO_REG(info->base + ESPI_CHAN_CTRL)] = ctrl;
    }

    s->regs[TO_REG(info->base + ESPI_CHAN_STS)] = sts;
}

static void aspeed_espi_config_update_ready(AspeedESPIState *s, int chan)
{
    int reg = aspeed_espi_config_reg(ESPI_CFG_CH0 + (chan << 4));
    uint32_t val;
    bool ready;

    if (reg < 0) {
        return;
    }

    val = s->regs[TO_REG(reg)];
    ready = (val & ESPI_CFG_CHAN_ENABLE) && s->chan[chan].sw_ready;

    if (ready) {
        val |= ESPI_CFG_CHAN_READY;
    } else {
        val &= ~ESPI_CFG_CHAN_READY;
    }

    if (val != s->regs[TO_REG(reg)]) {
        s->regs[TO_REG(reg)] = val;
        trace_aspeed_espi_chan_config(aspeed_espi_chan_info[chan].name,
                                      val, ready);
    }

    aspeed_espi_chan_refresh(s, chan);
}

uint32_t aspeed_espi_get_config(AspeedESPIState *s, uint16_t cfg_addr)
{
    int reg = aspeed_espi_config_reg(cfg_addr);

    if (reg < 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: read of unknown configuration address 0x%x\n",
                      __func__, cfg_addr);
        return 0;
    }

    return s->regs[TO_REG(reg)];
}

/*
 * Which bits of a configuration register a controller may set: the operating
 * sizes and counts, the flash sharing mode, the erase size, and each
 * channel's enable. The rest describe what the target supports and whether it
 * is ready, and a write leaves them alone; covering the whole word would lose
 * the target's own advertisement the moment the controller negotiates. On the
 * AST2700 the controller can also set the flash channel's sharing mode
 * support field, bits 7:5.
 */
#define ESPI_CFG_GEN_WMASK      0xdcf0f000
#define ESPI_CFG_CH0_WMASK      0x00007705
#define ESPI_CFG_CH1_WMASK      0x003f0001
#define ESPI_CFG_CH2_WMASK      0x00000701
#define ESPI_CFG_CH3_WMASK      0x00007ffd

static uint32_t aspeed_espi_config_mask(uint16_t cfg_addr)
{
    switch (cfg_addr) {
    case ESPI_CFG_GEN:
        return ESPI_CFG_GEN_WMASK;
    case ESPI_CFG_CH0:
        return ESPI_CFG_CH0_WMASK;
    case ESPI_CFG_CH1:
        return ESPI_CFG_CH1_WMASK;
    case ESPI_CFG_CH2:
        return ESPI_CFG_CH2_WMASK;
    case ESPI_CFG_CH3:
        return ESPI_CFG_CH3_WMASK;
    default:
        return 0xffffffff;
    }
}

bool aspeed_espi_set_config(AspeedESPIState *s, uint16_t cfg_addr,
                            uint32_t val)
{
    int reg = aspeed_espi_config_reg(cfg_addr);
    uint32_t mask;
    int chan;

    if (reg < 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write of unknown configuration address 0x%x\n",
                      __func__, cfg_addr);
        return false;
    }

    if (cfg_addr == ESPI_CFG_DEV_ID || cfg_addr == ESPI_CFG_CH3_1) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: configuration address 0x%x is read-only\n",
                      __func__, cfg_addr);
        return false;
    }

    /* The ready bit belongs to the target, not the controller. */
    mask = aspeed_espi_config_mask(cfg_addr) & ~ESPI_CFG_CHAN_READY;
    val &= mask;
    s->regs[TO_REG(reg)] = val | (s->regs[TO_REG(reg)] & ~mask);

    chan = aspeed_espi_config_chan(cfg_addr);
    if (chan >= 0) {
        aspeed_espi_config_update_ready(s, chan);
    }

    trace_aspeed_espi_set_config(cfg_addr, val);
    return true;
}

/* Channel owning a register offset, or -1 if the offset is not a channel's */
static int aspeed_espi_offset_chan(hwaddr offset)
{
    int chan;

    if (offset < ESPI_CH0_BASE || offset >= ESPI_CH0_BASE +
        (ASPEED_ESPI_NR_CHAN << 8)) {
        return -1;
    }

    chan = (offset >> 8) - 1;
    g_assert(chan >= 0 && chan < ASPEED_ESPI_NR_CHAN);

    return chan;
}

static uint32_t aspeed_espi_chan_reg(AspeedESPIState *s, int chan,
                                     hwaddr chan_offset)
{
    return s->regs[TO_REG(aspeed_espi_chan_info[chan].base + chan_offset)];
}

/*
 * Recompose the shared eSPI Status word from every channel, and compare it
 * with the copy the host last saw. A difference is what makes the target
 * signal Alert, so channels never poke the status word or the alert state
 * directly; each updates its contribution and calls this.
 */
static void aspeed_espi_update_status(AspeedESPIState *s)
{
    uint32_t status = ESPI_STATUS_IDLE;
    int i;

    for (i = 0; i < ASPEED_ESPI_NR_CHAN; i++) {
        status |= s->chan[i].status;
    }

    if (status == s->status) {
        return;
    }

    s->status = status;
    trace_aspeed_espi_status(status, s->status_shadow);

    if (status != s->status_shadow) {
        trace_aspeed_espi_alert(s->status_shadow, status);

        /*
         * Tell the controller something is waiting to be collected, the
         * same way a target signals Alert. Never while servicing a controller
         * request -- see the notifier's doc comment; the controller may well
         * be waiting on this same call's own reply.
         */
        if (!s->in_ctrl_request && s->notifier && s->notifier->alert_notify) {
            s->notifier->alert_notify(s->notifier_opaque, status);
        }
    }
}

/*
 * Every exit point of a controller-request handler must go through this
 * instead of writing in_ctrl_request directly.
 */
static void aspeed_espi_end_ctrl_request(AspeedESPIState *s)
{
    s->in_ctrl_request = false;
}

/*
 * Recompute the interrupt state.
 *
 * The per-channel summary bits in the global interrupt status register are
 * not masked: hardware sets one while that channel's own enable leaves the
 * pending status bit uncovered. Masking applies to delivery instead, per
 * channel, which is also what stops a permanently summarised bit from
 * spinning the handler.
 */
static void aspeed_espi_update_irq(AspeedESPIState *s)
{
    uint32_t global_sts = s->regs[TO_REG(ESPI_INT_STS)];
    uint32_t global_en = s->regs[TO_REG(ESPI_INT_EN)];
    uint32_t summary = 0;
    bool assert_irq = false;
    int i;

    for (i = 0; i < ASPEED_ESPI_NR_CHAN; i++) {
        uint32_t sts = aspeed_espi_chan_reg(s, i, ESPI_CHAN_INT_STS);
        uint32_t en = aspeed_espi_chan_reg(s, i, ESPI_CHAN_INT_EN);

        if (sts) {
            summary |= BIT(i);
        }
        if (sts & en) {
            assert_irq = true;
        }
    }

    global_sts &= ~ESPI_INT_STS_CHAN_MASK;
    global_sts |= summary;
    s->regs[TO_REG(ESPI_INT_STS)] = global_sts;

    if (global_sts & global_en &
        (ESPI_INT_STS_RST_ASSERT | ESPI_INT_STS_RST_DEASSERT)) {
        assert_irq = true;
    }

    trace_aspeed_espi_irq(global_sts, global_en, assert_irq);
    qemu_set_irq(s->irq[ASPEED_ESPI_IRQ_MAIN], assert_irq);
}

static void aspeed_espi_flash_update_window(AspeedESPIState *s);

/*
 * Bring one channel back to its post-reset state. Queues, status and pending
 * interrupts are dropped, and the channel comes back disabled.
 *
 * A configuration register mixes two kinds of field. What the target supports
 * is fixed at reset from straps and survives, so those are left alone. The
 * enable bit is not: the controller sets it during the configuration sequence
 * it runs after every reset, so a channel that keeps it would be usable
 * without the controller ever having asked for it.
 */
static void aspeed_espi_chan_reset(AspeedESPIState *s, int chan)
{
    const AspeedESPIChannelInfo *info = &aspeed_espi_chan_info[chan];
    int cfg_reg = aspeed_espi_config_reg(ESPI_CFG_CH0 + (chan << 4));

    s->regs[TO_REG(info->base + ESPI_CHAN_STS)] = info->sts_reset;
    s->regs[TO_REG(info->base + ESPI_CHAN_INT_STS)] = 0;
    s->regs[TO_REG(info->base + ESPI_CHAN_INT_EN)] = 0;
    s->regs[TO_REG(info->base + ESPI_CHAN_CTRL)] &= ~info->sw_rdy;

    if (cfg_reg >= 0) {
        s->regs[TO_REG(cfg_reg)] &= ~(ESPI_CFG_CHAN_ENABLE |
                                      ESPI_CFG_CHAN_READY);
    }

    s->chan[chan].sw_ready = false;
    s->chan[chan].status = 0;

    /*
     * A link reset drops whatever the flash channel had staged or queued;
     * otherwise a stale tx_pending or rx_buf survives the link going down
     * and reappears once software re-arms the channel. The mapping window
     * registers themselves are not cleared above, so the cached taddr/mask
     * this wipes must be rebuilt from them, or the channel decodes nothing
     * until software rewrites a mapping register that never actually
     * changed value.
     */
    if (chan == ASPEED_ESPI_CHAN_FLASH) {
        memset(&s->flash, 0, sizeof(s->flash));
        s->regs[TO_REG(ESPI_CH3_TX_CTRL)] &= ~ESPI_CYC_TRIG_PEND;
        s->regs[TO_REG(ESPI_CH3_RX_CTRL)] &= ~ESPI_CYC_SERV_PEND;
        aspeed_espi_flash_update_window(s);
    }

    aspeed_espi_chan_refresh(s, chan);
    trace_aspeed_espi_chan_reset(info->name);
}

/*
 * eSPI link reset de-assertion. Every channel is torn down, then the event
 * is reported so software can rearm them.
 */
static void aspeed_espi_link_reset(AspeedESPIState *s)
{
    int i;

    for (i = 0; i < ASPEED_ESPI_NR_CHAN; i++) {
        aspeed_espi_chan_reset(s, i);
    }

    s->status_shadow = ESPI_STATUS_IDLE;
    aspeed_espi_update_status(s);

    s->regs[TO_REG(ESPI_INT_STS)] |= ESPI_INT_STS_RST_DEASSERT;
    aspeed_espi_update_irq(s);

    trace_aspeed_espi_link_reset();
}

/*
 * The eSPI reset signal, driven by the host controller either on the
 * dedicated pin or by an in-band reset cycle. High means reset asserted.
 * Hardware reports both edges, so both are modelled: the assertion is only
 * an interrupt event, the de-assertion tears the channels down.
 */
static void aspeed_espi_reset_line(void *opaque, int n, int level)
{
    AspeedESPIState *s = ASPEED_ESPI(opaque);

    if (level) {
        uint32_t sts = s->regs[TO_REG(ESPI_STS)];

        s->regs[TO_REG(ESPI_STS)] =
            deposit32(sts, 16, 8, extract32(sts, 16, 8) + 1);
        s->regs[TO_REG(ESPI_INT_STS)] |= ESPI_INT_STS_RST_ASSERT;
        aspeed_espi_update_irq(s);
    } else {
        aspeed_espi_link_reset(s);
    }
}

/*
 * Recompute the flash channel mapping window.
 *
 * Both halves are register pairs holding a 64-bit value low word first. The
 * mask is built by software as the complement of the window size less one,
 * so an offset lies inside the window when it has no bits in common with the
 * mask.
 */
static void aspeed_espi_flash_update_window(AspeedESPIState *s)
{
    s->flash.taddr =
        ((uint64_t)s->regs[TO_REG(ESPI_CH3_EDAF_TADDRH)] << 32) |
        s->regs[TO_REG(ESPI_CH3_EDAF_TADDRL)];
    s->flash.mask =
        ((uint64_t)s->regs[TO_REG(ESPI_CH3_EDAF_MASKH)] << 32) |
        s->regs[TO_REG(ESPI_CH3_EDAF_MASKL)];

    trace_aspeed_espi_flash_window(s->flash.taddr, s->flash.mask,
                                   ~s->flash.mask + 1);
}

static uint32_t aspeed_espi_flash_mode(AspeedESPIState *s)
{
    return (s->regs[TO_REG(ESPI_CH3_CTRL)] & ESPI_CH3_CTRL_EDAF_MODE) >> 8;
}

/*
 * The flash channel half of a control register write. SW_RDY is handled by
 * the generic channel code; this deals with what is specific to channel 3.
 */
static void aspeed_espi_flash_ctrl_write(AspeedESPIState *s, uint32_t old)
{
    static const char * const modes[] = {
        [ESPI_EDAF_MODE_MIX] = "mixed",
        [ESPI_EDAF_MODE_SW]  = "software",
        [ESPI_EDAF_MODE_HW]  = "hardware",
    };
    uint32_t val = s->regs[TO_REG(ESPI_CH3_CTRL)];
    uint32_t fell = old & ~val;
    uint32_t mode = aspeed_espi_flash_mode(s);

    /* Active-low queue resets */
    if (fell & ESPI_CH3_CTRL_RX_RST) {
        s->flash.rx_len = 0;
        s->flash.rx_pos = 0;
        s->flash.np_pending = 0;
        s->regs[TO_REG(ESPI_CH3_RX_CTRL)] &= ~ESPI_CYC_SERV_PEND;
        s->regs[TO_REG(ESPI_CH3_BASE + ESPI_CHAN_INT_STS)] &=
            ~ESPI_CH3_INT_STS_RX_CMPLT;
        aspeed_espi_update_irq(s);
    }
    if (fell & ESPI_CH3_CTRL_TX_RST) {
        s->flash.tx_len = 0;
        s->flash.tx_pos = 0;
        s->flash.tx_pending = false;
        s->regs[TO_REG(ESPI_CH3_TX_CTRL)] &= ~ESPI_CYC_TRIG_PEND;
    }

    if (mode != (old & ESPI_CH3_CTRL_EDAF_MODE) >> 8) {
        trace_aspeed_espi_flash_mode(mode < ARRAY_SIZE(modes) && modes[mode] ?
                                     modes[mode] : "reserved");
        if (mode >= ARRAY_SIZE(modes) || !modes[mode]) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: reserved flash channel mode %u\n",
                          __func__, mode);
        }
    }

    aspeed_espi_flash_update_status(s);
}

/*
 * Target-to-controller system event wires (eSPI indices 4-6) and the System
 * Event register (ESPI210) bit each is driven from; -1 is a reserved wire.
 */
static const struct {
    uint8_t index;
    int8_t  evt_bit[4];
} aspeed_espi_evt0_wires[] = {
    { 4, { 16, -1, 18, 19 } },  /* OOB_RST_ACK, -, WAKE#, PME# */
    { 5, { 20, 21, 22, 23 } },  /* BOOT_LOAD_DONE, FATAL, NONFATAL, LOAD_STS */
    { 6, { 24, 25, 26, 27 } },  /* SCI#, SMI#, RCIN#, HOST_RST_ACK */
};

/*
 * Send the wires software changed in a System Event register as virtual
 * wire groups, marking only the changed wires valid.
 */
static void aspeed_espi_vw_report(AspeedESPIState *s, hwaddr evt_reg,
                                  uint32_t changed, uint32_t val)
{
    int i, w;

    if (!s->notifier || !s->notifier->vw_notify) {
        return;
    }

    if (evt_reg != ESPI_CH1_EVT0) {
        qemu_log_mask(LOG_UNIMP, "%s: System Event 1 wires 0x%08" PRIx32
                      " have no eSPI virtual wire index, not sent\n",
                      __func__, changed);
        return;
    }

    for (i = 0; i < ARRAY_SIZE(aspeed_espi_evt0_wires); i++) {
        uint8_t data = 0;

        for (w = 0; w < 4; w++) {
            int bit = aspeed_espi_evt0_wires[i].evt_bit[w];

            if (bit >= 0 && (changed & BIT(bit))) {
                data |= ESPI_VW_VALID(w) | (extract32(val, bit, 1) << w);
            }
        }
        if (data) {
            s->notifier->vw_notify(s->notifier_opaque,
                                   aspeed_espi_evt0_wires[i].index, data);
        }
    }
}

/*
 * Which wires of a System Event register trigger on a change from @old to
 * @val. Each wire's trigger type is spread over three registers, one bit per
 * wire in each: with type 2 set it triggers on both edges, otherwise types 1
 * and 0 select falling edge (00), rising edge (01), low level (10) or high
 * level (11). A level trigger fires while the wire holds the level.
 */
static uint32_t aspeed_espi_vw_triggered(AspeedESPIState *s, hwaddr evt_reg,
                                         uint32_t old, uint32_t val)
{
    uint32_t t0 = s->regs[TO_REG(evt_reg + ESPI_EVT_INT_T0)];
    uint32_t t1 = s->regs[TO_REG(evt_reg + ESPI_EVT_INT_T1)];
    uint32_t t2 = s->regs[TO_REG(evt_reg + ESPI_EVT_INT_T2)];
    uint32_t changed = old ^ val;
    uint32_t edge = t2 | ~t1;
    uint32_t hit;

    /* Edge triggers: both edges, else rising or falling by type 0 */
    hit = edge & changed & (t2 | (t0 & val) | (~t0 & old));
    /* Level triggers: high or low by type 0 */
    hit |= ~edge & ((t0 & val) | (~t0 & ~val));

    return hit & s->regs[TO_REG(evt_reg + ESPI_EVT_INT_EN)];
}

/* Latch the triggered wires and raise the channel interrupt for them */
static void aspeed_espi_vw_latch(AspeedESPIState *s, hwaddr evt_reg,
                                 uint32_t hit)
{
    uint32_t chan_int = evt_reg == ESPI_CH1_EVT0 ? ESPI_CH1_INT_SYS_EVT0 :
                                                   ESPI_CH1_INT_SYS_EVT1;

    if (!hit) {
        return;
    }
    s->regs[TO_REG(evt_reg + ESPI_EVT_INT_STS)] |= hit;
    s->regs[TO_REG(ESPI_CH1_BASE + ESPI_CHAN_INT_STS)] |= chan_int;
    aspeed_espi_update_irq(s);
}

/*
 * A level trigger stays pending while the wire holds the level, so clearing
 * its status, enabling it or changing its type checks it again.
 */
static void aspeed_espi_vw_check_levels(AspeedESPIState *s, hwaddr evt_reg)
{
    uint32_t val = s->regs[TO_REG(evt_reg)];

    aspeed_espi_vw_latch(s, evt_reg,
                         aspeed_espi_vw_triggered(s, evt_reg, val, val));
}

/*
 * Deliver a change of a System Event register, from the controller or from
 * software, and latch the wires it triggers.
 */
static void aspeed_espi_vw_event(AspeedESPIState *s, hwaddr evt_reg,
                                 uint32_t val)
{
    uint32_t old = s->regs[TO_REG(evt_reg)];
    uint32_t changed = old ^ val;
    uint32_t raised = aspeed_espi_vw_triggered(s, evt_reg, old, val);

    s->regs[TO_REG(evt_reg)] = val;

    if (!changed) {
        return;
    }

    trace_aspeed_espi_vw_event(evt_reg, old, val, raised);

    /*
     * Report a wire the target drove. Never while servicing a controller
     * request: that would call out to the controller while still owing it a
     * reply, and it may well be waiting for exactly that reply.
     */
    if (!s->in_ctrl_request) {
        aspeed_espi_vw_report(s, evt_reg, changed, val);
    }

    aspeed_espi_vw_latch(s, evt_reg, raised);
}

/*
 * Drive a system event towards the controller, or observe one it drove. The
 * host side uses this; software sees the result in the event registers.
 */
void aspeed_espi_vw_set_event(AspeedESPIState *s, bool evt1, uint32_t val)
{
    if (!s->chan[ASPEED_ESPI_CHAN_VW].sw_ready) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: virtual wire event while the channel is not "
                      "ready\n", __func__);
        return;
    }

    s->in_ctrl_request = true;
    if (evt1) {
        /* The controller drives every wire but the target's own */
        val = (val & ~ESPI_CH1_EVT1_WMASK) |
              (s->regs[TO_REG(ESPI_CH1_EVT1)] & ESPI_CH1_EVT1_WMASK);
        aspeed_espi_vw_event(s, ESPI_CH1_EVT1, val);
    } else {
        val = (val & ~ESPI_CH1_EVT0_WMASK) |
              (s->regs[TO_REG(ESPI_CH1_EVT0)] & ESPI_CH1_EVT0_WMASK);
        aspeed_espi_vw_event(s, ESPI_CH1_EVT0, val);
    }
    aspeed_espi_end_ctrl_request(s);
}

void aspeed_espi_set_notifier(AspeedESPIState *s,
                              const AspeedESPINotifier *notifier,
                              void *opaque)
{
    s->notifier = notifier;
    s->notifier_opaque = opaque;
}

/* The shared eSPI Status word, as a controller would read it. */
uint32_t aspeed_espi_get_status(AspeedESPIState *s)
{
    s->status_shadow = s->status;
    return s->status;
}

bool aspeed_espi_alert_pending(AspeedESPIState *s, uint32_t *status)
{
    if (status) {
        *status = s->status;
    }
    return s->status != s->status_shadow;
}

uint32_t aspeed_espi_peek_status(AspeedESPIState *s)
{
    return s->status;
}

uint32_t aspeed_espi_vw_get_event(AspeedESPIState *s, bool evt1)
{
    return s->regs[TO_REG(evt1 ? ESPI_CH1_EVT1 : ESPI_CH1_EVT0)];
}

/* A length field of zero encodes the maximum payload, not an empty one. */
static uint32_t aspeed_espi_cyc_len(uint32_t ctrl)
{
    uint32_t len = (ctrl & ESPI_CYC_LEN) >> 12;

    return len ? len : ASPEED_ESPI_MAX_PLD;
}

static uint32_t aspeed_espi_flash_window_size(AspeedESPIState *s)
{
    return (uint32_t)(~s->flash.mask + 1);
}

/*
 * Is this erase size one the target told the controller it supports? A size
 * outside the advertisement is answered with an unsuccessful completion
 * rather than erasing something else.
 */
static bool aspeed_espi_erase_advertised(AspeedESPIState *s, uint32_t code)
{
    uint32_t bitmap = (s->regs[TO_REG(ESPI_CAP_CH3_1)] >>
                       ESPI_ERASE_BITMAP_SHIFT) & 0xff;
    uint32_t kb;

    if (code >= ARRAY_SIZE(espi_erase_size_kb)) {
        return false;
    }

    kb = espi_erase_size_kb[code];

    return !!(bitmap & BIT(ctz32(kb)));
}

/* Update what the flash channel contributes to the shared status word. */
static void aspeed_espi_flash_update_status(AspeedESPIState *s)
{
    AspeedESPIChannel *chan = &s->chan[ASPEED_ESPI_CHAN_FLASH];
    uint32_t status = 0;

    if (chan->sw_ready) {
        if (s->flash.np_pending < s->np_queue_depth &&
            (s->regs[TO_REG(ESPI_CH3_CTRL)] & ESPI_CH3_CTRL_RX_RST)) {
            status |= ESPI_STATUS_FLASH_NP_FREE;
        }
        /*
         * A software-queued completion counts even with no payload: a
         * successful erase or write completes as ESPI_FLASH_SUC_CMPLT,
         * which carries zero data bytes.
         */
        if (s->flash.tx_pending || s->flash.tx_len) {
            status |= ESPI_STATUS_FLASH_C_AVAIL;
        }
    }

    chan->status = status;
    aspeed_espi_update_status(s);
}

/*
 * Hand a cycle to software: park the header in the receive control register,
 * stage the payload, and raise the completion interrupt. In DMA mode the
 * payload is pushed to the descriptor address instead of being read out
 * through the FIFO register.
 */
static void aspeed_espi_flash_post_rx(AspeedESPIState *s, uint8_t cyc,
                                      uint8_t tag, uint32_t addr, uint32_t len,
                                      const uint8_t *data)
{
    uint32_t ctrl = s->regs[TO_REG(ESPI_CH3_CTRL)];
    uint32_t enc_len = (len == ASPEED_ESPI_MAX_PLD) ? 0 : len;
    /*
     * A flash request puts its 4-byte address, big endian, at the head of the
     * receive buffer, ahead of any data; the length field counts the data
     * alone. Software reads the address from the head of the buffer, so
     * leaving it out makes the first four payload bytes be taken for it.
     * Completions posted
     * this way carry no address field.
     */
    bool has_addr = (cyc == ESPI_FLASH_READ || cyc == ESPI_FLASH_WRITE ||
                     cyc == ESPI_FLASH_ERASE);
    uint32_t hdr = has_addr ? 4 : 0;

    if (len > sizeof(s->flash.rx_buf) - hdr) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: %" PRIu32 "-byte cycle truncated to the %zu-byte "
                      "receive buffer\n",
                      __func__, len, sizeof(s->flash.rx_buf) - hdr);
        len = sizeof(s->flash.rx_buf) - hdr;
    }

    s->flash.rx_len = hdr + len;
    s->flash.rx_pos = 0;
    if (has_addr) {
        s->flash.rx_buf[0] = addr >> 24;
        s->flash.rx_buf[1] = addr >> 16;
        s->flash.rx_buf[2] = addr >> 8;
        s->flash.rx_buf[3] = addr;
    }
    if (len && data) {
        memcpy(s->flash.rx_buf + hdr, data, len);
    }

    s->regs[TO_REG(ESPI_CH3_RX_CTRL)] =
        ESPI_CYC_SERV_PEND |
        ((enc_len << 12) & ESPI_CYC_LEN) |
        ((tag << 8) & ESPI_CYC_TAG) |
        (cyc & ESPI_CYC_TYPE);

    if ((ctrl & ESPI_CH3_CTRL_RX_DMA_EN) && s->flash.rx_len) {
        uint64_t dma_addr =
            ((uint64_t)s->regs[TO_REG(ESPI_CH3_RX_DMAL + 4)] << 32) |
            s->regs[TO_REG(ESPI_CH3_RX_DMAL)];

        if (!s->dma_mr ||
            address_space_write(&s->dma_as, dma_addr - s->dma_base,
                                MEMTXATTRS_UNSPECIFIED,
                                s->flash.rx_buf, s->flash.rx_len) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: receive DMA to 0x%" PRIx64 " failed\n",
                          __func__, dma_addr);
        }
    }

    s->regs[TO_REG(ESPI_CH3_BASE + ESPI_CHAN_INT_STS)] |=
        ESPI_CH3_INT_STS_RX_CMPLT;

    trace_aspeed_espi_flash_cycle("to software", cyc, tag, len);
    aspeed_espi_update_irq(s);
}

/*
 * Turn a flash offset from the controller into the address the target drives
 * on its own bus: the masked bits come from the mapping target, the rest from
 * the incoming address. The flash channel carries the same target and mask
 * pair as the peripheral channel, but no source address to check against.
 *
 * An offset past the window wraps back into it rather than failing, so an
 * image larger than the window reads back as a corrupted version of itself.
 * That aliasing is faithful to hardware, hence a log rather than an error.
 */
static uint64_t aspeed_espi_flash_map(AspeedESPIState *s, uint32_t addr)
{
    uint64_t mapped = (addr & ~s->flash.mask) | (s->flash.taddr &
                                                 s->flash.mask);

    if (addr & s->flash.mask) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: flash offset 0x%" PRIx32 " lies outside the "
                      "mapped window of 0x%" PRIx32 " bytes and wraps to 0x%"
                      PRIx64 "\n", __func__, addr,
                      aspeed_espi_flash_window_size(s), mapped);
    }

    trace_aspeed_espi_flash_map(addr, mapped);
    return mapped;
}

/* AHB address of a hardware-serviced cycle */
static bool aspeed_espi_flash_hw_target(AspeedESPIState *s, uint32_t addr,
                                        uint64_t *ahb)
{
    if (!s->tafsb) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: no TAFS/SAFS bridge attached\n", __func__);
        return false;
    }

    *ahb = aspeed_espi_flash_map(s, addr);

    /*
     * The mapping target and the bridge's flash window are two registers in
     * different blocks naming the same thing. Nothing keeps them alike, so a
     * mismatch is reported rather than silently favouring one.
     */
    if (aspeed_tafsb_mem_base(s->tafsb) != (s->flash.taddr &
                                             s->flash.mask)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: the mapping target 0x%" PRIx64 " and the TAFS/SAFS "
                      "bridge's flash window 0x%" PRIx64 " disagree\n",
                      __func__, s->flash.taddr & s->flash.mask,
                      aspeed_tafsb_mem_base(s->tafsb));
    }
    return true;
}

/* Service a read in hardware, without software ever seeing it */
static MemTxResult aspeed_espi_flash_hw_read(AspeedESPIState *s,
                                             uint32_t addr, uint32_t len,
                                             uint8_t *buf)
{
    uint64_t ahb;

    if (!aspeed_espi_flash_hw_target(s, addr, &ahb)) {
        return MEMTX_ERROR;
    }
    return aspeed_tafsb_ahb_read(s->tafsb, ahb, buf, len);
}

/* Hardware mode write or erase */
static MemTxResult aspeed_espi_flash_hw_update(AspeedESPIState *s,
                                               uint8_t cyc, uint32_t addr,
                                               uint32_t len,
                                               const uint8_t *data)
{
    uint64_t ahb;

    if (!aspeed_espi_flash_hw_target(s, addr, &ahb)) {
        return MEMTX_ERROR;
    }
    if (cyc == ESPI_FLASH_WRITE) {
        return aspeed_tafsb_program(s->tafsb, ahb, data, len);
    }
    /* For an erase, len is the size code */
    return aspeed_tafsb_erase(s->tafsb, ahb,
                              espi_erase_size_kb[len] * KiB);
}

/*
 * Accept a flash cycle from the controller.
 *
 * Reads are answered by hardware unless the channel is in software mode.
 * Writes and erases are answered by hardware only in hardware mode; mixed
 * mode passes them to software, which is the whole point of the mode.
 */
bool aspeed_espi_flash_cycle(AspeedESPIState *s, uint8_t cyc, uint8_t tag,
                             uint32_t len, uint32_t addr,
                             const uint8_t *data, uint8_t *rsp_cyc,
                             uint8_t *rsp_data, uint32_t *rsp_len)
{
    uint32_t rsp_cap = rsp_len ? *rsp_len : 0;
    bool hw_serviced;

    if (rsp_cyc) {
        *rsp_cyc = 0;
    }
    if (rsp_len) {
        *rsp_len = 0;
    }

    if (!s->chan[ASPEED_ESPI_CHAN_FLASH].sw_ready) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: cycle 0x%02x while the flash channel is not "
                      "ready\n", __func__, cyc);
        return false;
    }

    if (s->flash.np_pending >= s->np_queue_depth) {
        trace_aspeed_espi_flash_queue_full(s->flash.np_pending);
        return false;
    }

    switch (cyc) {
    case ESPI_FLASH_READ:
        hw_serviced = aspeed_espi_flash_mode(s) != ESPI_EDAF_MODE_SW;
        break;
    case ESPI_FLASH_WRITE:
    case ESPI_FLASH_ERASE:
        hw_serviced = aspeed_espi_flash_mode(s) == ESPI_EDAF_MODE_HW;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unsupported flash cycle type 0x%02x\n",
                      __func__, cyc);
        return false;
    }

    /* The Rx queue takes nothing while its reset is held low */
    if (!hw_serviced &&
        !(s->regs[TO_REG(ESPI_CH3_CTRL)] & ESPI_CH3_CTRL_RX_RST)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: cycle 0x%02x for software while the receive "
                      "queue is held in reset\n", __func__, cyc);
        return false;
    }

    /*
     * A read or an erase states how much it wants rather than how much it
     * carries, so the caller cannot bound @len from the message size and
     * this is the only place that can. A cycle larger than the buffers hold
     * is answered unsuccessfully rather than truncated: a short completion
     * would hand back an image the controller believes is whole.
     */
    if (cyc != ESPI_FLASH_ERASE && len > ASPEED_ESPI_MAX_PLD) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: cycle of %" PRIu32 " bytes exceeds the %u-byte "
                      "maximum payload; answering with an unsuccessful "
                      "completion\n", __func__, len, ASPEED_ESPI_MAX_PLD);
        trace_aspeed_espi_flash_cpl(tag, ESPI_FLASH_UNSUC_CMPLT_ONLY, 0);
        aspeed_espi_flash_post_rx(s, ESPI_FLASH_UNSUC_CMPLT_ONLY, tag, 0, 0,
                                  NULL);
        if (rsp_cyc) {
            *rsp_cyc = ESPI_FLASH_UNSUC_CMPLT_ONLY;
        }
        return true;
    }

    if (cyc == ESPI_FLASH_ERASE &&
        !aspeed_espi_erase_advertised(s, len)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: erase size code %u was never advertised; "
                      "answering with an unsuccessful completion\n",
                      __func__, len);
        trace_aspeed_espi_flash_cpl(tag, ESPI_FLASH_UNSUC_CMPLT_ONLY, 0);
        aspeed_espi_flash_post_rx(s, ESPI_FLASH_UNSUC_CMPLT_ONLY, tag, 0, 0,
                                  NULL);
        if (rsp_cyc) {
            *rsp_cyc = ESPI_FLASH_UNSUC_CMPLT_ONLY;
        }
        return true;
    }

    /* SAFS hardware reset (active low) held */
    if (hw_serviced &&
        !(s->regs[TO_REG(ESPI_CH3_CTRL)] & ESPI_CH3_CTRL_SAFS_HW_RST)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: cycle 0x%02x for hardware while its reset is "
                      "held; answering with an unsuccessful completion\n",
                      __func__, cyc);
        trace_aspeed_espi_flash_cpl(tag, ESPI_FLASH_UNSUC_CMPLT_ONLY, 0);
        if (rsp_cyc) {
            *rsp_cyc = ESPI_FLASH_UNSUC_CMPLT_ONLY;
        }
        return true;
    }

    s->flash.np_pending++;
    s->in_ctrl_request = true;
    trace_aspeed_espi_flash_cycle(hw_serviced ? "to hardware" : "queued",
                                  cyc, tag, len);

    if (!hw_serviced) {
        aspeed_espi_flash_post_rx(s, cyc, tag, addr, len, data);
        aspeed_espi_flash_update_status(s);
        aspeed_espi_end_ctrl_request(s);
        return true;
    }

    /*
     * Hardware servicing is invisible to software: no receive descriptor is
     * posted and no interrupt is raised.
     */
    if (cyc == ESPI_FLASH_READ) {
        g_autofree uint8_t *buf = g_malloc0(len);
        MemTxResult res = aspeed_espi_flash_hw_read(s, addr, len, buf);
        uint8_t cpl = (res == MEMTX_OK) ?
            ESPI_FLASH_SUC_CMPLT_D_ONLY : ESPI_FLASH_UNSUC_CMPLT_ONLY;

        s->flash.tx_len = (res == MEMTX_OK) ? len : 0;
        if (res == MEMTX_OK) {
            memcpy(s->flash.tx_buf, buf, len);
        }
        trace_aspeed_espi_flash_cpl(tag, cpl, s->flash.tx_len);

        if (rsp_cyc) {
            *rsp_cyc = cpl;
        }
        if (rsp_data && rsp_len && s->flash.tx_len) {
            uint32_t n = MIN(s->flash.tx_len, rsp_cap);

            if (n < s->flash.tx_len) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "%s: completion of %" PRIu32 " bytes truncated "
                              "to the caller's %" PRIu32 "-byte buffer\n",
                              __func__, s->flash.tx_len, rsp_cap);
            }
            memcpy(rsp_data, s->flash.tx_buf, n);
            *rsp_len = n;
        }
    } else {
        MemTxResult res = aspeed_espi_flash_hw_update(s, cyc, addr, len, data);
        uint8_t cpl = (res == MEMTX_OK) ?
            ESPI_FLASH_SUC_CMPLT : ESPI_FLASH_UNSUC_CMPLT_ONLY;

        trace_aspeed_espi_flash_cpl(tag, cpl, 0);
        if (rsp_cyc) {
            *rsp_cyc = cpl;
        }
    }

    s->flash.np_pending--;
    aspeed_espi_flash_update_status(s);
    aspeed_espi_end_ctrl_request(s);

    return true;
}

/*
 * Drop the completion state a hardware-serviced read leaves behind.
 *
 * The completion itself came back in aspeed_espi_flash_cycle()'s rsp_data and
 * rsp_len, but tx_len stays set so that response's own status word shows
 * FLASH_C_AVAIL. aspeed_espi_flash_update_status() treats any nonzero tx_len
 * as a completion being available, and a plain read has no equivalent of the
 * software-serviced path's tx_pending flag to tell the two apart, so left
 * uncleared it would report FLASH_C_AVAIL forever.
 *
 * This is separate from aspeed_espi_flash_cycle() because the caller peeks
 * status to fill the read's own response after that call returns; clearing
 * tx_len any earlier would blank the answer that response has to carry.
 */
void aspeed_espi_flash_read_done(AspeedESPIState *s)
{
    s->flash.tx_len = 0;
    s->flash.tx_pos = 0;
    /*
     * A synchronous, already-delivered completion dropping back out of the
     * status word is not news to raise Alert over -- the controller that
     * just read it is, by construction, the same one this cleanup is
     * running on behalf of. aspeed_espi_flash_cycle() protected its own
     * status recompute with in_ctrl_request for exactly this reason; this
     * one runs after that function already cleared the flag (see the
     * comment above), so it has to re-arm it around its own recompute.
     */
    s->in_ctrl_request = true;
    aspeed_espi_flash_update_status(s);
    aspeed_espi_end_ctrl_request(s);
}

/*
 * Software has finished with the cycle in the receive registers. Drop it and
 * make room for the next one.
 */
static void aspeed_espi_flash_rx_serviced(AspeedESPIState *s)
{
    s->regs[TO_REG(ESPI_CH3_RX_CTRL)] &= ~ESPI_CYC_SERV_PEND;
    s->flash.rx_len = 0;
    s->flash.rx_pos = 0;

    if (s->flash.np_pending) {
        s->flash.np_pending--;
    }

    trace_aspeed_espi_flash_rx_serviced(s->flash.np_pending);
    aspeed_espi_flash_update_status(s);
}

/*
 * Software is handing back a completion through the transmit registers. The
 * trigger bit stays set until the controller collects the completion; a
 * trigger that cannot be taken is dropped.
 */
static bool aspeed_espi_flash_tx_trigger(AspeedESPIState *s, uint32_t ctrl)
{
    uint32_t cfg = s->regs[TO_REG(ESPI_CAP_CH3_0)];
    uint32_t len = aspeed_espi_cyc_len(ctrl);
    uint8_t cyc = ctrl & ESPI_CYC_TYPE;
    uint8_t tag = (ctrl & ESPI_CYC_TAG) >> 8;

    /* Transmitting on a channel the controller has not enabled is an error */
    if (!(cfg & ESPI_CFG_CHAN_ENABLE)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: completion queued while the flash channel is "
                      "disabled\n", __func__);
        s->regs[TO_REG(ESPI_CH3_BASE + ESPI_CHAN_INT_STS)] |=
            ESPI_CH3_INT_STS_TX_ERR;
        aspeed_espi_update_irq(s);
        return false;
    }

    /* The Tx queue sends nothing while its reset is held low */
    if (!(s->regs[TO_REG(ESPI_CH3_CTRL)] & ESPI_CH3_CTRL_TX_RST)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: completion queued while the transmit queue is "
                      "held in reset\n", __func__);
        return false;
    }

    switch (cyc) {
    case ESPI_FLASH_SUC_CMPLT:
    case ESPI_FLASH_UNSUC_CMPLT_LAST:
    case ESPI_FLASH_UNSUC_CMPLT_ONLY:
        len = 0;
        break;
    case ESPI_FLASH_SUC_CMPLT_D_ONLY:
    case ESPI_FLASH_SUC_CMPLT_D_FIRST:
    case ESPI_FLASH_SUC_CMPLT_D_MIDDLE:
    case ESPI_FLASH_SUC_CMPLT_D_LAST:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: software queued cycle type 0x%02x, which is not a "
                      "completion\n", __func__, cyc);
        return false;
    }

    if ((s->regs[TO_REG(ESPI_CH3_CTRL)] & ESPI_CH3_CTRL_TX_DMA_EN) && len) {
        uint64_t addr =
            ((uint64_t)s->regs[TO_REG(ESPI_CH3_TX_DMAL + 4)] << 32) |
            s->regs[TO_REG(ESPI_CH3_TX_DMAL)];

        if (!s->dma_mr ||
            address_space_read(&s->dma_as, addr - s->dma_base,
                               MEMTXATTRS_UNSPECIFIED,
                               s->flash.tx_buf, len) != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: transmit DMA from 0x%" PRIx64 " failed\n",
                          __func__, addr);
            return false;
        }
        s->flash.tx_len = len;
    }

    trace_aspeed_espi_flash_cpl(tag, cyc, len);

    /* The completion is ready for the controller to collect. */
    s->flash.tx_cyc     = cyc;
    s->flash.tx_tag     = tag;
    s->flash.tx_pending = true;

    aspeed_espi_flash_update_status(s);
    return true;
}

/*
 * Collect the completion software queued for a cycle it serviced
 * asynchronously. The transmit slot holds one completion, so a controller
 * that gets false polls again rather than expecting a second one.
 */
bool aspeed_espi_flash_get_completion(AspeedESPIState *s, uint8_t *rsp_cyc,
                                      uint8_t *rsp_tag, uint8_t *rsp_data,
                                      uint32_t *rsp_len)
{
    uint32_t rsp_cap = rsp_len ? *rsp_len : 0;

    if (rsp_len) {
        *rsp_len = 0;
    }

    if (!s->flash.tx_pending) {
        return false;
    }

    if (rsp_cyc) {
        *rsp_cyc = s->flash.tx_cyc;
    }
    if (rsp_tag) {
        *rsp_tag = s->flash.tx_tag;
    }
    if (rsp_data && rsp_len && s->flash.tx_len) {
        uint32_t n = MIN(s->flash.tx_len, rsp_cap);

        if (n < s->flash.tx_len) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: completion of %" PRIu32 " bytes truncated to "
                          "the caller's %" PRIu32 "-byte buffer\n",
                          __func__, s->flash.tx_len, rsp_cap);
        }
        memcpy(rsp_data, s->flash.tx_buf, n);
        *rsp_len = n;
    }

    s->flash.tx_pending = false;
    s->flash.tx_len = 0;
    s->flash.tx_pos = 0;
    s->regs[TO_REG(ESPI_CH3_TX_CTRL)] &= ~ESPI_CYC_TRIG_PEND;
    s->regs[TO_REG(ESPI_CH3_BASE + ESPI_CHAN_INT_STS)] |=
        ESPI_CH3_INT_STS_TX_CMPLT;
    aspeed_espi_flash_update_status(s);
    aspeed_espi_update_irq(s);

    return true;
}

/* Track software arming a channel, so channel code never rereads the bit. */
static void aspeed_espi_chan_ctrl_write(AspeedESPIState *s, int chan,
                                        uint32_t val)
{
    const AspeedESPIChannelInfo *info = &aspeed_espi_chan_info[chan];
    hwaddr reg = info->base + ESPI_CHAN_CTRL;
    uint32_t old = s->regs[TO_REG(reg)];
    bool ready = !!(val & info->sw_rdy);

    if ((val ^ old) & info->ctrl_ro) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to the read-only bits 0x%" PRIx32 " of the "
                      "%s channel control register\n", __func__,
                      info->ctrl_ro, info->name);
    }
    val = (val & ~info->ctrl_ro) | (old & info->ctrl_ro);
    s->regs[TO_REG(reg)] = val;

    if (ready != s->chan[chan].sw_ready) {
        s->chan[chan].sw_ready = ready;
        trace_aspeed_espi_chan_ready(info->name, ready);
        aspeed_espi_config_update_ready(s, chan);
        aspeed_espi_update_status(s);
    }
}

static uint64_t aspeed_espi_read(void *opaque, hwaddr offset, unsigned size)
{
    AspeedESPIState *s = ASPEED_ESPI(opaque);
    uint32_t val;

    if (!aspeed_espi_offset_implemented(offset)) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: read of unimplemented offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }

    /* The current status word and the one the controller last read */
    if (offset == ESPI_DEV_STS) {
        val = (s->status << 16) | (s->status_shadow & 0xffff);
        trace_aspeed_espi_read(offset, size, val);
        return val;
    }

    /*
     * The receive data register is a FIFO port, not storage: each read takes
     * the next byte of the staged payload.
     */
    if (offset == ESPI_CH3_RX_DATA) {
        if (s->flash.rx_pos >= s->flash.rx_len) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: read past the end of the staged flash "
                          "payload\n", __func__);
            return 0;
        }
        val = s->flash.rx_buf[s->flash.rx_pos++];
        trace_aspeed_espi_read(offset, size, val);
        return val;
    }

    val = s->regs[TO_REG(offset)];
    trace_aspeed_espi_read(offset, size, val);

    return val;
}

static void aspeed_espi_write(void *opaque, hwaddr offset, uint64_t data,
                              unsigned size)
{
    AspeedESPIState *s = ASPEED_ESPI(opaque);
    uint32_t val = (uint32_t)data;
    int chan;

    trace_aspeed_espi_write(offset, size, val);

    if (!aspeed_espi_offset_implemented(offset)) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: write 0x%08x to unimplemented offset 0x%"
                      HWADDR_PRIx "\n", __func__, val, offset);
        return;
    }

    if (aspeed_espi_offset_ro(offset)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write 0x%08x to read-only offset 0x%" HWADDR_PRIx
                      "\n", __func__, val, offset);
        return;
    }

    chan = aspeed_espi_offset_chan(offset);
    if (chan >= 0) {
        hwaddr chan_offset = offset - aspeed_espi_chan_info[chan].base;

        switch (chan_offset) {
        case ESPI_CHAN_CTRL: {
            uint32_t old = s->regs[TO_REG(offset)];

            aspeed_espi_chan_ctrl_write(s, chan, val);
            if (chan == ASPEED_ESPI_CHAN_FLASH) {
                aspeed_espi_flash_ctrl_write(s, old);
            }
            return;
        }
        case ESPI_CHAN_STS:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: write 0x%08x to read-only %s channel status\n",
                          __func__, val, aspeed_espi_chan_info[chan].name);
            return;
        case ESPI_CHAN_INT_STS:
            if (chan == ASPEED_ESPI_CHAN_VW) {
                uint32_t swap = ESPI_CH1_INT_GPIO | ESPI_CH1_INT_SYS_EVT1;

                val = (val & ~swap) |
                      ((val & ESPI_CH1_INT_SYS_EVT1) ? ESPI_CH1_INT_GPIO : 0) |
                      ((val & ESPI_CH1_INT_GPIO) ? ESPI_CH1_INT_SYS_EVT1 : 0);
            }
            s->regs[TO_REG(offset)] &= ~val;
            aspeed_espi_update_irq(s);
            return;
        case ESPI_CHAN_INT_EN:
            if (chan == ASPEED_ESPI_CHAN_VW) {
                val &= MAKE_64BIT_MASK(0, 4);
            } else if (chan == ASPEED_ESPI_CHAN_FLASH) {
                val &= MAKE_64BIT_MASK(0, 5);
            }
            s->regs[TO_REG(offset)] = val;
            aspeed_espi_update_irq(s);
            return;
        default:
            break;
        }
    }

    switch (offset) {
    case ESPI_CH1_EVT0:
        /* Software drives only its own wires; the rest are the controller's */
        val = (val & ESPI_CH1_EVT0_WMASK) |
              (s->regs[TO_REG(offset)] & ~ESPI_CH1_EVT0_WMASK);
        aspeed_espi_vw_event(s, ESPI_CH1_EVT0, val);
        return;

    case ESPI_CH1_EVT1:
        val = (val & ESPI_CH1_EVT1_WMASK) |
              (s->regs[TO_REG(offset)] & ~ESPI_CH1_EVT1_WMASK);
        aspeed_espi_vw_event(s, ESPI_CH1_EVT1, val);
        return;

    case ESPI_CH1_EVT0 + ESPI_EVT_INT_EN:
    case ESPI_CH1_EVT1 + ESPI_EVT_INT_EN: {
        hwaddr evt_reg = offset - ESPI_EVT_INT_EN;

        s->regs[TO_REG(offset)] = val & (evt_reg == ESPI_CH1_EVT0 ?
                                         ESPI_CH1_EVT0_INT_EN_MASK :
                                         ESPI_CH1_EVT1_INT_EN_MASK);
        aspeed_espi_vw_check_levels(s, evt_reg);
        return;
    }

    case ESPI_CH1_EVT0 + ESPI_EVT_INT_T0 ... ESPI_CH1_EVT0 + ESPI_EVT_INT_T2:
    case ESPI_CH1_EVT1 + ESPI_EVT_INT_T0 ... ESPI_CH1_EVT1 + ESPI_EVT_INT_T2:
        s->regs[TO_REG(offset)] = val;
        aspeed_espi_vw_check_levels(s, offset < ESPI_CH1_EVT1 ?
                                    ESPI_CH1_EVT0 : ESPI_CH1_EVT1);
        return;

    case ESPI_CH1_EVT0 + ESPI_EVT_INT_STS:
    case ESPI_CH1_EVT1 + ESPI_EVT_INT_STS:
        s->regs[TO_REG(offset)] &= ~val;
        aspeed_espi_vw_check_levels(s, offset - ESPI_EVT_INT_STS);
        return;

    case ESPI_FLTR_FIRST ... ESPI_FLTR_LAST:
        /*
         * The regions are stored and traced. Filtering itself is not
         * modelled: how regions combine, whether an end address is inside its
         * region and how a blocked cycle is answered are left out, so arming
         * a region logs that it has no effect.
         */
        if (val != s->regs[TO_REG(offset)]) {
            bool is_start = !((offset - ESPI_FLTR_FIRST) & 4);
            unsigned region = (offset - ESPI_FLTR_FIRST) / 8;

            if (is_start) {
                trace_aspeed_espi_filter_region(
                    region, (uint64_t)(val & ESPI_FLTR_ADDR),
                    !!(val & ESPI_FLTR_VALID), !!(val & ESPI_FLTR_READ),
                    !!(val & ESPI_FLTR_WRITE));
            }
            qemu_log_mask(LOG_UNIMP,
                          "%s: SAFS filter region %u %s address set to 0x%08x,"
                          " but filter evaluation is not modelled: no cycle "
                          "will be permitted or denied by it\n",
                          __func__, region, is_start ? "start" : "end", val);
        }
        s->regs[TO_REG(offset)] = val;
        return;

    case ESPI_FLTR_CTRL:
        s->regs[TO_REG(offset)] = val;
        trace_aspeed_espi_filter_ctrl(
            (val & ESPI_FLTR_CTRL_ALLOW) ? "allow" : "block",
            (uint64_t)((val & ESPI_FLTR_CTRL_SIZE) >> 16)
                << ESPI_FLTR_CTRL_SIZE_SHIFT);
        return;

    case ESPI_FLTR_INT:
        s->regs[TO_REG(offset)] = val;
        return;

    case ESPI_CH3_EDAF_TADDRL:
    case ESPI_CH3_EDAF_MASKL:
        val = (val & ESPI_CH3_EDAF_LO_WMASK) |
              (s->regs[TO_REG(offset)] & ~ESPI_CH3_EDAF_LO_WMASK);
        s->regs[TO_REG(offset)] = val;
        aspeed_espi_flash_update_window(s);
        return;
    case ESPI_CH3_EDAF_TADDRH:
    case ESPI_CH3_EDAF_MASKH:
        s->regs[TO_REG(offset)] = val;
        aspeed_espi_flash_update_window(s);
        return;

    case ESPI_CH3_RX_CTRL:
        /*
         * Software acknowledges the staged cycle by writing the pending bit
         * back; the rest of the register is hardware's to set.
         */
        if (val & ESPI_CYC_SERV_PEND) {
            aspeed_espi_flash_rx_serviced(s);
        }
        return;

    case ESPI_CH3_RX_DATA:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to the read-only flash receive FIFO\n",
                      __func__);
        return;

    case ESPI_CH3_TX_DATA:
        if (s->flash.tx_pos >= ASPEED_ESPI_MAX_PLD) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: flash transmit FIFO overflow\n", __func__);
            return;
        }
        s->flash.tx_buf[s->flash.tx_pos++] = val & 0xff;
        s->flash.tx_len = s->flash.tx_pos;
        return;

    case ESPI_CH3_TX_CTRL:
        if (s->regs[TO_REG(offset)] & ESPI_CYC_TRIG_PEND) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: flash transmit triggered while a completion "
                          "is still waiting to be collected\n", __func__);
            return;
        }
        s->regs[TO_REG(offset)] = val & ~ESPI_CYC_TRIG_PEND;
        if ((val & ESPI_CYC_TRIG_PEND) &&
            aspeed_espi_flash_tx_trigger(s, val)) {
            s->regs[TO_REG(offset)] |= ESPI_CYC_TRIG_PEND;
        }
        s->flash.tx_pos = 0;
        return;

    case ESPI_INT_STS:
        /*
         * Write one to clear, but only the reset events. The per-channel
         * summary bits are recomputed from the channels themselves.
         */
        s->regs[TO_REG(offset)] &= ~(val & ~ESPI_INT_STS_CHAN_MASK);
        aspeed_espi_update_irq(s);
        return;
    case ESPI_INT_EN:
        s->regs[TO_REG(offset)] = val & (ESPI_INT_EN_RST_ASSERT |
                                         ESPI_INT_EN_RST_DEASSERT);
        aspeed_espi_update_irq(s);
        return;
    case ESPI_CTRL:
        s->regs[TO_REG(offset)] = val & ESPI_CTRL_WMASK;
        return;
    case ESPI_STS: {
        /*
         * The abort counter is writable; the reset counter clears when 0xEA
         * is written to it.
         */
        uint32_t sts = s->regs[TO_REG(offset)];

        sts = deposit32(sts, 28, 4, extract32(val, 28, 4));
        if (extract32(val, 16, 8) == ESPI_STS_RESET_CNT_CLEAR) {
            sts &= ~ESPI_STS_RESET_CNT;
        }
        s->regs[TO_REG(offset)] = sts;
        return;
    }
    case ESPI_CAP_GEN:
        /* Only the RTC support bit is writable */
        s->regs[TO_REG(offset)] = (val & ESPI_CAP_GEN_RTC) |
            (s->regs[TO_REG(offset)] & ~ESPI_CAP_GEN_RTC);
        return;
    default:
        break;
    }

    s->regs[TO_REG(offset)] = val;
}

static const MemoryRegionOps aspeed_espi_ops = {
    .read = aspeed_espi_read,
    .write = aspeed_espi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* The registers are 32 bits wide */
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void aspeed_espi_reset_hold(Object *obj, ResetType type)
{
    AspeedESPIState *s = ASPEED_ESPI(obj);
    int i;

    memset(s->regs, 0, sizeof(s->regs));

    s->regs[TO_REG(ESPI_CTRL)] = ESPI_CTRL_RESET;
    s->regs[TO_REG(ESPI_STS)] = ESPI_STS_RESET;
    s->regs[TO_REG(ESPI_DEV_STS)] = ESPI_DEV_STS_RESET;

    s->regs[TO_REG(ESPI_DEV_ID)] = s->dev_id;
    s->regs[TO_REG(ESPI_CAP_GEN)] = s->cap_gen;
    s->regs[TO_REG(ESPI_CAP_CH0)] = s->cap_ch0;
    s->regs[TO_REG(ESPI_CAP_CH1)] = s->cap_ch1;
    s->regs[TO_REG(ESPI_CAP_CH2)] = s->cap_ch2;
    s->regs[TO_REG(ESPI_CAP_CH3_0)] = s->cap_ch3_0;
    s->regs[TO_REG(ESPI_CAP_CH3_1)] = s->cap_ch3_1;

    s->regs[TO_REG(ESPI_DBG_CMD)] = ESPI_DBG_CMD_RESET;
    s->regs[TO_REG(ESPI_DBG_RES)] = ESPI_DBG_RES_RESET;
    s->regs[TO_REG(ESPI_CH_ACC_CTRL)] = ESPI_CH_ACC_CTRL_RESET;

    for (i = 0; i < ASPEED_ESPI_NR_CHAN; i++) {
        const AspeedESPIChannelInfo *info = &aspeed_espi_chan_info[i];

        s->regs[TO_REG(info->base + ESPI_CHAN_CTRL)] = info->ctrl_reset;
        s->regs[TO_REG(info->base + ESPI_CHAN_STS)] = info->sts_reset;
    }

    s->regs[TO_REG(ESPI_CH1_EVT0)] = ESPI_CH1_EVT0_RESET;

    s->regs[TO_REG(ESPI_CH3_EDAF_TADDRH)] = ESPI_CH3_TADDRH_RESET;
    s->regs[TO_REG(ESPI_CH3_EDAF_MASKL)] = ESPI_CH3_MASKL_RESET;
    s->regs[TO_REG(ESPI_CH3_EDAF_MASKH)] = ESPI_CH3_MASKH_RESET;

    s->regs[TO_REG(ESPI_FLTR_CTRL)] = ESPI_FLTR_CTRL_RESET;

    for (i = 0; i < ESPI_FLTR_NR_REGIONS; i++) {
        hwaddr saddr = ESPI_FLTR_FIRST + (i * 8);

        s->regs[TO_REG(saddr)] = ESPI_FLTR_SADDR_RESET;
        s->regs[TO_REG(saddr + 4)] = ESPI_FLTR_EADDR_RESET;
    }

    memset(s->chan, 0, sizeof(s->chan));
    memset(&s->flash, 0, sizeof(s->flash));
    s->status = ESPI_STATUS_IDLE;
    s->status_shadow = ESPI_STATUS_IDLE;

    /* The mapping registers come up describing a window, so honour it. */
    aspeed_espi_flash_update_window(s);

    for (i = 0; i < ASPEED_ESPI_NR_CHAN; i++) {
        aspeed_espi_chan_refresh(s, i);
    }

    for (i = 0; i < ASPEED_ESPI_NR_IRQS; i++) {
        qemu_irq_lower(s->irq[i]);
    }

    trace_aspeed_espi_reset(type);
}

static void aspeed_espi_realize(DeviceState *dev, Error **errp)
{
    AspeedESPIState *s = ASPEED_ESPI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    memory_region_init_io(&s->mmio, OBJECT(s), &aspeed_espi_ops, s,
                          TYPE_ASPEED_ESPI, ASPEED_ESPI_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);

    for (i = 0; i < ASPEED_ESPI_NR_IRQS; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }

    qdev_init_gpio_in_named(dev, aspeed_espi_reset_line,
                            ASPEED_ESPI_RESET_GPIO, 1);

    if (s->dma_mr) {
        address_space_init(&s->dma_as, s->dma_mr, "aspeed-espi-dma");
    }
}

static const VMStateDescription vmstate_aspeed_espi_channel = {
    .name = "aspeed.espi/channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(sw_ready, AspeedESPIChannel),
        VMSTATE_UINT32(status, AspeedESPIChannel),
        VMSTATE_END_OF_LIST()
    }
};

static int aspeed_espi_post_load(void *opaque, int version_id)
{
    AspeedESPIState *s = opaque;

    /*
     * IRQ lines are not themselves migrated state -- they are derived from
     * the registers above, which are. Without this a pending interrupt the
     * migrated INT_STS status implies can go unraised until the next
     * unrelated event happens to recompute it.
     */
    aspeed_espi_update_irq(s);

    return 0;
}

static const VMStateDescription vmstate_aspeed_espi = {
    .name = TYPE_ASPEED_ESPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = aspeed_espi_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AspeedESPIState, ASPEED_ESPI_NR_REGS),
        VMSTATE_UINT32(status, AspeedESPIState),
        VMSTATE_UINT32(status_shadow, AspeedESPIState),
        VMSTATE_UINT64(flash.taddr, AspeedESPIState),
        VMSTATE_UINT64(flash.mask, AspeedESPIState),
        VMSTATE_UINT8_ARRAY(flash.rx_buf, AspeedESPIState,
                            ASPEED_ESPI_MAX_PLD),
        VMSTATE_UINT32(flash.rx_len, AspeedESPIState),
        VMSTATE_UINT32(flash.rx_pos, AspeedESPIState),
        VMSTATE_UINT8_ARRAY(flash.tx_buf, AspeedESPIState,
                            ASPEED_ESPI_MAX_PLD),
        VMSTATE_UINT32(flash.tx_len, AspeedESPIState),
        VMSTATE_UINT32(flash.tx_pos, AspeedESPIState),
        VMSTATE_UINT8(flash.tx_cyc, AspeedESPIState),
        VMSTATE_UINT8(flash.tx_tag, AspeedESPIState),
        VMSTATE_BOOL(flash.tx_pending, AspeedESPIState),
        VMSTATE_UINT32(flash.np_pending, AspeedESPIState),
        /*
         * Whether software has armed a channel, and each channel's share of
         * the status word. Neither can be rebuilt from the registers: the
         * shared word is migrated as a total, and its contributions are not
         * recoverable from the sum.
         */
        VMSTATE_STRUCT_ARRAY(chan, AspeedESPIState, ASPEED_ESPI_NR_CHAN, 1,
                             vmstate_aspeed_espi_channel, AspeedESPIChannel),
        VMSTATE_END_OF_LIST()
    },
};

static const Property aspeed_espi_properties[] = {
    DEFINE_PROP_UINT32("dev-id", AspeedESPIState, dev_id, ESPI_DEV_ID_RESET),
    DEFINE_PROP_UINT32("cap-gen", AspeedESPIState, cap_gen,
                       ESPI_CAP_GEN_RESET),
    DEFINE_PROP_UINT32("cap-ch0", AspeedESPIState, cap_ch0,
                       ESPI_CAP_CH0_RESET),
    DEFINE_PROP_UINT32("cap-ch1", AspeedESPIState, cap_ch1,
                       ESPI_CAP_CH1_RESET),
    DEFINE_PROP_UINT32("cap-ch2", AspeedESPIState, cap_ch2,
                       ESPI_CAP_CH2_RESET),
    DEFINE_PROP_UINT32("cap-ch3-0", AspeedESPIState, cap_ch3_0,
                       ESPI_CAP_CH3_0_RESET),
    DEFINE_PROP_UINT32("cap-ch3-1", AspeedESPIState, cap_ch3_1,
                       ESPI_CAP_CH3_1_RESET),
    /*
     * How many non-posted flash requests the target accepts before it stops
     * advertising queue space. A controller normally keeps one in flight, so
     * this mostly bounds a misbehaving one.
     */
    DEFINE_PROP_UINT32("np-queue-depth", AspeedESPIState, np_queue_depth, 4),
    DEFINE_PROP_UINT64("dma-base", AspeedESPIState, dma_base, 0),
    DEFINE_PROP_LINK("dma", AspeedESPIState, dma_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_LINK("tafsb", AspeedESPIState, tafsb,
                     TYPE_ASPEED_TAFSB, AspeedTAFSBState *),
};

static void aspeed_espi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = aspeed_espi_realize;
    dc->vmsd = &vmstate_aspeed_espi;
    dc->desc = "ASPEED eSPI Controller";
    rc->phases.hold = aspeed_espi_reset_hold;
    device_class_set_props(dc, aspeed_espi_properties);
}

static const TypeInfo aspeed_espi_types[] = {
    {
        .name          = TYPE_ASPEED_ESPI,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AspeedESPIState),
        .class_init    = aspeed_espi_class_init,
    },
};

DEFINE_TYPES(aspeed_espi_types)
