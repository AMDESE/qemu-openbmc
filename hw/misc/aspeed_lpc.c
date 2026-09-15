/*
 *  ASPEED LPC Controller
 *
 *  Copyright (C) 2017-2018 IBM Corp.
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/misc/aspeed_lpc.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TO_REG(offset) ((offset) >> 2)

#define HICR0                TO_REG(0x00)
#define   HICR0_LPC3E        BIT(7)
#define   HICR0_LPC2E        BIT(6)
#define   HICR0_LPC1E        BIT(5)
#define   HICR0_PMEE         BIT(2)
#define HICR1                TO_REG(0x04)
#define   HICR1_PMEB         BIT(2)
#define HICR2                TO_REG(0x08)
#define   HICR2_IBFIE3       BIT(3)
#define   HICR2_IBFIE2       BIT(2)
#define   HICR2_IBFIE1       BIT(1)
#define HICR3                TO_REG(0x0C)
#define HICR4                TO_REG(0x10)
#define   HICR4_KCSENBL      BIT(2)
#define   HICR4_LADR12AS     BIT(7)
#define LADR3H               TO_REG(0x14)
#define LADR3L               TO_REG(0x18)
#define LADR12H              TO_REG(0x1C)
#define LADR12L              TO_REG(0x20)
#define IDR1                 TO_REG(0x24)
#define IDR2                 TO_REG(0x28)
#define IDR3                 TO_REG(0x2C)
#define ODR1                 TO_REG(0x30)
#define ODR2                 TO_REG(0x34)
#define ODR3                 TO_REG(0x38)
#define STR1                 TO_REG(0x3C)
#define   STR_OBF            BIT(0)
#define   STR_IBF            BIT(1)
#define   STR_CMD_DATA       BIT(3)
#define STR2                 TO_REG(0x40)
#define STR3                 TO_REG(0x44)
#define HICR5                TO_REG(0x80)
#define HICR6                TO_REG(0x84)
#define HICR7                TO_REG(0x88)
#define HICR8                TO_REG(0x8C)
#define HICRB                TO_REG(0x100)
#define   HICRB_EN16LADR1    BIT(4)
#define   HICRB_EN16LADR2    BIT(5)
#define   HICRB_IBFIE4       BIT(1)
#define   HICRB_LPC4E        BIT(0)
#define LADR4                TO_REG(0x110)
#define IDR4                 TO_REG(0x114)
#define ODR4                 TO_REG(0x118)
#define STR4                 TO_REG(0x11C)
#define LSADR12              TO_REG(0x120)
#define   LSADR12_LSADR1_MASK  0x0000FFFF
#define   LSADR12_LSADR2_SHIFT 16
#define   LSADR12_LSADR2_MASK  0xFFFF0000

#define PCCR6                TO_REG(0x0C4)
#define   PCCR6_ADDR_MASK    0x0FFFFFFF
#define PCCR4                TO_REG(0x0D0)
#define   PCCR4_ADDR_MASK     0xFFFFFFF0
#define PCCR5                TO_REG(0x0D4)
#define   PCCR5_VERSION       0x10000000
#define   PCCR5_RW_MASK       0x0FFFFFFF
#define   PCCR5_LEN_MASK      0x00FFFFFF
#define   PCCR5_ADDR_HI_SHIFT 24
#define   PCCR5_ADDR_HI_MASK  0x0F
#define PCCR0                TO_REG(0x130)
#define   PCCR0_EN            BIT(0)
#define   PCCR0_EN_RXAVA_INT  BIT(1)
#define   PCCR0_EN_RXOVR_INT  BIT(3)
#define   PCCR0_TRIG_SHIFT    8
#define   PCCR0_TRIG_MASK     0x00000700
#define   PCCR0_RESET         0x60000000
#define   PCCR0_MODE_SEL_SHIFT 4
#define   PCCR0_MODE_SEL_MASK 0x00000030
#define   PCCR0_CLR_RX_FIFO   BIT(7)
#define   PCCR0_ADDR_SEL_SHIFT 12
#define   PCCR0_ADDR_SEL_MASK 0x00003000
#define   PCCR0_EN_DMA_MODE   BIT(14)
#define   PCCR0_EN_DMA_INT    BIT(31)
#define PCCR1                TO_REG(0x134)
#define   PCCR1_BASE_MASK     0x0000FFFF
#define   PCCR1_DC_SHIFT      16
#define   PCCR1_DC_MASK       0x003F0000
#define PCCR2                TO_REG(0x138)
#define   PCCR2_WPTR_SHIFT    24
#define   PCCR2_PATB_INT      BIT(16)
#define   PCCR2_PATA_INT      BIT(8)
#define   PCCR2_FIFO_FULL     BIT(6)
#define   PCCR2_COUNT_BIT8    BIT(5)
#define   PCCR2_DATA_RDY      BIT(4)
#define   PCCR2_RXOVR         BIT(3)
#define   PCCR2_RXAVA         BIT(1)
#define PCCR3                TO_REG(0x13C)
#define   PCCR3_DATA_MASK     0xFF
#define   PCCR3_WPTR_SHIFT    24
#define   PCCR3_RPTR_SHIFT    16
#define   PCCR3_COUNT_SHIFT   8

/* Bytes the capture block appends to the ring for one snooped byte. */
enum aspeed_lpc_pcc_record_mode {
    pcc_record_1b = 0,
    pcc_record_2b,
    pcc_record_4b,
    pcc_record_full,
};

/* Which of the snooped port's address bits the tag byte carries. */
enum aspeed_lpc_pcc_addr_sel {
    pcc_addr_sel_none = 0,
    pcc_addr_sel_45,
    pcc_addr_sel_67,
    pcc_addr_sel_89,
};

/* The write flag in the second byte of a two-byte record */
#define PCC_REC_WRITE        BIT(6)

enum aspeed_kcs_channel_id {
    kcs_channel_1 = 0,
    kcs_channel_2,
    kcs_channel_3,
    kcs_channel_4,
};

static const enum aspeed_lpc_subdevice aspeed_kcs_subdevice_map[] = {
    [kcs_channel_1] = aspeed_lpc_kcs_1,
    [kcs_channel_2] = aspeed_lpc_kcs_2,
    [kcs_channel_3] = aspeed_lpc_kcs_3,
    [kcs_channel_4] = aspeed_lpc_kcs_4,
};

struct aspeed_kcs_channel {
    enum aspeed_kcs_channel_id id;

    int idr;
    int odr;
    int str;
};

static const struct aspeed_kcs_channel aspeed_kcs_channel_map[] = {
    [kcs_channel_1] = {
        .id = kcs_channel_1,
        .idr = IDR1,
        .odr = ODR1,
        .str = STR1
    },

    [kcs_channel_2] = {
        .id = kcs_channel_2,
        .idr = IDR2,
        .odr = ODR2,
        .str = STR2
    },

    [kcs_channel_3] = {
        .id = kcs_channel_3,
        .idr = IDR3,
        .odr = ODR3,
        .str = STR3
    },

    [kcs_channel_4] = {
        .id = kcs_channel_4,
        .idr = IDR4,
        .odr = ODR4,
        .str = STR4
    },
};

struct aspeed_kcs_register_data {
    const char *name;
    int reg;
    const struct aspeed_kcs_channel *chan;
};

static const struct aspeed_kcs_register_data aspeed_kcs_registers[] = {
    {
        .name = "idr1",
        .reg = IDR1,
        .chan = &aspeed_kcs_channel_map[kcs_channel_1],
    },
    {
        .name = "odr1",
        .reg = ODR1,
        .chan = &aspeed_kcs_channel_map[kcs_channel_1],
    },
    {
        .name = "str1",
        .reg = STR1,
        .chan = &aspeed_kcs_channel_map[kcs_channel_1],
    },
    {
        .name = "idr2",
        .reg = IDR2,
        .chan = &aspeed_kcs_channel_map[kcs_channel_2],
    },
    {
        .name = "odr2",
        .reg = ODR2,
        .chan = &aspeed_kcs_channel_map[kcs_channel_2],
    },
    {
        .name = "str2",
        .reg = STR2,
        .chan = &aspeed_kcs_channel_map[kcs_channel_2],
    },
    {
        .name = "idr3",
        .reg = IDR3,
        .chan = &aspeed_kcs_channel_map[kcs_channel_3],
    },
    {
        .name = "odr3",
        .reg = ODR3,
        .chan = &aspeed_kcs_channel_map[kcs_channel_3],
    },
    {
        .name = "str3",
        .reg = STR3,
        .chan = &aspeed_kcs_channel_map[kcs_channel_3],
    },
    {
        .name = "idr4",
        .reg = IDR4,
        .chan = &aspeed_kcs_channel_map[kcs_channel_4],
    },
    {
        .name = "odr4",
        .reg = ODR4,
        .chan = &aspeed_kcs_channel_map[kcs_channel_4],
    },
    {
        .name = "str4",
        .reg = STR4,
        .chan = &aspeed_kcs_channel_map[kcs_channel_4],
    },
    { },
};

static const struct aspeed_kcs_register_data *
aspeed_kcs_get_register_data_by_name(const char *name)
{
    const struct aspeed_kcs_register_data *pos = aspeed_kcs_registers;

    while (pos->name) {
        if (!strcmp(pos->name, name)) {
            return pos;
        }
        pos++;
    }

    return NULL;
}

static const struct aspeed_kcs_channel *
aspeed_kcs_get_channel_by_register(int reg)
{
    const struct aspeed_kcs_register_data *pos = aspeed_kcs_registers;

    while (pos->name) {
        if (pos->reg == reg) {
            return pos->chan;
        }
        pos++;
    }

    return NULL;
}

static void aspeed_kcs_get_register_property(Object *obj,
                                             Visitor *v,
                                             const char *name,
                                             void *opaque,
                                             Error **errp)
{
    const struct aspeed_kcs_register_data *data;
    AspeedLPCState *s = ASPEED_LPC(obj);
    uint32_t val;

    data = aspeed_kcs_get_register_data_by_name(name);
    if (!data) {
        return;
    }

    if (!strncmp("odr", name, 3)) {
        s->regs[data->chan->str] &= ~STR_OBF;
    }

    val = s->regs[data->reg];

    visit_type_uint32(v, name, &val, errp);
}

static bool aspeed_kcs_channel_enabled(AspeedLPCState *s,
                                       const struct aspeed_kcs_channel *channel)
{
    switch (channel->id) {
    case kcs_channel_1: return s->regs[HICR0] & HICR0_LPC1E;
    case kcs_channel_2: return s->regs[HICR0] & HICR0_LPC2E;
    case kcs_channel_3:
        return (s->regs[HICR0] & HICR0_LPC3E) &&
                    (s->regs[HICR4] & HICR4_KCSENBL);
    case kcs_channel_4: return s->regs[HICRB] & HICRB_LPC4E;
    default: return false;
    }
}

static bool
aspeed_kcs_channel_ibf_irq_enabled(AspeedLPCState *s,
                                   const struct aspeed_kcs_channel *channel)
{
    if (!aspeed_kcs_channel_enabled(s, channel)) {
            return false;
    }

    switch (channel->id) {
    case kcs_channel_1: return s->regs[HICR2] & HICR2_IBFIE1;
    case kcs_channel_2: return s->regs[HICR2] & HICR2_IBFIE2;
    case kcs_channel_3: return s->regs[HICR2] & HICR2_IBFIE3;
    case kcs_channel_4: return s->regs[HICRB] & HICRB_IBFIE4;
    default: return false;
    }
}

/*
 * Channels 1 and 2 each have a data port address behind LADR12H/L, which
 * HICR4_LADR12AS banks between them. The status port is the data port plus
 * four, or the LSADR12 address when the matching HICRB EN16LADR bit is set.
 * Channel 3 has its own LADR3H/L and no status register at all -- the odd
 * address is implicit. Channel 4 packs both ports into the single LADR4
 * register.
 */
static bool aspeed_kcs_channel_ports(AspeedLPCState *s,
                                     const struct aspeed_kcs_channel *channel,
                                     uint32_t *data_port,
                                     uint32_t *status_port)
{
    switch (channel->id) {
    case kcs_channel_1:
        *data_port = s->kcs_ladr[0];
        *status_port = (s->regs[HICRB] & HICRB_EN16LADR1) ?
            (s->regs[LSADR12] & LSADR12_LSADR1_MASK) : *data_port + 4;
        return true;
    case kcs_channel_2:
        *data_port = s->kcs_ladr[1];
        *status_port = (s->regs[HICRB] & HICRB_EN16LADR2) ?
            ((s->regs[LSADR12] & LSADR12_LSADR2_MASK) >> LSADR12_LSADR2_SHIFT) :
            *data_port + 4;
        return true;
    case kcs_channel_3:
        *data_port = (s->regs[LADR3H] << 8) | s->regs[LADR3L];
        *status_port = *data_port | 1;
        return true;
    case kcs_channel_4:
        *data_port = s->regs[LADR4] & 0xFFFF;
        *status_port = (s->regs[LADR4] >> 16) & 0xFFFF;
        return true;
    default:
        return false;
    }
}

static const struct aspeed_kcs_channel *
aspeed_kcs_decode_port(AspeedLPCState *s, uint32_t port, bool *is_status)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(aspeed_kcs_channel_map); i++) {
        const struct aspeed_kcs_channel *channel = &aspeed_kcs_channel_map[i];
        uint32_t data_port, status_port;

        if (!aspeed_kcs_channel_enabled(s, channel)) {
            continue;
        }
        if (!aspeed_kcs_channel_ports(s, channel, &data_port, &status_port)) {
            continue;
        }
        if (port == data_port) {
            *is_status = false;
            return channel;
        }
        if (port == status_port) {
            *is_status = true;
            return channel;
        }
    }

    return NULL;
}

static void aspeed_kcs_set_register_property(Object *obj,
                                             Visitor *v,
                                             const char *name,
                                             void *opaque,
                                             Error **errp)
{
    const struct aspeed_kcs_register_data *data;
    AspeedLPCState *s = ASPEED_LPC(obj);
    uint32_t val;

    data = aspeed_kcs_get_register_data_by_name(name);
    if (!data) {
        return;
    }

    if (!visit_type_uint32(v, name, &val, errp)) {
        return;
    }

    if (strncmp("str", name, 3)) {
        s->regs[data->reg] = val;
    }

    if (!strncmp("idr", name, 3)) {
        s->regs[data->chan->str] |= STR_IBF;
        if (aspeed_kcs_channel_ibf_irq_enabled(s, data->chan)) {
            enum aspeed_lpc_subdevice subdev;

            subdev = aspeed_kcs_subdevice_map[data->chan->id];
            qemu_irq_raise(s->subdevice_irqs[subdev]);
        }
    }
}

static void aspeed_lpc_set_irq(void *opaque, int irq, int level)
{
    AspeedLPCState *s = (AspeedLPCState *)opaque;

    if (level) {
        s->subdevice_irqs_pending |= BIT(irq);
    } else {
        s->subdevice_irqs_pending &= ~BIT(irq);
    }

    qemu_set_irq(s->irq, !!s->subdevice_irqs_pending);
}

static void aspeed_lpc_update_pme(AspeedLPCState *s)
{
    qemu_set_irq(s->pme, (s->regs[HICR0] & HICR0_PMEE) &&
                         (s->regs[HICR1] & HICR1_PMEB));
}

/* Bytes in the capture FIFO at which RxAva is raised */
static uint32_t aspeed_lpc_pcc_trigger(AspeedLPCState *s)
{
    uint32_t level = (s->regs[PCCR0] & PCCR0_TRIG_MASK) >> PCCR0_TRIG_SHIFT;

    return level ? level * ASPEED_LPC_PCC_FIFO_SIZE / 8 : 1;
}

static bool aspeed_lpc_pcc_rxava(AspeedLPCState *s)
{
    return s->pcc_fifo_len >= aspeed_lpc_pcc_trigger(s);
}

/*
 * In DMA mode the capture interrupt is DMA done; in FIFO mode it is data
 * available at the trigger level, or an overrun.
 */
static void aspeed_lpc_pcc_update_irq(AspeedLPCState *s)
{
    uint32_t ctrl = s->regs[PCCR0];
    bool pending;

    if (ctrl & PCCR0_EN_DMA_MODE) {
        pending = (ctrl & PCCR0_EN_DMA_INT) &&
                  (s->regs[PCCR2] & PCCR2_DATA_RDY);
    } else {
        pending = ((ctrl & PCCR0_EN_RXAVA_INT) && aspeed_lpc_pcc_rxava(s)) ||
                  ((ctrl & PCCR0_EN_RXOVR_INT) &&
                   (s->regs[PCCR2] & PCCR2_RXOVR));
    }

    qemu_set_irq(s->subdevice_irqs[aspeed_lpc_pcc], pending);
}

static uint32_t aspeed_lpc_pcc_fifo_wptr(AspeedLPCState *s)
{
    return (s->pcc_fifo_rptr + s->pcc_fifo_len) % ASPEED_LPC_PCC_FIFO_SIZE;
}

/* PCCR2 with the fields that follow the FIFO filled in */
static uint32_t aspeed_lpc_pcc_read_status(AspeedLPCState *s)
{
    uint32_t val = s->regs[PCCR2] & (PCCR2_PATB_INT | PCCR2_PATA_INT |
                                     PCCR2_RXOVR);

    if (s->regs[PCCR0] & PCCR0_EN_DMA_MODE) {
        return val | (s->regs[PCCR2] & PCCR2_DATA_RDY);
    }

    val |= aspeed_lpc_pcc_fifo_wptr(s) << PCCR2_WPTR_SHIFT;
    if (s->pcc_fifo_len == ASPEED_LPC_PCC_FIFO_SIZE) {
        val |= PCCR2_FIFO_FULL;
    }
    if (s->pcc_fifo_len & 0x100) {
        val |= PCCR2_COUNT_BIT8;
    }
    if (s->pcc_fifo_len) {
        val |= PCCR2_DATA_RDY;
    }
    if (aspeed_lpc_pcc_rxava(s)) {
        val |= PCCR2_RXAVA;
    }

    return val;
}

/* Take the next byte from the capture FIFO, with its pointers and count */
static uint32_t aspeed_lpc_pcc_read_data(AspeedLPCState *s)
{
    uint32_t val = (aspeed_lpc_pcc_fifo_wptr(s) << PCCR3_WPTR_SHIFT) |
                   (s->pcc_fifo_rptr << PCCR3_RPTR_SHIFT) |
                   ((s->pcc_fifo_len & 0xff) << PCCR3_COUNT_SHIFT);

    if (s->pcc_fifo_len) {
        val |= s->pcc_fifo[s->pcc_fifo_rptr];
        s->pcc_fifo_rptr = (s->pcc_fifo_rptr + 1) % ASPEED_LPC_PCC_FIFO_SIZE;
        s->pcc_fifo_len--;
        aspeed_lpc_pcc_update_irq(s);
    }

    return val;
}

/*
 * subdevice_irqs[] and irq are qemu_irq objects, not migrated state, so a
 * restored register file leaves the wires at whatever level they happened
 * to be on the destination. Re-derive each subdevice's line from the
 * registers that determine it, the same way the register write paths do.
 */
static int aspeed_lpc_post_load(void *opaque, int version_id)
{
    AspeedLPCState *s = ASPEED_LPC(opaque);
    size_t i;

    for (i = 0; i < ARRAY_SIZE(aspeed_kcs_channel_map); i++) {
        const struct aspeed_kcs_channel *chan = &aspeed_kcs_channel_map[i];
        enum aspeed_lpc_subdevice subdev = aspeed_kcs_subdevice_map[chan->id];
        bool ibf = (s->regs[chan->str] & STR_IBF) &&
                   aspeed_kcs_channel_ibf_irq_enabled(s, chan);

        qemu_set_irq(s->subdevice_irqs[subdev], ibf);
    }

    aspeed_lpc_pcc_update_irq(s);
    aspeed_lpc_update_pme(s);
    qemu_set_irq(s->irq, !!s->subdevice_irqs_pending);

    return 0;
}

/*
 * Lay out the record appended for one snooped byte.
 *
 * A port write wider than a byte arrives here one byte at a time. In
 * two-byte mode each byte is followed by a second one that says which port
 * it came from: the port's low nibble, two more of its address bits selected
 * by PCCR0, and the write flag. The abort flag above it is never set, every
 * captured cycle completes.
 */
static uint32_t aspeed_lpc_pcc_record(AspeedLPCState *s, uint32_t port,
                                      uint8_t byte, uint8_t *rec)
{
    uint32_t mode = (s->regs[PCCR0] & PCCR0_MODE_SEL_MASK) >>
                    PCCR0_MODE_SEL_SHIFT;
    uint32_t hbits = (s->regs[PCCR0] & PCCR0_ADDR_SEL_MASK) >>
                     PCCR0_ADDR_SEL_SHIFT;
    uint8_t tag;

    rec[0] = byte;

    if (mode != pcc_record_2b) {
        if (mode != pcc_record_1b) {
            qemu_log_mask(LOG_UNIMP,
                          "%s: record mode %u is not modelled, capturing "
                          "single bytes\n", __func__, mode);
        }
        return 1;
    }

    tag = PCC_REC_WRITE | (port & 0x0f);
    switch (hbits) {
    case pcc_addr_sel_45:
        tag |= port & 0x30;
        break;
    case pcc_addr_sel_67:
        tag |= (port & 0xc0) >> 2;
        break;
    case pcc_addr_sel_89:
        tag |= (port & 0x300) >> 4;
        break;
    default:
        break;
    }

    rec[1] = tag;
    return 2;
}

void aspeed_lpc_pcc_capture(AspeedLPCState *s, uint32_t port, uint8_t byte)
{
    uint32_t base = s->regs[PCCR1] & PCCR1_BASE_MASK;
    uint32_t dc = (s->regs[PCCR1] & PCCR1_DC_MASK) >> PCCR1_DC_SHIFT;
    uint8_t rec[2];
    uint32_t reclen;
    uint32_t i;

    if (!(s->regs[PCCR0] & PCCR0_EN)) {
        return;
    }

    if (((port ^ base) & ~dc) != 0) {
        return;
    }

    reclen = aspeed_lpc_pcc_record(s, port, byte, rec);

    if (s->regs[PCCR0] & PCCR0_EN_DMA_MODE) {
        uint64_t dma_base = s->regs[PCCR4] |
            ((uint64_t)((s->regs[PCCR5] >> PCCR5_ADDR_HI_SHIFT) &
                        PCCR5_ADDR_HI_MASK) << 32);
        uint32_t bufsz = (s->regs[PCCR5] & PCCR5_LEN_MASK) * 4;

        if (!s->dma_mr || !bufsz) {
            return;
        }

        /*
         * Software can shrink the ring by reprogramming PCCR5 without
         * clearing PCCR0_CLR_RX_FIFO first, leaving pcc_wptr pointing past
         * the new, smaller bufsz.
         */
        if (s->pcc_wptr >= bufsz) {
            s->pcc_wptr %= bufsz;
        }

        /* Written a byte at a time: a record can straddle the ring's end. */
        for (i = 0; i < reclen; i++) {
            if (address_space_write(&s->dma_as, dma_base + s->pcc_wptr -
                                    s->dma_base, MEMTXATTRS_UNSPECIFIED,
                                    &rec[i], 1) != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "%s: PCC capture DMA to 0x%" PRIx64 " failed\n",
                              __func__, dma_base + s->pcc_wptr);
                return;
            }

            s->pcc_wptr = (s->pcc_wptr + 1) % bufsz;
        }

        s->regs[PCCR6] = (dma_base + s->pcc_wptr) & PCCR6_ADDR_MASK;
    } else {
        for (i = 0; i < reclen; i++) {
            if (s->pcc_fifo_len >= ASPEED_LPC_PCC_FIFO_SIZE) {
                s->regs[PCCR2] |= PCCR2_RXOVR;
                break;
            }

            s->pcc_fifo[aspeed_lpc_pcc_fifo_wptr(s)] = rec[i];
            s->pcc_fifo_len++;
        }
        aspeed_lpc_pcc_update_irq(s);
        return;
    }

    s->regs[PCCR2] |= PCCR2_DATA_RDY;
    aspeed_lpc_pcc_update_irq(s);
}

bool aspeed_lpc_kcs_host_write(AspeedLPCState *s, uint32_t port, uint8_t byte)
{
    const struct aspeed_kcs_channel *channel;
    enum aspeed_lpc_subdevice subdev;
    bool is_status;

    channel = aspeed_kcs_decode_port(s, port, &is_status);
    if (!channel) {
        return false;
    }

    s->regs[channel->idr] = byte;
    s->regs[channel->str] |= STR_IBF;
    if (is_status) {
        s->regs[channel->str] |= STR_CMD_DATA;
    } else {
        s->regs[channel->str] &= ~STR_CMD_DATA;
    }

    if (aspeed_kcs_channel_ibf_irq_enabled(s, channel)) {
        subdev = aspeed_kcs_subdevice_map[channel->id];
        qemu_irq_raise(s->subdevice_irqs[subdev]);
    }

    trace_aspeed_lpc_kcs_host_write(channel->id, port, byte);
    return true;
}

bool aspeed_lpc_kcs_host_read(AspeedLPCState *s, uint32_t port, uint8_t *byte)
{
    const struct aspeed_kcs_channel *channel;
    bool is_status;

    channel = aspeed_kcs_decode_port(s, port, &is_status);
    if (!channel) {
        return false;
    }

    if (is_status) {
        *byte = s->regs[channel->str];
    } else {
        *byte = s->regs[channel->odr];
        s->regs[channel->str] &= ~STR_OBF;
    }

    trace_aspeed_lpc_kcs_host_read(channel->id, port, *byte);
    return true;
}

static uint64_t aspeed_lpc_read(void *opaque, hwaddr offset, unsigned size)
{
    AspeedLPCState *s = ASPEED_LPC(opaque);
    int reg = TO_REG(offset);

    if (reg >= ARRAY_SIZE(s->regs)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }

    switch (reg) {
    case IDR1:
    case IDR2:
    case IDR3:
    case IDR4:
    {
        const struct aspeed_kcs_channel *channel;

        channel = aspeed_kcs_get_channel_by_register(reg);
        if (s->regs[channel->str] & STR_IBF) {
            enum aspeed_lpc_subdevice subdev;

            subdev = aspeed_kcs_subdevice_map[channel->id];
            qemu_irq_lower(s->subdevice_irqs[subdev]);
        }

        s->regs[channel->str] &= ~STR_IBF;
        break;
    }
    case LADR12H:
        return (s->kcs_ladr[!!(s->regs[HICR4] & HICR4_LADR12AS)] >> 8) |
               (s->kcs_ladr[1] << 16);
    case LADR12L:
        return (s->kcs_ladr[!!(s->regs[HICR4] & HICR4_LADR12AS)] & 0xff) |
               (s->kcs_ladr[0] << 16);
    case PCCR2:
        return aspeed_lpc_pcc_read_status(s);
    case PCCR3:
        return aspeed_lpc_pcc_read_data(s);
    default:
        break;
    }

    return s->regs[reg];
}

static void aspeed_lpc_write(void *opaque, hwaddr offset, uint64_t data,
                             unsigned int size)
{
    AspeedLPCState *s = ASPEED_LPC(opaque);
    int reg = TO_REG(offset);

    if (reg >= ARRAY_SIZE(s->regs)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds write at offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return;
    }


    switch (reg) {
    case ODR1:
    case ODR2:
    case ODR3:
    case ODR4:
        s->regs[aspeed_kcs_get_channel_by_register(reg)->str] |= STR_OBF;
        s->regs[reg] = data;
        break;
    case HICR0:
    case HICR1:
        s->regs[reg] = data;
        aspeed_lpc_update_pme(s);
        break;
    case LADR12H:
    case LADR12L:
    {
        uint16_t *ladr = &s->kcs_ladr[!!(s->regs[HICR4] & HICR4_LADR12AS)];

        if (reg == LADR12H) {
            *ladr = (*ladr & 0x00ff) | ((data & 0xff) << 8);
        } else {
            *ladr = (*ladr & 0xff00) | (data & 0xff);
        }
        break;
    }
    case PCCR2:
    {
        /* DMA done is write-1-clear in DMA mode only */
        uint32_t w1c = PCCR2_PATB_INT | PCCR2_PATA_INT | PCCR2_RXOVR;

        if (s->regs[PCCR0] & PCCR0_EN_DMA_MODE) {
            w1c |= PCCR2_DATA_RDY;
        }
        s->regs[PCCR2] &= ~(data & w1c);
        aspeed_lpc_pcc_update_irq(s);
        break;
    }
    case PCCR3:
    case PCCR6:
        break;
    case PCCR4:
        s->regs[PCCR4] = data & PCCR4_ADDR_MASK;
        break;
    case PCCR5:
        s->regs[PCCR5] = (data & PCCR5_RW_MASK) | PCCR5_VERSION;
        break;
    case PCCR0:
        s->regs[PCCR0] = data & ~PCCR0_CLR_RX_FIFO;
        if (data & PCCR0_CLR_RX_FIFO) {
            s->pcc_fifo_rptr = 0;
            s->pcc_fifo_len = 0;
            s->pcc_wptr = 0;
        }
        aspeed_lpc_pcc_update_irq(s);
        break;
    default:
        s->regs[reg] = data;
        break;
    }
}

static const MemoryRegionOps aspeed_lpc_ops = {
    .read = aspeed_lpc_read,
    .write = aspeed_lpc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void aspeed_lpc_reset(DeviceState *dev)
{
    struct AspeedLPCState *s = ASPEED_LPC(dev);
    int i;

    s->subdevice_irqs_pending = 0;

    memset(s->regs, 0, sizeof(s->regs));

    s->regs[HICR7] = s->hicr7;

    s->pcc_wptr = 0;
    s->pcc_fifo_rptr = 0;
    s->pcc_fifo_len = 0;
    s->regs[PCCR0] = PCCR0_RESET;
    s->regs[PCCR5] = PCCR5_VERSION;

    s->kcs_ladr[0] = 0x60;
    s->kcs_ladr[1] = 0x62;

    /*
     * The registers above are cleared by the memset, but the qemu_irq
     * lines they had been driving are separate objects: a subdevice's
     * line left asserted from before reset would otherwise stay asserted
     * on the wire despite subdevice_irqs_pending now reading zero.
     */
    qemu_set_irq(s->irq, 0);
    qemu_set_irq(s->pme, 0);
    for (i = 0; i < ASPEED_LPC_NR_SUBDEVS; i++) {
        qemu_set_irq(s->subdevice_irqs[i], 0);
    }
}

static void aspeed_lpc_realize(DeviceState *dev, Error **errp)
{
    AspeedLPCState *s = ASPEED_LPC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->subdevice_irqs[aspeed_lpc_kcs_1]);
    sysbus_init_irq(sbd, &s->subdevice_irqs[aspeed_lpc_kcs_2]);
    sysbus_init_irq(sbd, &s->subdevice_irqs[aspeed_lpc_kcs_3]);
    sysbus_init_irq(sbd, &s->subdevice_irqs[aspeed_lpc_kcs_4]);
    sysbus_init_irq(sbd, &s->subdevice_irqs[aspeed_lpc_ibt]);
    sysbus_init_irq(sbd, &s->subdevice_irqs[aspeed_lpc_pcc]);

    memory_region_init_io(&s->iomem, OBJECT(s), &aspeed_lpc_ops, s,
            TYPE_ASPEED_LPC, 0x1000);

    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_in(dev, aspeed_lpc_set_irq, ASPEED_LPC_NR_SUBDEVS);
    qdev_init_gpio_out_named(dev, &s->pme, ASPEED_LPC_PME_GPIO, 1);

    if (s->dma_mr) {
        address_space_init(&s->dma_as, s->dma_mr, "aspeed-lpc-pcc-dma");
    }
}

static void aspeed_lpc_init(Object *obj)
{
    object_property_add(obj, "idr1", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "odr1", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "str1", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "idr2", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "odr2", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "str2", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "idr3", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "odr3", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "str3", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "idr4", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "odr4", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
    object_property_add(obj, "str4", "uint32", aspeed_kcs_get_register_property,
                        aspeed_kcs_set_register_property, NULL, NULL);
}

static bool aspeed_lpc_host_state_needed(void *opaque)
{
    return true;
}

static int aspeed_lpc_host_state_post_load(void *opaque, int version_id)
{
    AspeedLPCState *s = ASPEED_LPC(opaque);

    if (s->pcc_fifo_rptr >= ASPEED_LPC_PCC_FIFO_SIZE ||
        s->pcc_fifo_len > ASPEED_LPC_PCC_FIFO_SIZE) {
        return -EINVAL;
    }

    return 0;
}

static const VMStateDescription vmstate_aspeed_lpc_host_state = {
    .name = TYPE_ASPEED_LPC "/host-state",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = aspeed_lpc_host_state_needed,
    .post_load = aspeed_lpc_host_state_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(pcc_wptr, AspeedLPCState),
        VMSTATE_UINT8_ARRAY(pcc_fifo, AspeedLPCState, ASPEED_LPC_PCC_FIFO_SIZE),
        VMSTATE_UINT32(pcc_fifo_rptr, AspeedLPCState),
        VMSTATE_UINT32(pcc_fifo_len, AspeedLPCState),
        VMSTATE_UINT16_ARRAY(kcs_ladr, AspeedLPCState, 2),
        VMSTATE_END_OF_LIST(),
    }
};

/*
 * The host-side state, Post Code Capture and the banked KCS addresses, lives
 * in a subsection rather than the top-level version so that a stream from
 * before this device modelled it still loads: an older source never sends
 * it, and the destination keeps those fields at the reset values.
 */
static const VMStateDescription vmstate_aspeed_lpc = {
    .name = TYPE_ASPEED_LPC,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = aspeed_lpc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AspeedLPCState, ASPEED_LPC_NR_REGS),
        VMSTATE_UINT32(subdevice_irqs_pending, AspeedLPCState),
        VMSTATE_END_OF_LIST(),
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_aspeed_lpc_host_state,
        NULL,
    }
};

static const Property aspeed_lpc_properties[] = {
    DEFINE_PROP_UINT32("hicr7", AspeedLPCState, hicr7, 0),
    DEFINE_PROP_UINT64("dma-base", AspeedLPCState, dma_base, 0),
    DEFINE_PROP_LINK("dma", AspeedLPCState, dma_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static void aspeed_lpc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = aspeed_lpc_realize;
    device_class_set_legacy_reset(dc, aspeed_lpc_reset);
    dc->desc = "Aspeed LPC Controller",
    dc->vmsd = &vmstate_aspeed_lpc;
    device_class_set_props(dc, aspeed_lpc_properties);
}

static const TypeInfo aspeed_lpc_info = {
    .name = TYPE_ASPEED_LPC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AspeedLPCState),
    .class_init = aspeed_lpc_class_init,
    .instance_init = aspeed_lpc_init,
};

static void aspeed_lpc_register_types(void)
{
    type_register_static(&aspeed_lpc_info);
}

type_init(aspeed_lpc_register_types);
