/*
 * ASPEED AST2700 MIPI I3C HCI controller
 *
 * Adds to the MIPI I3C HCI base model the AST2700 register layout, the
 * INHOUSE (0xd00) and PHY (0xe00) register blocks, the AUTOCMD automatic
 * read, the ASPEED Extended Capabilities, and commands whose DEV_INDEX is
 * the target's dynamic address.
 *
 * The container device (TYPE_ASPEED_I3C_HCI) maps the 16 controllers at
 * consecutive 4 KiB offsets of a 64 KiB region.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/registerfields.h"
#include "hw/i3c/aspeed_i3c_hci.h"
#include "hw/i3c/i3c.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

/* Register layout of a controller's 4 KiB window */
#define AST2700_HCI_VERSION       0x110      /* HCI 1.1 */
#define AST2700_PIO_OFFSET        0x0d0
#define AST2700_DAT_OFFSET        0x100
#define AST2700_DAT_ENTRIES       127
#define AST2700_DCT_OFFSET        0x500
#define AST2700_EXT_CAPS_OFFSET   0xf00
#define INHOUSE_BASE_OFFSET       0xd00
#define PHY_BASE_OFFSET           0xe00

/*
 * HC_CAPABILITIES of the AST2700 without what the model does not
 * implement: HDR-DDR, CCC defining bytes and controller handoff.
 */
#define AST2700_HC_CAPABILITIES   (0x468 & ~(BIT(10) | BIT(6) | BIT(5)))

/*
 * Extended Capabilities.  Each capability starts with a header holding its
 * ID in bits [7:0] and its length in words, header included, in bits
 * [23:8].
 */
#define EXTCAP_HDR(id, len)         (((len) << 8) | (id))
#define EXTCAP_HARDWARE_ID          0x01
#define EXTCAP_MASTER_CONFIG        0x02
#define EXTCAP_VENDOR_ASPEED        0xc0
#define MIPI_VENDOR_ASPEED          0x000003f6
#define VENDOR_PRODUCT_ID_A1        0x00010000
#define MASTER_CONFIG_MASTER_ONLY   0x00000010

static const uint32_t aspeed_i3c_hci_extcaps[] = {
    EXTCAP_HDR(EXTCAP_HARDWARE_ID, 4),
    MIPI_VENDOR_ASPEED,             /* vendor MIPI ID */
    0x00000002,                     /* vendor version ID */
    VENDOR_PRODUCT_ID_A1,           /* vendor product ID */

    EXTCAP_HDR(EXTCAP_MASTER_CONFIG, 2),
    MASTER_CONFIG_MASTER_ONLY,

    EXTCAP_HDR(EXTCAP_VENDOR_ASPEED, 3),
    INHOUSE_BASE_OFFSET,
    PHY_BASE_OFFSET,
};

/* INHOUSE registers, relative to INHOUSE_BASE_OFFSET */
REG32(INHOUSE_DAA_INDEX0, 0x10)
#define INHOUSE_DAA_INDEX_NR_REGS     4
REG32(INHOUSE_AUTOCMD_0, 0x20)
    FIELD(INHOUSE_AUTOCMD_0, ENABLE, 0, 1)
    FIELD(INHOUSE_AUTOCMD_0, RNW, 1, 1)
    FIELD(INHOUSE_AUTOCMD_0, LEN, 8, 8)
    FIELD(INHOUSE_AUTOCMD_0, DA, 16, 7)
#define INHOUSE_AUTOCMD_NR_SLOTS      8
#define INHOUSE_AUTOCMD_SLOT_SIZE     8
REG32(INHOUSE_INTR_STATUS, 0xe0)
    FIELD(INHOUSE_INTR_STATUS, WRITE_DONE, 5, 1)
    FIELD(INHOUSE_INTR_STATUS, READ_DONE, 6, 1)
    FIELD(INHOUSE_INTR_STATUS, IBI_DONE, 7, 1)
REG32(INHOUSE_INTR_STATUS_EN, 0xe4)
REG32(INHOUSE_INTR_SIG_EN, 0xe8)
REG32(INHOUSE_INTR_FORCE, 0xec)
REG32(INHOUSE_INTR_SUM_STATUS, 0xf0)
    FIELD(INHOUSE_INTR_SUM_STATUS, PIO, 1, 1)
    FIELD(INHOUSE_INTR_SUM_STATUS, INHOUSE, 3, 1)
REG32(INHOUSE_INTR_RENEW, 0xf4)

/* Responses to AUTOCMD reads carry this transaction ID. */
#define AUTOCMD_TID                   0xf

static uint32_t aspeed_i3c_hci_intr_sum(AspeedI3CHCICtrl *c)
{
    uint32_t sum = 0;

    if (mipi_i3c_hci_pio_irq_pending(MIPI_I3C_HCI(c))) {
        sum = FIELD_DP32(sum, INHOUSE_INTR_SUM_STATUS, PIO, 1);
    }
    if (c->inhouse_regs[R_INHOUSE_INTR_STATUS] &
        c->inhouse_regs[R_INHOUSE_INTR_SIG_EN]) {
        sum = FIELD_DP32(sum, INHOUSE_INTR_SUM_STATUS, INHOUSE, 1);
    }
    return sum;
}

static uint64_t aspeed_i3c_hci_ext_read(MIPII3CHCIState *s, hwaddr offset,
                                        unsigned size)
{
    AspeedI3CHCICtrl *c = ASPEED_I3C_HCI_CTRL(s);
    uint32_t val = 0;

    if (offset >= INHOUSE_BASE_OFFSET && offset < PHY_BASE_OFFSET) {
        hwaddr ih_off = offset - INHOUSE_BASE_OFFSET;

        if (ih_off < sizeof(c->inhouse_regs)) {
            if (ih_off == A_INHOUSE_INTR_SUM_STATUS) {
                val = aspeed_i3c_hci_intr_sum(c);
            } else {
                val = c->inhouse_regs[ih_off >> 2];
            }
        }
    } else if (offset >= PHY_BASE_OFFSET &&
               offset < ASPEED_I3C_HCI_CTRL_SIZE) {
        hwaddr phy_off = offset - PHY_BASE_OFFSET;

        if (phy_off < sizeof(c->phy_regs)) {
            val = c->phy_regs[phy_off >> 2];
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: i3c%u invalid read at 0x%" HWADDR_PRIx "\n",
                      __func__, s->id, offset);
    }
    return val;
}

static void aspeed_i3c_hci_ext_write(MIPII3CHCIState *s, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    AspeedI3CHCICtrl *c = ASPEED_I3C_HCI_CTRL(s);
    uint32_t val = value;

    if (offset >= INHOUSE_BASE_OFFSET && offset < PHY_BASE_OFFSET) {
        hwaddr ih_off = offset - INHOUSE_BASE_OFFSET;

        if (ih_off >= sizeof(c->inhouse_regs)) {
            goto invalid;
        }
        switch (ih_off) {
        case A_INHOUSE_INTR_STATUS:
            c->inhouse_regs[R_INHOUSE_INTR_STATUS] &= ~val;
            break;
        case A_INHOUSE_INTR_FORCE:
            c->inhouse_regs[R_INHOUSE_INTR_STATUS] |= val;
            break;
        case A_INHOUSE_INTR_RENEW:
            /* The interrupt line follows the PIO status; nothing to do. */
            break;
        case A_INHOUSE_INTR_SUM_STATUS:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: i3c%u write to read-only INTR_SUM_STATUS\n",
                          __func__, s->id);
            break;
        default:
            c->inhouse_regs[ih_off >> 2] = val;
            break;
        }
        return;
    } else if (offset >= PHY_BASE_OFFSET &&
               offset < ASPEED_I3C_HCI_CTRL_SIZE) {
        hwaddr phy_off = offset - PHY_BASE_OFFSET;

        if (phy_off < sizeof(c->phy_regs)) {
            c->phy_regs[phy_off >> 2] = val;
        }
        return;
    }

invalid:
    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: i3c%u invalid write at 0x%" HWADDR_PRIx
                  " (value 0x%08x)\n", __func__, s->id, offset, val);
}

static void aspeed_i3c_hci_ext_reset(MIPII3CHCIState *s)
{
    AspeedI3CHCICtrl *c = ASPEED_I3C_HCI_CTRL(s);

    memset(c->inhouse_regs, 0, sizeof(c->inhouse_regs));
    memset(c->phy_regs, 0, sizeof(c->phy_regs));
}

static const uint32_t *aspeed_i3c_hci_get_extcaps(MIPII3CHCIState *s,
                                                  size_t *nwords)
{
    *nwords = ARRAY_SIZE(aspeed_i3c_hci_extcaps);
    return aspeed_i3c_hci_extcaps;
}

/*
 * Software allocates the DAT entry whose index is the device's dynamic
 * address and never fills in its address field: the command's DEV_INDEX
 * is the dynamic address.
 */
static uint8_t aspeed_i3c_hci_dat_to_da(MIPII3CHCIState *s, uint8_t dev_idx)
{
    return dev_idx;
}

static int aspeed_i3c_hci_da_to_dat(MIPII3CHCIState *s, uint8_t da)
{
    return da < MIPI_I3C_HCI_GET_CLASS(s)->dat_entries ? da : -1;
}

/*
 * The addresses to assign come from the DAA_INDEX0-3 bitmap, where bit N
 * stands for address N.  Software rewrites it before each ENTDAA, so it is
 * left as it is.
 */
static uint8_t aspeed_i3c_hci_entdaa_next_da(MIPII3CHCIState *s,
                                             uint8_t dev_idx_base,
                                             uint8_t iter)
{
    AspeedI3CHCICtrl *c = ASPEED_I3C_HCI_CTRL(s);
    uint32_t reg;

    for (reg = 0; reg < INHOUSE_DAA_INDEX_NR_REGS; reg++) {
        uint32_t mask = c->inhouse_regs[R_INHOUSE_DAA_INDEX0 + reg];

        while (mask) {
            int bit = ctz32(mask);

            if (iter == 0) {
                return reg * 32 + bit;
            }
            iter--;
            mask &= ~(1u << bit);
        }
    }
    return 0xff;
}

static void aspeed_i3c_hci_post_cmd(MIPII3CHCIState *s, bool ok, bool is_read)
{
    AspeedI3CHCICtrl *c = ASPEED_I3C_HCI_CTRL(s);

    if (is_read) {
        ARRAY_FIELD_DP32(c->inhouse_regs, INHOUSE_INTR_STATUS, READ_DONE, 1);
    } else {
        ARRAY_FIELD_DP32(c->inhouse_regs, INHOUSE_INTR_STATUS, WRITE_DONE, 1);
    }
}

/*
 * AUTOCMD read: read @len bytes from the device at @da and queue them with
 * a response of transaction ID AUTOCMD_TID.
 */
static void aspeed_i3c_hci_autocmd(MIPII3CHCIState *s, uint8_t da,
                                   uint32_t len)
{
    AspeedI3CHCICtrl *c = ASPEED_I3C_HCI_CTRL(s);
    uint8_t rx_buf[MIPI_I3C_HCI_TX_BUF_SIZE];
    uint32_t read_len = MIN(len, (uint32_t)sizeof(rx_buf));
    uint32_t got = 0;
    uint32_t resp;
    bool ok;

    if (i3c_start_transfer(s->bus, da, true) != 0) {
        ok = false;
    } else {
        ok = i3c_recv(s->bus, rx_buf, read_len, &got) == 0;
        i3c_end_transfer(s->bus);
    }

    if (!ok) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: i3c%u AUTOCMD read from 0x%02x of %u bytes failed\n",
                      __func__, s->id, da, read_len);
        got = 0;
    }

    resp = mipi_i3c_hci_resp(ok ? MIPI_I3C_HCI_RESP_SUCCESS :
                             MIPI_I3C_HCI_RESP_ERR_ADDR_HEADER,
                             AUTOCMD_TID, got);
    mipi_i3c_hci_complete_autocmd(s, rx_buf, got, resp);
    ARRAY_FIELD_DP32(c->inhouse_regs, INHOUSE_INTR_STATUS, READ_DONE, 1);
}

/* Run the AUTOCMD read of every enabled slot of the IBI's source. */
static void aspeed_i3c_hci_post_ibi(MIPII3CHCIState *s, uint8_t da)
{
    AspeedI3CHCICtrl *c = ASPEED_I3C_HCI_CTRL(s);
    uint32_t n;

    ARRAY_FIELD_DP32(c->inhouse_regs, INHOUSE_INTR_STATUS, IBI_DONE, 1);

    if (!da) {
        return;
    }

    for (n = 0; n < INHOUSE_AUTOCMD_NR_SLOTS; n++) {
        uint32_t w0 = c->inhouse_regs[R_INHOUSE_AUTOCMD_0 +
                                      n * INHOUSE_AUTOCMD_SLOT_SIZE / 4];
        uint32_t len = FIELD_EX32(w0, INHOUSE_AUTOCMD_0, LEN);

        if (!FIELD_EX32(w0, INHOUSE_AUTOCMD_0, ENABLE) ||
            FIELD_EX32(w0, INHOUSE_AUTOCMD_0, DA) != da ||
            !FIELD_EX32(w0, INHOUSE_AUTOCMD_0, RNW) || !len) {
            continue;
        }

        aspeed_i3c_hci_autocmd(s, da, len);
    }
}

I3CBus *aspeed_i3c_hci_get_bus(AspeedI3CHCICtrl *ctrl)
{
    return mipi_i3c_hci_get_bus(MIPI_I3C_HCI(ctrl));
}

bool aspeed_i3c_hci_inject_ibi(AspeedI3CHCICtrl *ctrl, uint8_t da,
                               uint8_t ibi_byte, bool has_byte)
{
    return mipi_i3c_hci_inject_ibi(MIPI_I3C_HCI(ctrl), da, ibi_byte,
                                   has_byte);
}

static const VMStateDescription vmstate_aspeed_i3c_hci_ctrl = {
    .name = TYPE_ASPEED_I3C_HCI_CTRL,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(parent_obj, AspeedI3CHCICtrl, 1,
                       vmstate_mipi_i3c_hci, MIPII3CHCIState),
        VMSTATE_UINT32_ARRAY(inhouse_regs, AspeedI3CHCICtrl,
                             ASPEED_I3C_HCI_INHOUSE_NR_REGS),
        VMSTATE_UINT32_ARRAY(phy_regs, AspeedI3CHCICtrl,
                             ASPEED_I3C_HCI_PHY_NR_REGS),
        VMSTATE_END_OF_LIST(),
    },
};

static void aspeed_i3c_hci_ctrl_class_init(ObjectClass *klass,
                                           const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    MIPII3CHCIClass *mc = MIPI_I3C_HCI_CLASS(klass);

    dc->desc = "ASPEED AST2700 I3C HCI controller";
    /* Part of the AST2700 controller block, not created on its own. */
    dc->user_creatable = false;
    dc->vmsd = &vmstate_aspeed_i3c_hci_ctrl;

    mc->mmio_size = ASPEED_I3C_HCI_CTRL_SIZE;
    mc->version = AST2700_HCI_VERSION;
    mc->capabilities = AST2700_HC_CAPABILITIES;
    mc->pio_offset = AST2700_PIO_OFFSET;
    mc->dat_offset = AST2700_DAT_OFFSET;
    mc->dat_entries = AST2700_DAT_ENTRIES;
    mc->dct_offset = AST2700_DCT_OFFSET;
    mc->ext_caps_offset = AST2700_EXT_CAPS_OFFSET;
    mc->get_extcaps = aspeed_i3c_hci_get_extcaps;
    mc->ext_read = aspeed_i3c_hci_ext_read;
    mc->ext_write = aspeed_i3c_hci_ext_write;
    mc->ext_reset = aspeed_i3c_hci_ext_reset;
    mc->dat_to_da = aspeed_i3c_hci_dat_to_da;
    mc->da_to_dat = aspeed_i3c_hci_da_to_dat;
    mc->entdaa_next_da = aspeed_i3c_hci_entdaa_next_da;
    mc->post_cmd = aspeed_i3c_hci_post_cmd;
    mc->post_ibi = aspeed_i3c_hci_post_ibi;
}

static void aspeed_i3c_hci_instance_init(Object *obj)
{
    AspeedI3CHCIState *s = ASPEED_I3C_HCI(obj);
    int i;

    for (i = 0; i < ASPEED_I3C_HCI_NR_CTRLS; i++) {
        object_initialize_child(obj, "ctrl[*]", &s->ctrls[i],
                                TYPE_ASPEED_I3C_HCI_CTRL);
    }
}

static void aspeed_i3c_hci_realize(DeviceState *dev, Error **errp)
{
    AspeedI3CHCIState *s = ASPEED_I3C_HCI(dev);
    int i;

    memory_region_init(&s->iomem, OBJECT(s), TYPE_ASPEED_I3C_HCI,
                       ASPEED_I3C_HCI_TOTAL_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    for (i = 0; i < ASPEED_I3C_HCI_NR_CTRLS; i++) {
        if (!object_property_set_uint(OBJECT(&s->ctrls[i]), "ctrl-id", i,
                                      errp) ||
            !sysbus_realize(SYS_BUS_DEVICE(&s->ctrls[i]), errp)) {
            return;
        }
        memory_region_add_subregion(&s->iomem, i * ASPEED_I3C_HCI_CTRL_SIZE,
                                    &MIPI_I3C_HCI(&s->ctrls[i])->mr);
    }
}

static const VMStateDescription vmstate_aspeed_i3c_hci = {
    .name = TYPE_ASPEED_I3C_HCI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ctrls, AspeedI3CHCIState,
                             ASPEED_I3C_HCI_NR_CTRLS, 1,
                             vmstate_aspeed_i3c_hci_ctrl, AspeedI3CHCICtrl),
        VMSTATE_END_OF_LIST(),
    },
};

static void aspeed_i3c_hci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "ASPEED AST2700 I3C HCI controllers";
    dc->realize = aspeed_i3c_hci_realize;
    dc->vmsd = &vmstate_aspeed_i3c_hci;
}

static const TypeInfo aspeed_i3c_hci_types[] = {
    {
        .name = TYPE_ASPEED_I3C_HCI_CTRL,
        .parent = TYPE_MIPI_I3C_HCI,
        .instance_size = sizeof(AspeedI3CHCICtrl),
        .class_init = aspeed_i3c_hci_ctrl_class_init,
    },
    {
        .name = TYPE_ASPEED_I3C_HCI,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AspeedI3CHCIState),
        .instance_init = aspeed_i3c_hci_instance_init,
        .class_init = aspeed_i3c_hci_class_init,
    },
};

DEFINE_TYPES(aspeed_i3c_hci_types)
