/*
 * ASPEED AST2700 TAFS/SAFS Global Controller
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Settings shared by the two TAFS/SAFS bridges: the memory mode erase fill.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/aspeed_tafsc.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TAFSC_ERASE_CTRL        0xc100
/* Bridge n erases with ones */
#define TAFSC_ERASE_ONES(n)     BIT(1 + 8 * (n))
#define TAFSC_ERASE_CTRL_MASK   (TAFSC_ERASE_ONES(0) | TAFSC_ERASE_ONES(1))

uint8_t aspeed_tafsc_erase_fill(AspeedTAFSCState *s, unsigned int bridge)
{
    assert(bridge < ASPEED_TAFSC_NR_BRIDGES);
    return (s->erase_ctrl & TAFSC_ERASE_ONES(bridge)) ? 0xff : 0x00;
}

static uint64_t aspeed_tafsc_read(void *opaque, hwaddr offset, unsigned size)
{
    AspeedTAFSCState *s = ASPEED_TAFSC(opaque);

    if (offset != TAFSC_ERASE_CTRL) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: read of unimplemented offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }
    trace_aspeed_tafsc_read(offset, s->erase_ctrl);
    return s->erase_ctrl;
}

static void aspeed_tafsc_write(void *opaque, hwaddr offset, uint64_t data,
                               unsigned size)
{
    AspeedTAFSCState *s = ASPEED_TAFSC(opaque);

    trace_aspeed_tafsc_write(offset, data);
    if (offset != TAFSC_ERASE_CTRL) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: write 0x%" PRIx64 " to unimplemented offset 0x%"
                      HWADDR_PRIx "\n", __func__, data, offset);
        return;
    }
    s->erase_ctrl = data & TAFSC_ERASE_CTRL_MASK;
}

static const MemoryRegionOps aspeed_tafsc_ops = {
    .read = aspeed_tafsc_read,
    .write = aspeed_tafsc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void aspeed_tafsc_reset_hold(Object *obj, ResetType type)
{
    AspeedTAFSCState *s = ASPEED_TAFSC(obj);

    s->erase_ctrl = 0;
}

static void aspeed_tafsc_realize(DeviceState *dev, Error **errp)
{
    AspeedTAFSCState *s = ASPEED_TAFSC(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &aspeed_tafsc_ops, s,
                          TYPE_ASPEED_TAFSC, ASPEED_TAFSC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
}

static const VMStateDescription vmstate_aspeed_tafsc = {
    .name = TYPE_ASPEED_TAFSC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(erase_ctrl, AspeedTAFSCState),
        VMSTATE_END_OF_LIST()
    }
};

static void aspeed_tafsc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = aspeed_tafsc_realize;
    dc->vmsd = &vmstate_aspeed_tafsc;
    dc->desc = "ASPEED TAFS/SAFS Global Controller";
    rc->phases.hold = aspeed_tafsc_reset_hold;
}

static const TypeInfo aspeed_tafsc_types[] = {
    {
        .name          = TYPE_ASPEED_TAFSC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AspeedTAFSCState),
        .class_init    = aspeed_tafsc_class_init,
    },
};

DEFINE_TYPES(aspeed_tafsc_types)
