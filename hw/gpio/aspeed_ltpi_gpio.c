/*
 * ASPEED AST2700 LTPI GPIO controller
 *
 * The controller carries 112 input and 112 output lines over an LTPI link.
 * Each pair of lines has a control register: the even line is the input,
 * which can raise an interrupt, and the odd line is the output.  Four
 * interrupt status registers mirror the status bit of 32 pairs each.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/registerfields.h"
#include "hw/gpio/aspeed_ltpi_gpio.h"
#include "migration/vmstate.h"
#include "trace.h"

REG32(CTRL, 0x000)
    FIELD(CTRL, OUT_DATA, 0, 1)
    FIELD(CTRL, DIR, 1, 1)
    FIELD(CTRL, IRQ_EN, 2, 1)
    FIELD(CTRL, IRQ_TYPE, 3, 3)
    FIELD(CTRL, RST_TOLERANCE, 6, 1)
    FIELD(CTRL, IRQ_STS, 12, 1)
    FIELD(CTRL, IN_DATA, 13, 1)
REG32(IRQ_STATUS, 0x200)

#define LTPI_GPIO_MMIO_SIZE     0x400
#define LTPI_GPIO_NR_IRQ_STATUS DIV_ROUND_UP(ASPEED_LTPI_GPIO_NR_PAIRS, 32)

/* IRQ_TYPE values; any value with bit 2 set selects both edges */
#define LTPI_GPIO_IRQ_FALLING     0
#define LTPI_GPIO_IRQ_RISING      1
#define LTPI_GPIO_IRQ_LEVEL_LOW   2
#define LTPI_GPIO_IRQ_LEVEL_HIGH  3
#define LTPI_GPIO_IRQ_BOTH_EDGES  4

/* IRQ_STS is write-1-to-clear, IN_DATA and the reserved bits read-only */
#define LTPI_GPIO_CTRL_RW_MASK                                   \
    (R_CTRL_OUT_DATA_MASK | R_CTRL_DIR_MASK | R_CTRL_IRQ_EN_MASK | \
     R_CTRL_IRQ_TYPE_MASK | R_CTRL_RST_TOLERANCE_MASK)

static void aspeed_ltpi_gpio_update_irq(AspeedLtpiGpioState *s)
{
    bool pending = false;
    int i;

    for (i = 0; i < ASPEED_LTPI_GPIO_NR_PAIRS; i++) {
        if (s->ctrl[i] & R_CTRL_IRQ_STS_MASK) {
            pending = true;
            break;
        }
    }
    qemu_set_irq(s->irq, pending);
}

/* A level trigger stays pending for as long as the input holds the level. */
static void aspeed_ltpi_gpio_check_level(AspeedLtpiGpioState *s, int pair)
{
    uint32_t ctrl = s->ctrl[pair];
    uint32_t type = FIELD_EX32(ctrl, CTRL, IRQ_TYPE);
    bool level = FIELD_EX32(ctrl, CTRL, IN_DATA);

    if (FIELD_EX32(ctrl, CTRL, IRQ_EN) &&
        ((type == LTPI_GPIO_IRQ_LEVEL_LOW && !level) ||
         (type == LTPI_GPIO_IRQ_LEVEL_HIGH && level))) {
        s->ctrl[pair] = FIELD_DP32(ctrl, CTRL, IRQ_STS, 1);
    }
}

static void aspeed_ltpi_gpio_set_input(void *opaque, int pair, int level)
{
    AspeedLtpiGpioState *s = ASPEED_LTPI_GPIO(opaque);
    uint32_t ctrl = s->ctrl[pair];
    uint32_t type = FIELD_EX32(ctrl, CTRL, IRQ_TYPE);
    bool prev = FIELD_EX32(ctrl, CTRL, IN_DATA);
    bool curr = level;
    bool edge = false;

    if (FIELD_EX32(ctrl, CTRL, IRQ_EN) && prev != curr) {
        if (type & LTPI_GPIO_IRQ_BOTH_EDGES) {
            edge = true;
        } else if (type == LTPI_GPIO_IRQ_FALLING) {
            edge = !curr;
        } else if (type == LTPI_GPIO_IRQ_RISING) {
            edge = curr;
        }
    }
    ctrl = FIELD_DP32(ctrl, CTRL, IN_DATA, curr);
    if (edge) {
        ctrl = FIELD_DP32(ctrl, CTRL, IRQ_STS, 1);
    }
    s->ctrl[pair] = ctrl;
    aspeed_ltpi_gpio_check_level(s, pair);

    trace_aspeed_ltpi_gpio_input(pair, prev, curr,
                                 FIELD_EX32(s->ctrl[pair], CTRL, IRQ_STS));
    aspeed_ltpi_gpio_update_irq(s);
}

static uint64_t aspeed_ltpi_gpio_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    AspeedLtpiGpioState *s = ASPEED_LTPI_GPIO(opaque);
    uint64_t val = 0;

    if (offset < A_IRQ_STATUS) {
        unsigned pair = offset >> 2;

        if (pair < ASPEED_LTPI_GPIO_NR_PAIRS) {
            val = s->ctrl[pair];
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: no control register at 0x%" HWADDR_PRIx "\n",
                          __func__, offset);
        }
    } else if (offset < A_IRQ_STATUS + LTPI_GPIO_NR_IRQ_STATUS * 4) {
        unsigned first = (offset - A_IRQ_STATUS) / 4 * 32;
        unsigned i;

        for (i = 0; i < 32 && first + i < ASPEED_LTPI_GPIO_NR_PAIRS; i++) {
            if (s->ctrl[first + i] & R_CTRL_IRQ_STS_MASK) {
                val |= 1u << i;
            }
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad offset 0x%" HWADDR_PRIx "\n", __func__, offset);
    }

    trace_aspeed_ltpi_gpio_read(offset, val);
    return val;
}

static void aspeed_ltpi_gpio_write(void *opaque, hwaddr offset, uint64_t data,
                                   unsigned size)
{
    AspeedLtpiGpioState *s = ASPEED_LTPI_GPIO(opaque);
    unsigned pair = offset >> 2;
    uint32_t val = data;
    uint32_t old;

    trace_aspeed_ltpi_gpio_write(offset, data);

    if (offset >= A_IRQ_STATUS || pair >= ASPEED_LTPI_GPIO_NR_PAIRS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to read-only or unimplemented offset 0x%"
                      HWADDR_PRIx "\n", __func__, offset);
        return;
    }

    old = s->ctrl[pair];
    s->ctrl[pair] = (old & ~LTPI_GPIO_CTRL_RW_MASK) |
                    (val & LTPI_GPIO_CTRL_RW_MASK);
    if (val & R_CTRL_IRQ_STS_MASK) {
        s->ctrl[pair] &= ~R_CTRL_IRQ_STS_MASK;
    }
    aspeed_ltpi_gpio_check_level(s, pair);

    /*
     * The odd line of a pair is always an output: the direction bit is kept
     * for software but does not gate the line.
     */
    if ((old ^ s->ctrl[pair]) & R_CTRL_OUT_DATA_MASK) {
        qemu_set_irq(s->out[pair], FIELD_EX32(s->ctrl[pair], CTRL, OUT_DATA));
    }
    aspeed_ltpi_gpio_update_irq(s);
}

static const MemoryRegionOps aspeed_ltpi_gpio_ops = {
    .read = aspeed_ltpi_gpio_read,
    .write = aspeed_ltpi_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void aspeed_ltpi_gpio_reset_enter(Object *obj, ResetType type)
{
    AspeedLtpiGpioState *s = ASPEED_LTPI_GPIO(obj);
    int i;

    /*
     * Inputs keep the level driven from outside.  After the power-on reset,
     * reset tolerant pairs also keep the output data and direction.
     */
    for (i = 0; i < ASPEED_LTPI_GPIO_NR_PAIRS; i++) {
        uint32_t keep = R_CTRL_IN_DATA_MASK;

        if (s->powered && (s->ctrl[i] & R_CTRL_RST_TOLERANCE_MASK)) {
            keep |= R_CTRL_OUT_DATA_MASK | R_CTRL_DIR_MASK;
        }
        s->ctrl[i] &= keep;
    }
    s->powered = true;
}

static void aspeed_ltpi_gpio_reset_hold(Object *obj, ResetType type)
{
    AspeedLtpiGpioState *s = ASPEED_LTPI_GPIO(obj);
    int i;

    for (i = 0; i < ASPEED_LTPI_GPIO_NR_PAIRS; i++) {
        qemu_set_irq(s->out[i], FIELD_EX32(s->ctrl[i], CTRL, OUT_DATA));
    }
    qemu_set_irq(s->irq, 0);
}

static void aspeed_ltpi_gpio_realize(DeviceState *dev, Error **errp)
{
    AspeedLtpiGpioState *s = ASPEED_LTPI_GPIO(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &aspeed_ltpi_gpio_ops, s,
                          TYPE_ASPEED_LTPI_GPIO, LTPI_GPIO_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(dev, aspeed_ltpi_gpio_set_input, "ltpi-gpio-in",
                            ASPEED_LTPI_GPIO_NR_PAIRS);
    qdev_init_gpio_out_named(dev, s->out, "ltpi-gpio-out",
                             ASPEED_LTPI_GPIO_NR_PAIRS);
}

static const VMStateDescription vmstate_aspeed_ltpi_gpio = {
    .name = TYPE_ASPEED_LTPI_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ctrl, AspeedLtpiGpioState,
                             ASPEED_LTPI_GPIO_NR_PAIRS),
        VMSTATE_BOOL(powered, AspeedLtpiGpioState),
        VMSTATE_END_OF_LIST()
    },
};

static void aspeed_ltpi_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "ASPEED AST2700 LTPI GPIO controller";
    dc->realize = aspeed_ltpi_gpio_realize;
    dc->vmsd = &vmstate_aspeed_ltpi_gpio;
    rc->phases.enter = aspeed_ltpi_gpio_reset_enter;
    rc->phases.hold = aspeed_ltpi_gpio_reset_hold;
}

static const TypeInfo aspeed_ltpi_gpio_types[] = {
    {
        .name = TYPE_ASPEED_LTPI_GPIO,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AspeedLtpiGpioState),
        .class_init = aspeed_ltpi_gpio_class_init,
    },
};

DEFINE_TYPES(aspeed_ltpi_gpio_types)
