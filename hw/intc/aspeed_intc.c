/*
 * ASPEED INTC Controller
 *
 * Copyright (C) 2024 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/intc/aspeed_intc.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "trace.h"
#include "hw/registerfields.h"
#include "qapi/error.h"

/* AST2700 INTC0 Registers */
REG32(INTC0_GICINT128_EN,         0x1000)
REG32(INTC0_GICINT128_STATUS,     0x1004)
REG32(INTC0_GICINT129_EN,         0x1100)
REG32(INTC0_GICINT129_STATUS,     0x1104)
REG32(INTC0_GICINT130_EN,         0x1200)
REG32(INTC0_GICINT130_STATUS,     0x1204)
REG32(INTC0_GICINT131_EN,         0x1300)
REG32(INTC0_GICINT131_STATUS,     0x1304)
REG32(INTC0_GICINT132_EN,         0x1400)
REG32(INTC0_GICINT132_STATUS,     0x1404)
REG32(INTC0_GICINT133_EN,         0x1500)
REG32(INTC0_GICINT133_STATUS,     0x1504)
REG32(INTC0_GICINT134_EN,         0x1600)
REG32(INTC0_GICINT134_STATUS,     0x1604)
REG32(INTC0_GICINT135_EN,         0x1700)
REG32(INTC0_GICINT135_STATUS,     0x1704)
REG32(INTC0_GICINT136_EN,         0x1800)
REG32(INTC0_GICINT136_STATUS,     0x1804)
REG32(INTC0_GICINT192_201_EN,         0x1B00)
REG32(INTC0_GICINT192_201_STATUS,     0x1B04)

/* AST2700 INTC1 Registers */
REG32(INTC1_GICINT192_EN,         0x100)
REG32(INTC1_GICINT192_STATUS,     0x104)
REG32(INTC1_GICINT193_EN,         0x110)
REG32(INTC1_GICINT193_STATUS,     0x114)
REG32(INTC1_GICINT194_EN,         0x120)
REG32(INTC1_GICINT194_STATUS,     0x124)
REG32(INTC1_GICINT195_EN,         0x130)
REG32(INTC1_GICINT195_STATUS,     0x134)
REG32(INTC1_GICINT196_EN,         0x140)
REG32(INTC1_GICINT196_STATUS,     0x144)
REG32(INTC1_GICINT197_EN,         0x150)
REG32(INTC1_GICINT197_STATUS,     0x154)

static AspeedINTCIRQ aspeed_2700_intc0_irqs[ASPEED_INTC_MAX_INPINS] = {
    {0, 0, 10, R_INTC0_GICINT192_201_EN, R_INTC0_GICINT192_201_STATUS},
    {1, 10, 1, R_INTC0_GICINT128_EN, R_INTC0_GICINT128_STATUS},
    {2, 11, 1, R_INTC0_GICINT129_EN, R_INTC0_GICINT129_STATUS},
    {3, 12, 1, R_INTC0_GICINT130_EN, R_INTC0_GICINT130_STATUS},
    {4, 13, 1, R_INTC0_GICINT131_EN, R_INTC0_GICINT131_STATUS},
    {5, 14, 1, R_INTC0_GICINT132_EN, R_INTC0_GICINT132_STATUS},
    {6, 15, 1, R_INTC0_GICINT133_EN, R_INTC0_GICINT133_STATUS},
    {7, 16, 1, R_INTC0_GICINT134_EN, R_INTC0_GICINT134_STATUS},
    {8, 17, 1, R_INTC0_GICINT135_EN, R_INTC0_GICINT135_STATUS},
    {9, 18, 1, R_INTC0_GICINT136_EN, R_INTC0_GICINT136_STATUS},
};

static AspeedINTCIRQ aspeed_2700_intc1_irqs[ASPEED_INTC_MAX_INPINS] = {
    {0, 0, 1, R_INTC1_GICINT192_EN, R_INTC1_GICINT192_STATUS},
    {1, 1, 1, R_INTC1_GICINT193_EN, R_INTC1_GICINT193_STATUS},
    {2, 2, 1, R_INTC1_GICINT194_EN, R_INTC1_GICINT194_STATUS},
    {3, 3, 1, R_INTC1_GICINT195_EN, R_INTC1_GICINT195_STATUS},
    {4, 4, 1, R_INTC1_GICINT196_EN, R_INTC1_GICINT196_STATUS},
    {5, 5, 1, R_INTC1_GICINT197_EN, R_INTC1_GICINT197_STATUS},
};

static const AspeedINTCIRQ *aspeed_intc_get_irq(AspeedINTCClass *aic,
                                                uint32_t addr)
{
    int i;

    for (i = 0; i < aic->irq_table_count; i++) {
        if (aic->irq_table[i].enable_addr == addr ||
            aic->irq_table[i].status_addr == addr) {
            return &aic->irq_table[i];
        }
    }

    /*
     * Invalid addr.
     */
    g_assert_not_reached();
}

/*
 * Update the state of an interrupt controller pin by setting
 * the specified output pin to the given level.
 * The input pin index should be between 0 and the number of input pins.
 * The output pin index should be between 0 and the number of output pins.
 */
static void aspeed_intc_update(AspeedINTCState *s, int inpin_idx,
                               int outpin_idx, int level)
{
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);

    if (inpin_idx >= aic->num_inpins) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Invalid input pin index: %d\n",
                      __func__, inpin_idx);
        return;
    }

    if (outpin_idx >= aic->num_outpins) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Invalid output pin index: %d\n",
                      __func__, outpin_idx);
        return;
    }

    trace_aspeed_intc_update_irq(aic->id, inpin_idx, outpin_idx, level);
    qemu_set_irq(s->output_pins[outpin_idx], level);
}

static void aspeed_intc_set_irq_handler(AspeedINTCState *s,
                            const AspeedINTCIRQ *irq, uint32_t select)
{
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);

    if (s->mask[irq->inpin_idx] || s->regs[irq->status_addr]) {
        /*
         * a. mask is not 0 means in ISR mode
         * sources interrupt routine are executing.
         * b. status register value is not 0 means previous
         * source interrupt does not be executed, yet.
         *
         * save source interrupt to pending variable.
         */
        s->pending[irq->inpin_idx] |= select;
        trace_aspeed_intc_pending_irq(aic->id, irq->inpin_idx,
                                      s->pending[irq->inpin_idx]);
    } else {
        /*
         * notify firmware which source interrupt are coming
         * by setting status register
         */
        s->regs[irq->status_addr] = select;
        trace_aspeed_intc_trigger_irq(aic->id, irq->inpin_idx,
                                      irq->outpin_idx,
                                      s->regs[irq->status_addr]);
        aspeed_intc_update(s, irq->inpin_idx, irq->outpin_idx, 1);
    }
}

static void aspeed_intc_set_irq_handler_multi_outpins(AspeedINTCState *s,
                                     const AspeedINTCIRQ *irq, uint32_t select)
{
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    int i;

    for (i = 0; i < irq->num_outpins; i++) {
        if (select & BIT(i)) {
            if (s->mask[irq->inpin_idx] & BIT(i) ||
                s->regs[irq->status_addr] & BIT(i)) {
                /*
                 * a. mask bit is not 0 means in ISR mode sources interrupt
                 * routine are executing.
                 * b. status bit is not 0 means previous source interrupt
                 * does not be executed, yet.
                 *
                 * save source interrupt to pending bit.
                 */
                 s->pending[irq->inpin_idx] |= BIT(i);
                 trace_aspeed_intc_pending_irq(aic->id, irq->inpin_idx,
                                               s->pending[irq->inpin_idx]);
            } else {
                /*
                 * notify firmware which source interrupt are coming
                 * by setting status bit
                 */
                s->regs[irq->status_addr] |= BIT(i);
                trace_aspeed_intc_trigger_irq(aic->id, irq->inpin_idx,
                                              irq->outpin_idx + i,
                                              s->regs[irq->status_addr]);
                aspeed_intc_update(s, irq->inpin_idx, irq->outpin_idx + i, 1);
            }
        }
    }
}

/*
 * GICINT192_201 maps 1:10 to input IRQ 0 and output IRQs 0 to 9.
 * GICINT128 to GICINT136 map 1:1 to input IRQs 1 to 9 and output
 * IRQs 10 to 18. The value of input IRQ should be between 0 and
 * the number of input pins.
 */
static void aspeed_intc_set_irq(void *opaque, int irq_idx, int level)
{
    AspeedINTCState *s = (AspeedINTCState *)opaque;
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    const AspeedINTCIRQ *irq;
    uint32_t select = 0;
    uint32_t enable;
    int i;

    if (irq_idx >= aic->num_inpins) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Invalid input interrupt number: %d\n",
                      __func__, irq_idx);
        return;
    }

    irq = &aic->irq_table[irq_idx];

    trace_aspeed_intc_set_irq(aic->id, irq->inpin_idx, level);
    enable = s->enable[irq->inpin_idx];

    if (!level) {
        return;
    }

    for (i = 0; i < aic->num_lines; i++) {
        if (s->orgates[irq->inpin_idx].levels[i]) {
            if (enable & BIT(i)) {
                select |= BIT(i);
            }
        }
    }

    if (!select) {
        return;
    }

    trace_aspeed_intc_select(aic->id, select);

    if (irq->num_outpins > 1) {
        aspeed_intc_set_irq_handler_multi_outpins(s, irq, select);
    } else {
        aspeed_intc_set_irq_handler(s, irq, select);
    }
}

static void aspeed_2700_intc_enable_handler(AspeedINTCState *s, uint32_t addr,
                                            uint64_t data)
{
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    const AspeedINTCIRQ *irq;
    uint32_t old_enable;
    uint32_t change;

    irq = aspeed_intc_get_irq(aic, addr);

    if (irq->inpin_idx >= aic->num_inpins) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Invalid input pin index number: %d\n",
                      __func__, irq->inpin_idx);
        return;
    }

    /*
     * The enable registers are used to enable source interrupts.
     * They also handle masking and unmasking of source interrupts
     * during the execution of the source ISR.
     */

    /* disable all source interrupt */
    if (!data && !s->enable[irq->inpin_idx]) {
        s->regs[addr] = data;
        return;
    }

    old_enable = s->enable[irq->inpin_idx];
    s->enable[irq->inpin_idx] |= data;

    /* enable new source interrupt */
    if (old_enable != s->enable[irq->inpin_idx]) {
        trace_aspeed_intc_enable(aic->id, s->enable[irq->inpin_idx]);
        s->regs[addr] = data;
        return;
    }

    /* mask and unmask source interrupt */
    change = s->regs[addr] ^ data;
    if (change & data) {
        s->mask[irq->inpin_idx] &= ~change;
        trace_aspeed_intc_unmask(aic->id, change, s->mask[irq->inpin_idx]);
    } else {
        s->mask[irq->inpin_idx] |= change;
        trace_aspeed_intc_mask(aic->id, change, s->mask[irq->inpin_idx]);
    }
    s->regs[addr] = data;
}

static void aspeed_2700_intc_status_handler(AspeedINTCState *s, uint32_t addr,
                                            uint64_t data)
{
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    const AspeedINTCIRQ *irq;

    if (!data) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Invalid data 0\n", __func__);
        return;
    }

    irq = aspeed_intc_get_irq(aic, addr);

    if (irq->inpin_idx >= aic->num_inpins) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Invalid input pin index number: %d\n",
                      __func__, irq->inpin_idx);
        return;
    }

    /* clear status */
    s->regs[addr] &= ~data;

    /*
     * The status registers are used for notify sources ISR are executed.
     * If one source ISR is executed, it will clear one bit.
     * If it clear all bits, it means to initialize this register status
     * rather than sources ISR are executed.
     */
    if (data == 0xffffffff) {
        return;
    }

    /* All source ISR execution are done */
    if (!s->regs[addr]) {
        trace_aspeed_intc_all_isr_done(aic->id, irq->inpin_idx);
        if (s->pending[irq->inpin_idx]) {
            /*
             * handle pending source interrupt
             * notify firmware which source interrupt are pending
             * by setting status register
             */
            s->regs[addr] = s->pending[irq->inpin_idx];
            s->pending[irq->inpin_idx] = 0;
            trace_aspeed_intc_trigger_irq(aic->id, irq->inpin_idx,
                                          irq->outpin_idx, s->regs[addr]);
            aspeed_intc_update(s, irq->inpin_idx, irq->outpin_idx, 1);
        } else {
            /* clear irq */
            trace_aspeed_intc_clear_irq(aic->id, irq->inpin_idx,
                                        irq->outpin_idx, 0);
            aspeed_intc_update(s, irq->inpin_idx, irq->outpin_idx, 0);
        }
    }
}

static void aspeed_2700_intc_status_handler_multi_outpins(AspeedINTCState *s,
                                                uint32_t addr, uint64_t data)
{
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    const AspeedINTCIRQ *irq;
    int i;

    if (!data) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Invalid data 0\n", __func__);
        return;
    }

    irq = aspeed_intc_get_irq(aic, addr);

    if (irq->inpin_idx >= aic->num_inpins) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Invalid input pin index number: %d\n",
                      __func__,  irq->inpin_idx);
        return;
    }

    /* clear status */
    s->regs[addr] &= ~data;

    /*
     * The status registers are used for notify sources ISR are executed.
     * If one source ISR is executed, it will clear one bit.
     * If it clear all bits, it means to initialize this register status
     * rather than sources ISR are executed.
     */
    if (data == 0xffffffff) {
        return;
    }

    for (i = 0; i < irq->num_outpins; i++) {
        /* All source ISR executions are done from a specific bit */
        if (data & BIT(i)) {
            trace_aspeed_intc_all_isr_done_bit(aic->id, irq->inpin_idx, i);
            if (s->pending[irq->inpin_idx] & BIT(i)) {
                /*
                 * Handle pending source interrupt.
                 * Notify firmware which source interrupt is pending
                 * by setting the status bit.
                 */
                s->regs[addr] |= BIT(i);
                s->pending[irq->inpin_idx] &= ~BIT(i);
                trace_aspeed_intc_trigger_irq(aic->id,
                                              irq->inpin_idx,
                                              irq->outpin_idx + i,
                                              s->regs[addr]);
                aspeed_intc_update(s, irq->inpin_idx,
                                   irq->outpin_idx + i, 1);
            } else {
                /* clear irq for the specific bit */
                trace_aspeed_intc_clear_irq(aic->id, irq->inpin_idx,
                                            irq->outpin_idx + i, 0);
                aspeed_intc_update(s, irq->inpin_idx, irq->outpin_idx + i, 0);
            }
        }
    }
}

static uint64_t aspeed_2700_intc0_read(void *opaque, hwaddr offset,
                                       unsigned int size)
{
    AspeedINTCState *s = ASPEED_INTC(opaque);
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    uint32_t addr = offset >> 2;
    uint32_t value = 0;

    if (offset >= aic->reg_size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }

    value = s->regs[addr];
    trace_aspeed_intc_read(aic->id, offset, size, value);

    return value;
}

static void aspeed_2700_intc0_write(void *opaque, hwaddr offset, uint64_t data,
                                    unsigned size)
{
    AspeedINTCState *s = ASPEED_INTC(opaque);
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    uint32_t addr = offset >> 2;

    if (offset >= aic->reg_size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds write at offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return;
    }

    trace_aspeed_intc_write(aic->id, offset, size, data);

    switch (addr) {
    case R_INTC0_GICINT128_EN:
    case R_INTC0_GICINT129_EN:
    case R_INTC0_GICINT130_EN:
    case R_INTC0_GICINT131_EN:
    case R_INTC0_GICINT132_EN:
    case R_INTC0_GICINT133_EN:
    case R_INTC0_GICINT134_EN:
    case R_INTC0_GICINT135_EN:
    case R_INTC0_GICINT136_EN:
    case R_INTC0_GICINT192_201_EN:
        aspeed_2700_intc_enable_handler(s, addr, data);
        break;
    case R_INTC0_GICINT128_STATUS:
    case R_INTC0_GICINT129_STATUS:
    case R_INTC0_GICINT130_STATUS:
    case R_INTC0_GICINT131_STATUS:
    case R_INTC0_GICINT132_STATUS:
    case R_INTC0_GICINT133_STATUS:
    case R_INTC0_GICINT134_STATUS:
    case R_INTC0_GICINT135_STATUS:
    case R_INTC0_GICINT136_STATUS:
        aspeed_2700_intc_status_handler(s, addr, data);
        break;
    case R_INTC0_GICINT192_201_STATUS:
        aspeed_2700_intc_status_handler_multi_outpins(s, addr, data);
        break;
    default:
        s->regs[addr] = data;
        break;
    }

    return;
}

static uint64_t aspeed_2700_intc1_read(void *opaque, hwaddr offset,
                                                 unsigned int size)
{
    AspeedINTCState *s = ASPEED_INTC(opaque);
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    uint32_t addr = offset >> 2;
    uint32_t value = 0;

    if (offset >= aic->reg_size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }

    value = s->regs[addr];
    trace_aspeed_intc_read(aic->id, offset, size, value);

    return value;
}

static void aspeed_2700_intc1_write(void *opaque, hwaddr offset, uint64_t data,
                                      unsigned size)
{
    AspeedINTCState *s = ASPEED_INTC(opaque);
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    uint32_t addr = offset >> 2;

    if (offset >= aic->reg_size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds write at offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return;
    }

    trace_aspeed_intc_write(aic->id, offset, size, data);

    switch (addr) {
    case R_INTC1_GICINT192_EN:
    case R_INTC1_GICINT193_EN:
    case R_INTC1_GICINT194_EN:
    case R_INTC1_GICINT195_EN:
    case R_INTC1_GICINT196_EN:
    case R_INTC1_GICINT197_EN:
        aspeed_2700_intc_enable_handler(s, addr, data);
        break;
    case R_INTC1_GICINT192_STATUS:
    case R_INTC1_GICINT193_STATUS:
    case R_INTC1_GICINT194_STATUS:
    case R_INTC1_GICINT195_STATUS:
    case R_INTC1_GICINT196_STATUS:
    case R_INTC1_GICINT197_STATUS:
        aspeed_2700_intc_status_handler(s, addr, data);
        break;
    default:
        s->regs[addr] = data;
        break;
    }

    return;
}

static const MemoryRegionOps aspeed_intc_ops = {
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static void aspeed_intc_instance_init(Object *obj)
{
    AspeedINTCState *s = ASPEED_INTC(obj);
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    int i;

    assert(aic->num_inpins <= ASPEED_INTC_MAX_INPINS);
    for (i = 0; i < aic->num_inpins; i++) {
        object_initialize_child(obj, "intc-orgates[*]", &s->orgates[i],
                                TYPE_OR_IRQ);
        object_property_set_int(OBJECT(&s->orgates[i]), "num-lines",
                                aic->num_lines, &error_abort);
    }
}

static void aspeed_intc_reset(DeviceState *dev)
{
    AspeedINTCState *s = ASPEED_INTC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->enable, 0, sizeof(s->enable));
    memset(s->mask, 0, sizeof(s->mask));
    memset(s->pending, 0, sizeof(s->pending));
}

static void aspeed_intc_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    AspeedINTCState *s = ASPEED_INTC(dev);
    AspeedINTCClass *aic = ASPEED_INTC_GET_CLASS(s);
    int i;

    memory_region_init(&s->iomem_container, OBJECT(s),
            TYPE_ASPEED_INTC ".container", aic->mem_size);

    sysbus_init_mmio(sbd, &s->iomem_container);

    memory_region_init_io(&s->iomem, OBJECT(s), aic->reg_ops, s,
                          TYPE_ASPEED_INTC ".regs", aic->reg_size);

    memory_region_add_subregion(&s->iomem_container, 0x0, &s->iomem);

    qdev_init_gpio_in(dev, aspeed_intc_set_irq, aic->num_inpins);

    for (i = 0; i < aic->num_inpins; i++) {
        if (!qdev_realize(DEVICE(&s->orgates[i]), NULL, errp)) {
            return;
        }
    }

    for (i = 0; i < aic->num_outpins; i++) {
        sysbus_init_irq(sbd, &s->output_pins[i]);
    }
}

static void aspeed_intc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AspeedINTCClass *aic = ASPEED_INTC_CLASS(klass);

    dc->desc = "ASPEED INTC Controller";
    dc->realize = aspeed_intc_realize;
    device_class_set_legacy_reset(dc, aspeed_intc_reset);
    dc->vmsd = NULL;

    aic->reg_ops = &aspeed_intc_ops;
}

static const TypeInfo aspeed_intc_info = {
    .name = TYPE_ASPEED_INTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_init = aspeed_intc_instance_init,
    .instance_size = sizeof(AspeedINTCState),
    .class_init = aspeed_intc_class_init,
    .class_size = sizeof(AspeedINTCClass),
    .abstract = true,
};

static const MemoryRegionOps aspeed_2700_intc0_ops = {
    .read = aspeed_2700_intc0_read,
    .write = aspeed_2700_intc0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static void aspeed_2700_intc0_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AspeedINTCClass *aic = ASPEED_INTC_CLASS(klass);

    dc->desc = "ASPEED 2700 INTC 0 Controller";
    aic->num_lines = 32;
    aic->num_inpins = 10;
    aic->num_outpins = 19;
    aic->reg_ops = &aspeed_2700_intc0_ops;
    aic->mem_size = 0x4000;
    aic->reg_size = 0x2000;
    aic->irq_table = aspeed_2700_intc0_irqs;
    aic->irq_table_count = ARRAY_SIZE(aspeed_2700_intc0_irqs);
    aic->id = 0;
}

static const TypeInfo aspeed_2700_intc0_info = {
    .name = TYPE_ASPEED_2700_INTC0,
    .parent = TYPE_ASPEED_INTC,
    .class_init = aspeed_2700_intc0_class_init,
};

static const MemoryRegionOps aspeed_2700_intc1_ops = {
    .read = aspeed_2700_intc1_read,
    .write = aspeed_2700_intc1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static void aspeed_2700_intc1_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AspeedINTCClass *aic = ASPEED_INTC_CLASS(klass);

    dc->desc = "ASPEED 2700 INTC 1 Controller";
    aic->num_lines = 32;
    aic->num_inpins = 6;
    aic->num_outpins = 6;
    aic->mem_size = 0x400;
    aic->reg_size = 0x3d8;
    aic->reg_ops = &aspeed_2700_intc1_ops;
    aic->irq_table = aspeed_2700_intc1_irqs;
    aic->irq_table_count = ARRAY_SIZE(aspeed_2700_intc1_irqs);
    aic->id = 1;
}

static const TypeInfo aspeed_2700_intc1_info = {
    .name = TYPE_ASPEED_2700_INTC1,
    .parent = TYPE_ASPEED_INTC,
    .class_init = aspeed_2700_intc1_class_init,
};

static void aspeed_intc_register_types(void)
{
    type_register_static(&aspeed_intc_info);
    type_register_static(&aspeed_2700_intc0_info);
    type_register_static(&aspeed_2700_intc1_info);
}

type_init(aspeed_intc_register_types);
