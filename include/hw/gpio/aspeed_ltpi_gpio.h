/*
 * ASPEED AST2700 LTPI GPIO controller
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_GPIO_ASPEED_LTPI_GPIO_H
#define HW_GPIO_ASPEED_LTPI_GPIO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ASPEED_LTPI_GPIO "aspeed.ltpi-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedLtpiGpioState, ASPEED_LTPI_GPIO)

/* An input and an output line per pair, one control register each */
#define ASPEED_LTPI_GPIO_NR_PAIRS 112

struct AspeedLtpiGpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq out[ASPEED_LTPI_GPIO_NR_PAIRS];

    uint32_t ctrl[ASPEED_LTPI_GPIO_NR_PAIRS];
    /* Set by the first reset; later resets keep reset tolerant outputs */
    bool powered;
};

#endif /* HW_GPIO_ASPEED_LTPI_GPIO_H */
