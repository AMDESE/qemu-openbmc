/*
 * ASPEED AST2700 TAFS/SAFS Global Controller
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef ASPEED_TAFSC_H
#define ASPEED_TAFSC_H

#include "hw/core/sysbus.h"

#define TYPE_ASPEED_TAFSC "aspeed.tafsc"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedTAFSCState, ASPEED_TAFSC)

#define ASPEED_TAFSC_SIZE       0x10000
#define ASPEED_TAFSC_NR_BRIDGES 2

struct AspeedTAFSCState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;
    uint32_t erase_ctrl;
};

/* Memory mode erase fill of @bridge: 0x00 or 0xff */
uint8_t aspeed_tafsc_erase_fill(AspeedTAFSCState *s, unsigned int bridge);

#endif /* ASPEED_TAFSC_H */
