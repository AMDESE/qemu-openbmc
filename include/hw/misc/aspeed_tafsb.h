/*
 * ASPEED AST2700 TAFS/SAFS Bridge Controller
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef ASPEED_TAFSB_H
#define ASPEED_TAFSB_H

#include "hw/core/sysbus.h"
#include "hw/misc/aspeed_tafsc.h"

#define TYPE_ASPEED_TAFSB "aspeed.tafsb"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedTAFSBState, ASPEED_TAFSB)

/* The block spans 0x200, the stride between the two instances */
#define ASPEED_TAFSB_SIZE     0x200
#define ASPEED_TAFSB_NR_REGS  (ASPEED_TAFSB_SIZE >> 2)

struct AspeedTAFSBState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    /* The AHB the bridge issues its accesses on */
    MemoryRegion *ahb_mr;
    AddressSpace ahb_as;

    /* Chip select of the SPI controller with the flash, the first by default */
    uint32_t cs;

    /* Shared global controller and this bridge's index in it */
    AspeedTAFSCState *tafsc;
    uint32_t index;

    uint32_t regs[ASPEED_TAFSB_NR_REGS];
};

/*
 * The 40-bit base of the flash window this bridge drives. The eSPI flash
 * channel has a mapping target of its own, and the two are expected to name
 * the same window; the channel reports it when they do not.
 */
uint64_t aspeed_tafsb_mem_base(AspeedTAFSBState *s);

/* Flash operations on a 40-bit AHB address within the flash window */
MemTxResult aspeed_tafsb_ahb_read(AspeedTAFSBState *s,
                                  uint64_t addr,
                                  void *buf, uint32_t len);
MemTxResult aspeed_tafsb_program(AspeedTAFSBState *s,
                                 uint64_t addr,
                                 const void *buf, uint32_t len);
MemTxResult aspeed_tafsb_erase(AspeedTAFSBState *s, uint64_t addr,
                               uint32_t size);

#endif /* ASPEED_TAFSB_H */
