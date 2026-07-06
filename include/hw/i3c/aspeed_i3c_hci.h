/*
 * ASPEED AST2700 MIPI I3C HCI controller
 *
 * A MIPI I3C HCI with the ASPEED INHOUSE and PHY register blocks, the
 * AUTOCMD automatic read, ASPEED Extended Capabilities, and commands that
 * address a device by its dynamic address.  A container device holds the
 * 16 controllers (i3c0 to i3c15) in one 64 KiB window.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I3C_ASPEED_I3C_HCI_H
#define HW_I3C_ASPEED_I3C_HCI_H

#include "hw/core/sysbus.h"
#include "hw/i3c/mipi_i3c_hci.h"
#include "qom/object.h"

#define TYPE_ASPEED_I3C_HCI       "aspeed.i3c-hci"
#define TYPE_ASPEED_I3C_HCI_CTRL  "aspeed.i3c-hci.ctrl"

OBJECT_DECLARE_SIMPLE_TYPE(AspeedI3CHCIState, ASPEED_I3C_HCI)
OBJECT_DECLARE_SIMPLE_TYPE(AspeedI3CHCICtrl, ASPEED_I3C_HCI_CTRL)

#define ASPEED_I3C_HCI_NR_CTRLS    16
/* Each controller takes 4 KiB. */
#define ASPEED_I3C_HCI_CTRL_SIZE   0x1000
#define ASPEED_I3C_HCI_TOTAL_SIZE  (ASPEED_I3C_HCI_NR_CTRLS * \
                                    ASPEED_I3C_HCI_CTRL_SIZE)

/* Vendor register block sizes, in 32-bit words. */
#define ASPEED_I3C_HCI_INHOUSE_NR_REGS (0x100 / 4)
#define ASPEED_I3C_HCI_PHY_NR_REGS     (0x100 / 4)

/**
 * struct AspeedI3CHCICtrl - one AST2700 I3C controller
 *
 * The standard MIPI HCI state plus the INHOUSE (0xd00) and PHY (0xe00)
 * register blocks.
 */
struct AspeedI3CHCICtrl {
    MIPII3CHCIState parent_obj;

    uint32_t inhouse_regs[ASPEED_I3C_HCI_INHOUSE_NR_REGS];
    uint32_t phy_regs[ASPEED_I3C_HCI_PHY_NR_REGS];
};

/**
 * struct AspeedI3CHCIState - the 16 AST2700 I3C controllers
 */
struct AspeedI3CHCIState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    AspeedI3CHCICtrl ctrls[ASPEED_I3C_HCI_NR_CTRLS];
};

I3CBus *aspeed_i3c_hci_get_bus(AspeedI3CHCICtrl *ctrl);
bool aspeed_i3c_hci_inject_ibi(AspeedI3CHCICtrl *ctrl, uint8_t da,
                               uint8_t ibi_byte, bool has_byte);

#endif /* HW_I3C_ASPEED_I3C_HCI_H */
