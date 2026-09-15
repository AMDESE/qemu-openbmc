/*
 *  ASPEED LPC Controller
 *
 *  Copyright (C) 2017-2018 IBM Corp.
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#ifndef ASPEED_LPC_H
#define ASPEED_LPC_H

#include "hw/core/sysbus.h"

#define TYPE_ASPEED_LPC "aspeed.lpc"
#define ASPEED_LPC(obj) OBJECT_CHECK(AspeedLPCState, (obj), TYPE_ASPEED_LPC)

#define ASPEED_LPC_NR_REGS      (0x260 >> 2)

enum aspeed_lpc_subdevice {
    aspeed_lpc_kcs_1 = 0,
    aspeed_lpc_kcs_2,
    aspeed_lpc_kcs_3,
    aspeed_lpc_kcs_4,
    aspeed_lpc_ibt,
    aspeed_lpc_pcc,
};

#define ASPEED_LPC_NR_KCS_CHANS (aspeed_lpc_ibt - aspeed_lpc_kcs_1)
#define ASPEED_LPC_NR_SUBDEVS   6

#define ASPEED_LPC_PCC_FIFO_SIZE 256

/*
 * Output GPIO carrying the PME output, HICR1[2] while HICR0[2] enables it.
 * High means PME# asserted.
 */
#define ASPEED_LPC_PME_GPIO "pme"

typedef struct AspeedLPCState {
    /* <private> */
    SysBusDevice parent;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;

    qemu_irq subdevice_irqs[ASPEED_LPC_NR_SUBDEVS];
    qemu_irq pme;
    uint32_t subdevice_irqs_pending;

    uint32_t regs[ASPEED_LPC_NR_REGS];
    uint32_t hicr7;

    MemoryRegion *dma_mr;
    AddressSpace dma_as;
    uint64_t dma_base;
    uint32_t pcc_wptr;

    uint8_t pcc_fifo[ASPEED_LPC_PCC_FIFO_SIZE];
    uint32_t pcc_fifo_rptr;
    uint32_t pcc_fifo_len;

    /* KCS channel 1 and 2 data port addresses, banked behind LADR12H/L */
    uint16_t kcs_ladr[2];
} AspeedLPCState;

void aspeed_lpc_pcc_capture(AspeedLPCState *s, uint32_t port, uint8_t byte);
bool aspeed_lpc_kcs_host_write(AspeedLPCState *s, uint32_t port, uint8_t byte);
bool aspeed_lpc_kcs_host_read(AspeedLPCState *s, uint32_t port, uint8_t *byte);

#endif /* ASPEED_LPC_H */
