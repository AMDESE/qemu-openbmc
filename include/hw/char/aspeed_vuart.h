/*
 * ASPEED AST2700 eSPI Virtual UART (VUART)
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef ASPEED_VUART_H
#define ASPEED_VUART_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/fifo8.h"
#include "qemu/timer.h"

#define TYPE_ASPEED_VUART "aspeed.vuart"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedVUARTState, ASPEED_VUART)

/* Depth of each of the two FIFOs; receive trigger levels go up to 14 */
#define ASPEED_VUART_FIFO_SIZE 16

/*
 * One 16550-compatible register file. The host side is reached over eSPI
 * short I/O cycles; the slave side is reached by the BMC's ARM core over APB
 * MMIO. Each side has its own IER/IIR/LCR/MCR/LSR/MSR/SCR/FCR and its own
 * notion of "the FIFO it pushes into" / "the FIFO it pops from", but the two
 * sides share the underlying storage: what the host pushes to h2s the slave
 * reads back, and vice versa via s2h.
 */
typedef struct AspeedVUARTSide {
    uint8_t ier;
    uint8_t iir;
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr;
    uint8_t msr;
    uint8_t scr;
    uint8_t fcr;
    uint8_t dll;
    uint8_t dlh;

    bool thr_ipending;
    bool timeout_ipending;
    uint8_t recv_fifo_itl;
    QEMUTimer *fifo_timeout_timer;
} AspeedVUARTSide;

struct AspeedVUARTState {
    SysBusDevice parent;

    MemoryRegion iomem;

    AspeedVUARTSide host;
    AspeedVUARTSide slave;

    /* host -> slave and slave -> host, 16 bytes deep, shared by both sides */
    Fifo8 h2s;
    Fifo8 s2h;

    uint32_t gcra;
    uint32_t gcrb;
    uint32_t addrl;
    uint32_t addrh;
    uint32_t gcrf;
    uint32_t gcrg;
    /* GCRE and GCRH only reflect the state of the two sides; no storage */

    /* The last byte the slave wrote to THR, read back through GCRG */
    uint8_t slave_thr;

    qemu_irq irq;      /* slave side -> BMC INTC */
    qemu_irq sirq_out; /* host side -> eSPI SIRQ, level */

    /*
     * Optional, for tests: stands in for the slave end. Bytes it sends reach
     * the host receive FIFO, and bytes the host writes to THR come out of it.
     */
    CharFrontend host_chr;
};

/*
 * Claimed-bool convention, mirroring aspeed_lpc_kcs_host_write/read(): returns
 * false if the port is outside this instance's programmed host window or the
 * instance is disabled (GCRA bit 0), so the eSPI routing code can fall
 * through to whatever comes next in the decode chain.
 */
bool aspeed_vuart_host_write(AspeedVUARTState *s, uint32_t port, uint8_t byte);
bool aspeed_vuart_host_read(AspeedVUARTState *s, uint32_t port, uint8_t *byte);

/*
 * GCRB[7:4], the SIRQ number the guest programmed this instance's host-side
 * interrupt to report as (0=IRQ0 .. 2=SMI .. 15=IRQ15). Used by the eSPI
 * routing code to label a sirq_out level change for the controller.
 */
uint8_t aspeed_vuart_get_sirq_num(AspeedVUARTState *s);

#endif /* ASPEED_VUART_H */
