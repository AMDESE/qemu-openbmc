/*
 * ASPEED AST2700 eSPI Virtual UART (VUART)
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A 16550-compatible UART with two register files: the host side is reached
 * through I/O cycles at a programmable window (see
 * aspeed_vuart_host_write/read()), the slave side by the BMC over MMIO. The
 * two sides are coupled by two 16-byte FIFOs, h2s (host Tx, slave Rx) and
 * s2h (slave Tx, host Rx); no physical line sits between them.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "chardev/char-fe.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/char/aspeed_vuart.h"
#include "migration/vmstate.h"

#define TO_REG(offset) ((offset) >> 2)

/* Slave-side MMIO: 0x00-0x1C classic set, 0x20-0x3C control/status */
#define VUART_REG_RBR_THR_DLL 0x00
#define VUART_REG_IER_DLH     0x04
#define VUART_REG_IIR_FCR     0x08
#define VUART_REG_LCR         0x0c
#define VUART_REG_MCR         0x10
#define VUART_REG_LSR         0x14
#define VUART_REG_MSR         0x18
#define VUART_REG_SCR         0x1c
#define VUART_REG_GCRA        0x20
#define VUART_REG_GCRB        0x24
#define VUART_REG_ADDRL       0x28
#define VUART_REG_ADDRH       0x2c
#define VUART_REG_GCRE        0x30
#define VUART_REG_GCRF        0x34
#define VUART_REG_GCRG        0x38
#define VUART_REG_GCRH        0x3c
#define VUART_NR_REGS         (TO_REG(VUART_REG_GCRH) + 1)

/* Host side: same classic set, but 8 consecutive byte-wide I/O ports */
#define VUART_HOST_WINDOW_SIZE 8

/* 16550 register bits */
#define UART_LCR_DLAB   0x80
#define UART_LCR_BRK    0x40

#define UART_IER_MSI    0x08
#define UART_IER_RLSI   0x04
#define UART_IER_THRI   0x02
#define UART_IER_RDI    0x01

#define UART_IIR_NO_INT 0x01
#define UART_IIR_ID     0x06
#define UART_IIR_MSI    0x00
#define UART_IIR_THRI   0x02
#define UART_IIR_RDI    0x04
#define UART_IIR_RLSI   0x06
#define UART_IIR_CTI    0x0c
#define UART_IIR_FE     0xc0

#define UART_MCR_LOOP   0x10
#define UART_MCR_OUT2   0x08
#define UART_MCR_OUT1   0x04
#define UART_MCR_RTS    0x02
#define UART_MCR_DTR    0x01

#define UART_MSR_DCD    0x80
#define UART_MSR_RI     0x40
#define UART_MSR_DSR    0x20
#define UART_MSR_CTS    0x10
#define UART_MSR_DDCD   0x08
#define UART_MSR_TERI   0x04
#define UART_MSR_DDSR   0x02
#define UART_MSR_DCTS   0x01
#define UART_MSR_ANY_DELTA 0x0f

#define UART_LSR_TEMT   0x40
#define UART_LSR_THRE   0x20
#define UART_LSR_BI     0x10
#define UART_LSR_OE     0x02
#define UART_LSR_DR     0x01
/*
 * With no physical line, framing and parity errors cannot happen, so the
 * line status interrupt only comes from an overrun or a break the other side
 * sent.
 */
#define VUART_LSR_INT_ANY (UART_LSR_OE | UART_LSR_BI)

#define UART_FCR_ITL_1  0x00
#define UART_FCR_ITL_2  0x40
#define UART_FCR_ITL_3  0x80
#define UART_FCR_ITL_4  0xc0
#define UART_FCR_DMS    0x08
#define UART_FCR_XFR    0x04
#define UART_FCR_RFR    0x02
#define UART_FCR_FE     0x01

/* GCRA, offset 0x20 */
#define GCRA_ENABLE          (1u << 0)
#define GCRA_SIRQ_POLARITY   (1u << 1)
#define GCRA_TMO_WIDTH       (3u << 2)
#define GCRA_DISABLE_DISCARD (1u << 5)
#define GCRA_HOST_LOOPBACK_RO (1u << 4)
#define GCRA_HOST_FIFO_TRIG_RO (3u << 6)
#define GCRA_RW_MASK (GCRA_ENABLE | GCRA_SIRQ_POLARITY | GCRA_TMO_WIDTH | \
                      GCRA_DISABLE_DISCARD)

/* GCRB, offset 0x24 */
#define GCRB_WORDLEN_RO_MASK  0x03
#define GCRB_TMO_PERIOD_MASK  (3u << 2)
#define GCRB_SIRQ_MASK        (0xfu << 4)
#define GCRB_SIRQ_SHIFT       4
#define GCRB_RW_MASK (GCRB_TMO_PERIOD_MASK | GCRB_SIRQ_MASK)

/* ADDRL, offset 0x28 */
#define ADDRL_VERSION_RO   (1u << 0)
#define ADDRL_HALT_RX_DMA  (1u << 1)
#define ADDRL_HALT_TX_DMA  (1u << 2)
#define ADDRL_BASE_LOW_MASK 0xf8
#define ADDRL_RW_MASK (ADDRL_HALT_RX_DMA | ADDRL_HALT_TX_DMA | \
                       ADDRL_BASE_LOW_MASK)

/* GCRF, offset 0x34 */
#define GCRF_MIRROR_HOST_REGS (1u << 2)
#define GCRF_SLAVE_CTI_DIS    (1u << 1)
#define GCRF_HOST_CTI_DIS     (1u << 0)
#define GCRF_THRE_HALF_MODE   (1u << 6)

/* GCRG, offset 0x38: timeout width, counted in clocks when set */
#define GCRG_SLAVE_TMO_CLK    (1u << 1)
#define GCRG_HOST_TMO_CLK     (1u << 0)
#define GCRG_RW_MASK          (GCRG_SLAVE_TMO_CLK | GCRG_HOST_TMO_CLK)

/* GCRH, offset 0x3c */
#define GCRH_HOST_IER7        (1u << 5)
#define GCRH_HOST_FE          (1u << 4)

/* The LPC clock, which the clock-counted timeouts run on */
#define VUART_LPC_CLOCK_HZ    33000000

static uint32_t vuart_host_base(AspeedVUARTState *s)
{
    return (s->addrh << 8) | (s->addrl & ADDRL_BASE_LOW_MASK);
}

/*
 * h2s is the host's Tx FIFO. With host Tx discard disabled (GCRA[5]) its
 * bytes wait for the slave to read them, so THRE/TEMT follow its occupancy;
 * with discard enabled the host is never held off.
 */
static bool vuart_host_tx_empty(AspeedVUARTState *s)
{
    return !(s->gcra & GCRA_DISABLE_DISCARD) || fifo8_is_empty(&s->h2s);
}

/*
 * s2h is the slave's Tx FIFO. Its bytes wait for the host to read them, so
 * the slave's THRE/TEMT follow its occupancy and the slave is held off while
 * the host has not drained it.
 */
static bool vuart_slave_tx_empty(AspeedVUARTState *s)
{
    return fifo8_is_empty(&s->s2h);
}

/*
 * The 16550 interrupt priority encoder for one side. rx_fifo is the FIFO
 * that side receives from (s2h for the host, h2s for the slave).
 */
static void vuart_side_update_irq(AspeedVUARTSide *side, Fifo8 *rx_fifo,
                                   qemu_irq out)
{
    uint8_t tmp_iir = UART_IIR_NO_INT;

    if ((side->ier & UART_IER_RLSI) && (side->lsr & VUART_LSR_INT_ANY)) {
        tmp_iir = UART_IIR_RLSI;
    } else if ((side->ier & UART_IER_RDI) && side->timeout_ipending) {
        tmp_iir = UART_IIR_CTI;
    } else if ((side->ier & UART_IER_RDI) && (side->lsr & UART_LSR_DR) &&
               fifo8_num_used(rx_fifo) >= side->recv_fifo_itl) {
        tmp_iir = UART_IIR_RDI;
    } else if ((side->ier & UART_IER_THRI) && side->thr_ipending) {
        tmp_iir = UART_IIR_THRI;
    } else if ((side->ier & UART_IER_MSI) && (side->msr & UART_MSR_ANY_DELTA)) {
        tmp_iir = UART_IIR_MSI;
    }

    side->iir = tmp_iir | UART_IIR_FE;

    qemu_set_irq(out, tmp_iir != UART_IIR_NO_INT);
}

/* Re-evaluate one side's interrupt against its own receive fifo and line */
static void vuart_update_irq(AspeedVUARTState *s, AspeedVUARTSide *side)
{
    if (side == &s->host) {
        vuart_side_update_irq(side, &s->s2h, s->sirq_out);
    } else {
        vuart_side_update_irq(side, &s->h2s, s->irq);
    }
}

/* Raise or drop the host THRE interrupt as its Tx FIFO empties or fills */
static void vuart_host_tx_update(AspeedVUARTState *s, bool was_empty)
{
    bool empty = vuart_host_tx_empty(s);

    if (empty != was_empty) {
        s->host.thr_ipending = empty;
        vuart_side_update_irq(&s->host, &s->s2h, s->sirq_out);
    }
}

/* Raise or drop the slave THRE interrupt as its Tx FIFO empties or fills */
static void vuart_slave_tx_update(AspeedVUARTState *s, bool was_empty)
{
    bool empty = vuart_slave_tx_empty(s);

    if (empty != was_empty) {
        s->slave.thr_ipending = empty;
        vuart_side_update_irq(&s->slave, &s->h2s, s->irq);
    }
}

/*
 * Character timeout: raised when the receive FIFO still holds data a
 * timeout period after the last byte went in or came out, unless GCRF
 * disables it for that side.
 */
static void vuart_fifo_timeout(AspeedVUARTState *s, AspeedVUARTSide *side,
                               Fifo8 *rx_fifo)
{
    uint32_t dis = side == &s->host ? GCRF_HOST_CTI_DIS : GCRF_SLAVE_CTI_DIS;

    if (!(s->gcrf & dis) && !fifo8_is_empty(rx_fifo)) {
        side->timeout_ipending = true;
        vuart_update_irq(s, side);
    }
}

static void vuart_host_fifo_timeout(void *opaque)
{
    AspeedVUARTState *s = opaque;

    vuart_fifo_timeout(s, &s->host, &s->s2h);
}

static void vuart_slave_fifo_timeout(void *opaque)
{
    AspeedVUARTState *s = opaque;

    vuart_fifo_timeout(s, &s->slave, &s->h2s);
}

/*
 * The timeout period of one side: GCRA[3:2] for the slave, GCRB[3:2] for
 * the host select 1/3600 to 1/28800 second, or 512 down to 64 clocks when
 * the side's GCRG width bit is set.
 */
static int64_t vuart_timeout_ns(AspeedVUARTState *s, AspeedVUARTSide *side)
{
    bool host = side == &s->host;
    uint32_t sel = ((host ? s->gcrb : s->gcra) >> 2) & 3;
    uint32_t clk = host ? GCRG_HOST_TMO_CLK : GCRG_SLAVE_TMO_CLK;

    if (s->gcrg & clk) {
        return (int64_t)(512 >> sel) * NANOSECONDS_PER_SECOND /
               VUART_LPC_CLOCK_HZ;
    }
    return NANOSECONDS_PER_SECOND / (3600 << sel);
}

static void vuart_rx_timeout_rearm(AspeedVUARTState *s, AspeedVUARTSide *side,
                                   Fifo8 *rx_fifo)
{
    if (fifo8_is_empty(rx_fifo)) {
        side->timeout_ipending = false;
        timer_del(side->fifo_timeout_timer);
        return;
    }
    timer_mod(side->fifo_timeout_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              vuart_timeout_ns(s, side));
}

/*
 * Push a byte into the receive FIFO of dst_side. A full FIFO drops the
 * incoming byte and raises OE on the receiving side, rather than evicting
 * an old byte to make room.
 */
static void vuart_fifo_push(AspeedVUARTState *s, Fifo8 *fifo, uint8_t byte,
                            AspeedVUARTSide *dst_side, qemu_irq dst_irq)
{
    if (fifo8_is_full(fifo)) {
        dst_side->lsr |= UART_LSR_OE;
    } else {
        fifo8_push(fifo, byte);
        dst_side->lsr |= UART_LSR_DR;
        vuart_rx_timeout_rearm(s, dst_side, fifo);
    }
    vuart_side_update_irq(dst_side, fifo, dst_irq);
}

static uint8_t vuart_fifo_pop(Fifo8 *fifo, AspeedVUARTSide *side)
{
    uint8_t byte = 0;

    if (!fifo8_is_empty(fifo)) {
        byte = fifo8_pop(fifo);
    }
    if (fifo8_is_empty(fifo)) {
        side->lsr &= ~UART_LSR_DR;
    }
    return byte;
}

static void vuart_write_fcr(AspeedVUARTSide *side, Fifo8 *rx_fifo,
                             Fifo8 *tx_fifo, uint8_t val)
{
    if (val & UART_FCR_RFR) {
        fifo8_reset(rx_fifo);
        side->lsr &= ~UART_LSR_DR;
        side->timeout_ipending = false;
        timer_del(side->fifo_timeout_timer);
    }
    if (val & UART_FCR_XFR) {
        fifo8_reset(tx_fifo);
    }

    switch (val & 0xc0) {
    case UART_FCR_ITL_1:
        side->recv_fifo_itl = 1;
        break;
    case UART_FCR_ITL_2:
        side->recv_fifo_itl = 4;
        break;
    case UART_FCR_ITL_3:
        side->recv_fifo_itl = 8;
        break;
    case UART_FCR_ITL_4:
        side->recv_fifo_itl = 14;
        break;
    }

    /* The FIFOs cannot be turned off: FE always reads 1 */
    side->fcr = (val & 0xc0) | UART_FCR_FE;
}

static void vuart_write_lcr(AspeedVUARTState *s, AspeedVUARTSide *side,
                            AspeedVUARTSide *other, uint8_t val)
{
    bool break_was_set = side->lcr & UART_LCR_BRK;
    bool break_now_set = val & UART_LCR_BRK;

    /*
     * Bits 5:2 are stored as written rather than masked off: this matches
     * real 16550 behaviour for word-length/stop/parity bits that this model
     * does not otherwise interpret, and avoids silently dropping
     * guest-visible state on a readback.
     */
    side->lcr = val;

    if (break_now_set && !break_was_set) {
        /* Break is transmitted to the *opposite* side. */
        other->lsr |= UART_LSR_BI;
        vuart_update_irq(s, other);
    } else if (!break_now_set && break_was_set) {
        other->lsr &= ~UART_LSR_BI;
        vuart_update_irq(s, other);
    }
}

static void vuart_write_mcr(AspeedVUARTState *s, AspeedVUARTSide *side,
                            uint8_t val)
{
    side->mcr = val & 0x1f;

    if (side->mcr & UART_MCR_LOOP) {
        uint8_t msr = 0;

        msr |= (side->mcr & UART_MCR_RTS) ? UART_MSR_CTS : 0;
        msr |= (side->mcr & UART_MCR_DTR) ? UART_MSR_DSR : 0;
        msr |= (side->mcr & UART_MCR_OUT1) ? UART_MSR_RI : 0;
        msr |= (side->mcr & UART_MCR_OUT2) ? UART_MSR_DCD : 0;
        if ((msr & UART_MSR_CTS) != (side->msr & UART_MSR_CTS)) {
            msr |= UART_MSR_DCTS;
        }
        if ((msr & UART_MSR_DSR) != (side->msr & UART_MSR_DSR)) {
            msr |= UART_MSR_DDSR;
        }
        /* The slave MSR has no trailing edge ring indicator */
        if (side == &s->host &&
            (msr & UART_MSR_RI) && !(side->msr & UART_MSR_RI)) {
            msr |= UART_MSR_TERI;
        }
        if ((msr & UART_MSR_DCD) != (side->msr & UART_MSR_DCD)) {
            msr |= UART_MSR_DDCD;
        }
        side->msr = msr;
    }
}

/* ---- Host side: 8 consecutive byte-wide I/O ports ---- */

bool aspeed_vuart_host_write(AspeedVUARTState *s, uint32_t port, uint8_t byte)
{
    uint32_t base = vuart_host_base(s);
    uint32_t off;
    AspeedVUARTSide *side = &s->host;

    if (!(s->gcra & GCRA_ENABLE)) {
        return false;
    }
    if (port < base || port >= base + VUART_HOST_WINDOW_SIZE) {
        return false;
    }
    off = port - base;

    switch (off) {
    case 0: /* RBR/THR/DLL */
        if (side->lcr & UART_LCR_DLAB) {
            side->dll = byte;
            break;
        }
        vuart_fifo_push(s, &s->h2s, byte, &s->slave, s->irq);
        side->thr_ipending = vuart_host_tx_empty(s);
        vuart_side_update_irq(side, &s->s2h, s->sirq_out);
        break;
    case 1: /* IER/DLH */
        if (side->lcr & UART_LCR_DLAB) {
            side->dlh = byte;
        } else {
            /* Bit 7, half full THRE mode, is writable while GCRF[6] is */
            side->ier = byte & (UART_IER_MSI | UART_IER_RLSI |
                                UART_IER_THRI | UART_IER_RDI |
                                ((s->gcrf & GCRF_THRE_HALF_MODE) ? 0x80 : 0));
            vuart_side_update_irq(side, &s->s2h, s->sirq_out);
        }
        break;
    case 2: /* FCR (write) */
        {
            bool tx_was_empty = vuart_host_tx_empty(s);
            bool slave_tx_was_empty = vuart_slave_tx_empty(s);

            vuart_write_fcr(side, &s->s2h, &s->h2s, byte);
            vuart_side_update_irq(side, &s->s2h, s->sirq_out);
            vuart_host_tx_update(s, tx_was_empty);
            vuart_slave_tx_update(s, slave_tx_was_empty);
        }
        break;
    case 3: /* LCR */
        vuart_write_lcr(s, side, &s->slave, byte);
        break;
    case 4: /* MCR */
        vuart_write_mcr(s, side, byte);
        vuart_side_update_irq(side, &s->s2h, s->sirq_out);
        break;
    case 5: /* LSR -- storage only; guest cannot force error bits from here */
        break;
    case 6: /* MSR -- delta bits are hardware-derived, not guest-writable */
        break;
    case 7: /* SCR */
        side->scr = byte;
        break;
    default:
        g_assert_not_reached();
    }

    if (s->host_chr.chr) {
        /* Test affordance only: mirror what the guest wrote to THR. */
        if (off == 0 && !(side->lcr & UART_LCR_DLAB)) {
            qemu_chr_fe_write_all(&s->host_chr, &byte, 1);
        }
    }

    return true;
}

bool aspeed_vuart_host_read(AspeedVUARTState *s, uint32_t port, uint8_t *byte)
{
    uint32_t base = vuart_host_base(s);
    uint32_t off;
    AspeedVUARTSide *side = &s->host;

    if (!(s->gcra & GCRA_ENABLE)) {
        return false;
    }
    if (port < base || port >= base + VUART_HOST_WINDOW_SIZE) {
        return false;
    }
    off = port - base;

    switch (off) {
    case 0: /* RBR/DLL */
        if (side->lcr & UART_LCR_DLAB) {
            *byte = side->dll;
        } else {
            bool slave_tx_was_empty = vuart_slave_tx_empty(s);

            *byte = vuart_fifo_pop(&s->s2h, side);
            side->timeout_ipending = false;
            vuart_rx_timeout_rearm(s, side, &s->s2h);
            vuart_side_update_irq(side, &s->s2h, s->sirq_out);
            vuart_slave_tx_update(s, slave_tx_was_empty);
        }
        break;
    case 1: /* IER/DLH */
        *byte = (side->lcr & UART_LCR_DLAB) ? side->dlh : side->ier;
        break;
    case 2: /* IIR */
        *byte = side->iir;
        if ((side->iir & UART_IIR_ID) == UART_IIR_THRI) {
            side->thr_ipending = false;
            vuart_side_update_irq(side, &s->s2h, s->sirq_out);
        }
        break;
    case 3:
        *byte = side->lcr;
        break;
    case 4:
        *byte = side->mcr;
        break;
    case 5: /* LSR -- read clears OE/BI, the only error bits this model has */
        *byte = side->lsr & ~(UART_LSR_THRE | UART_LSR_TEMT);
        if (vuart_host_tx_empty(s)) {
            *byte |= UART_LSR_THRE | UART_LSR_TEMT;
        }
        side->lsr &= ~(UART_LSR_OE | UART_LSR_BI);
        vuart_side_update_irq(side, &s->s2h, s->sirq_out);
        break;
    case 6: /* MSR -- read clears the delta bits */
        *byte = side->msr;
        side->msr &= ~UART_MSR_ANY_DELTA;
        vuart_side_update_irq(side, &s->s2h, s->sirq_out);
        break;
    case 7:
        *byte = side->scr;
        break;
    default:
        g_assert_not_reached();
    }

    return true;
}

uint8_t aspeed_vuart_get_sirq_num(AspeedVUARTState *s)
{
    return (s->gcrb & GCRB_SIRQ_MASK) >> GCRB_SIRQ_SHIFT;
}

/* ---- Slave side: MMIO, word-spaced ---- */

static void aspeed_vuart_write(void *opaque, hwaddr addr, uint64_t val64,
                                unsigned size)
{
    AspeedVUARTState *s = ASPEED_VUART(opaque);
    AspeedVUARTSide *side = &s->slave;
    uint32_t val = val64;
    uint32_t reg = TO_REG(addr);

    if (reg >= VUART_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: out-of-bounds write @0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return;
    }

    switch (addr) {
    case VUART_REG_RBR_THR_DLL:
        if (side->lcr & UART_LCR_DLAB) {
            if (!(s->gcrf & GCRF_MIRROR_HOST_REGS)) {
                side->dll = val;
            }
            break;
        }
        s->slave_thr = val;
        vuart_fifo_push(s, &s->s2h, val, &s->host, s->sirq_out);
        side->thr_ipending = vuart_slave_tx_empty(s);
        vuart_side_update_irq(side, &s->h2s, s->irq);
        break;
    case VUART_REG_IER_DLH:
        if (side->lcr & UART_LCR_DLAB) {
            if (!(s->gcrf & GCRF_MIRROR_HOST_REGS)) {
                side->dlh = val;
            }
        } else {
            side->ier = val & (UART_IER_MSI | UART_IER_RLSI |
                               UART_IER_THRI | UART_IER_RDI);
            vuart_side_update_irq(side, &s->h2s, s->irq);
        }
        break;
    case VUART_REG_IIR_FCR:
        {
            bool tx_was_empty = vuart_host_tx_empty(s);
            bool slave_tx_was_empty = vuart_slave_tx_empty(s);

            vuart_write_fcr(side, &s->h2s, &s->s2h, val);
            vuart_side_update_irq(side, &s->h2s, s->irq);
            vuart_host_tx_update(s, tx_was_empty);
            vuart_slave_tx_update(s, slave_tx_was_empty);
        }
        break;
    case VUART_REG_LCR:
        vuart_write_lcr(s, side, &s->host, val);
        break;
    case VUART_REG_MCR:
        vuart_write_mcr(s, side, val);
        vuart_side_update_irq(side, &s->h2s, s->irq);
        break;
    case VUART_REG_LSR:
        break;
    case VUART_REG_MSR:
        break;
    case VUART_REG_SCR:
        /*
         * GCRF bit2 makes the slave's SCR (and DLL/DLH) an RO mirror of the
         * host's value -- a real cross-register mode switch, so honour it
         * rather than silently accepting the write.
         */
        if (!(s->gcrf & GCRF_MIRROR_HOST_REGS)) {
            side->scr = val;
        }
        break;
    case VUART_REG_GCRA:
        {
            bool tx_was_empty = vuart_host_tx_empty(s);

            s->gcra = val & GCRA_RW_MASK;
            vuart_host_tx_update(s, tx_was_empty);
        }
        break;
    case VUART_REG_GCRB:
        s->gcrb = (s->gcrb & ~GCRB_RW_MASK) | (val & GCRB_RW_MASK);
        break;
    case VUART_REG_ADDRL:
        s->addrl = (val & ADDRL_RW_MASK) | ADDRL_VERSION_RO;
        break;
    case VUART_REG_ADDRH:
        s->addrh = val & 0xff;
        break;
    case VUART_REG_GCRE:
        /* Read-only; guest writes are silently dropped. */
        break;
    case VUART_REG_GCRF:
        s->gcrf = val;
        break;
    case VUART_REG_GCRG:
        s->gcrg = val & GCRG_RW_MASK;
        break;
    case VUART_REG_GCRH:
        /* Read-only. */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        break;
    }
}

static uint64_t aspeed_vuart_read(void *opaque, hwaddr addr, unsigned size)
{
    AspeedVUARTState *s = ASPEED_VUART(opaque);
    AspeedVUARTSide *side = &s->slave;
    uint32_t reg = TO_REG(addr);

    if (reg >= VUART_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: out-of-bounds read @0x%" HWADDR_PRIx
                      "\n", __func__, addr);
        return 0;
    }

    switch (addr) {
    case VUART_REG_RBR_THR_DLL:
        if (side->lcr & UART_LCR_DLAB) {
            return (s->gcrf & GCRF_MIRROR_HOST_REGS) ? s->host.dll : side->dll;
        }
        {
            bool tx_was_empty = vuart_host_tx_empty(s);
            uint8_t byte = vuart_fifo_pop(&s->h2s, side);
            side->timeout_ipending = false;
            vuart_rx_timeout_rearm(s, side, &s->h2s);
            vuart_side_update_irq(side, &s->h2s, s->irq);
            vuart_host_tx_update(s, tx_was_empty);
            return byte;
        }
    case VUART_REG_IER_DLH:
        if (side->lcr & UART_LCR_DLAB) {
            return (s->gcrf & GCRF_MIRROR_HOST_REGS) ? s->host.dlh : side->dlh;
        }
        return side->ier;
    case VUART_REG_IIR_FCR:
        {
            uint8_t iir = side->iir;
            if ((side->iir & UART_IIR_ID) == UART_IIR_THRI) {
                side->thr_ipending = false;
                vuart_side_update_irq(side, &s->h2s, s->irq);
            }
            return iir;
        }
    case VUART_REG_LCR:
        return side->lcr;
    case VUART_REG_MCR:
        return side->mcr;
    case VUART_REG_LSR:
        {
            uint8_t lsr = side->lsr & ~(UART_LSR_THRE | UART_LSR_TEMT);

            if (vuart_slave_tx_empty(s)) {
                lsr |= UART_LSR_THRE | UART_LSR_TEMT;
            }
            side->lsr &= ~(UART_LSR_OE | UART_LSR_BI);
            vuart_side_update_irq(side, &s->h2s, s->irq);
            return lsr;
        }
    case VUART_REG_MSR:
        {
            uint8_t msr = side->msr;
            side->msr &= ~UART_MSR_ANY_DELTA;
            vuart_side_update_irq(side, &s->h2s, s->irq);
            return msr;
        }
    case VUART_REG_SCR:
        return (s->gcrf & GCRF_MIRROR_HOST_REGS) ? s->host.scr : side->scr;
    case VUART_REG_GCRA:
        /* Bits 7:6 report the host receive FIFO trigger level */
        return s->gcra |
               (s->host.mcr & UART_MCR_LOOP ? GCRA_HOST_LOOPBACK_RO : 0) |
               (s->host.fcr & GCRA_HOST_FIFO_TRIG_RO);
    case VUART_REG_GCRB:
        return (s->gcrb & ~GCRB_WORDLEN_RO_MASK) |
               (s->host.lcr & GCRB_WORDLEN_RO_MASK);
    case VUART_REG_ADDRL:
        return s->addrl;
    case VUART_REG_ADDRH:
        return s->addrh;
    case VUART_REG_GCRE:
        /*
         * Bit 7 is the slave transmit FIFO full, bits 3:0 the complement of
         * the host IIR bits 3:0. The THR read pointer in bits 6:4 is not
         * modelled and reads 0.
         */
        return (fifo8_is_full(&s->s2h) ? (1u << 7) : 0) |
               ((~s->host.iir) & 0x0f);
    case VUART_REG_GCRF:
        return s->gcrf;
    case VUART_REG_GCRG:
        /* The timeout width bits with GCRF[2] set, else the slave THR */
        return (s->gcrf & GCRF_MIRROR_HOST_REGS) ? s->gcrg : s->slave_thr;
    case VUART_REG_GCRH:
        return (s->slave.fcr & 0xc0) |
               ((s->host.ier & 0x80) ? GCRH_HOST_IER7 : 0) |
               ((s->host.fcr & UART_FCR_FE) ? GCRH_HOST_FE : 0) |
               (s->host.ier & 0x0f);
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return 0;
    }
}

static const MemoryRegionOps aspeed_vuart_ops = {
    .read = aspeed_vuart_read,
    .write = aspeed_vuart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---- Optional chardev, for tests: stands in for the slave end ---- */

static int aspeed_vuart_chr_can_receive(void *opaque)
{
    AspeedVUARTState *s = ASPEED_VUART(opaque);

    return fifo8_num_free(&s->s2h);
}

static void aspeed_vuart_chr_receive(void *opaque, const uint8_t *buf, int size)
{
    AspeedVUARTState *s = ASPEED_VUART(opaque);
    int i;

    /* Bytes from the chardev reach the host as if the slave sent them */
    for (i = 0; i < size; i++) {
        vuart_fifo_push(s, &s->s2h, buf[i], &s->host, s->sirq_out);
    }
}

static void aspeed_vuart_chr_event(void *opaque, QEMUChrEvent event)
{
}

/* ---- Reset / realize / vmstate / props ---- */

static void aspeed_vuart_side_reset(AspeedVUARTSide *side)
{
    side->ier = 0x00;
    side->iir = 0xc1;
    side->lcr = 0x03;
    side->mcr = 0x00;
    side->lsr = 0x60;
    side->msr = 0x00;
    side->scr = 0x00;
    side->fcr = UART_FCR_FE;
    side->dll = 0x00;
    side->dlh = 0x00;
    side->thr_ipending = false;
    side->timeout_ipending = false;
    side->recv_fifo_itl = 1;
    if (side->fifo_timeout_timer) {
        timer_del(side->fifo_timeout_timer);
    }
}

static void aspeed_vuart_reset_hold(Object *obj, ResetType type)
{
    AspeedVUARTState *s = ASPEED_VUART(obj);

    aspeed_vuart_side_reset(&s->host);
    aspeed_vuart_side_reset(&s->slave);
    fifo8_reset(&s->h2s);
    fifo8_reset(&s->s2h);

    s->gcra = 0x00;
    /* GCRB low bits mirror host LCR word-length, reset 0b11 -- see LCR. */
    s->gcrb = 0x03;
    s->addrl = ADDRL_VERSION_RO;
    s->addrh = 0x00;
    s->gcrf = 0x00;
    s->gcrg = 0x00;
    s->slave_thr = 0x00;

    qemu_set_irq(s->irq, 0);
    qemu_set_irq(s->sirq_out, 0);
}

static void aspeed_vuart_realize(DeviceState *dev, Error **errp)
{
    AspeedVUARTState *s = ASPEED_VUART(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &aspeed_vuart_ops, s,
                           TYPE_ASPEED_VUART, 0x40);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->sirq_out);

    fifo8_create(&s->h2s, ASPEED_VUART_FIFO_SIZE);
    fifo8_create(&s->s2h, ASPEED_VUART_FIFO_SIZE);

    s->host.fifo_timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                               vuart_host_fifo_timeout, s);
    s->slave.fifo_timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                                vuart_slave_fifo_timeout, s);

    if (qemu_chr_fe_backend_connected(&s->host_chr)) {
        qemu_chr_fe_set_handlers(&s->host_chr, aspeed_vuart_chr_can_receive,
                                  aspeed_vuart_chr_receive,
                                  aspeed_vuart_chr_event, NULL, s, NULL, true);
    }
}

static const VMStateDescription vmstate_aspeed_vuart_side = {
    .name = "aspeed-vuart-side",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(ier, AspeedVUARTSide),
        VMSTATE_UINT8(iir, AspeedVUARTSide),
        VMSTATE_UINT8(lcr, AspeedVUARTSide),
        VMSTATE_UINT8(mcr, AspeedVUARTSide),
        VMSTATE_UINT8(lsr, AspeedVUARTSide),
        VMSTATE_UINT8(msr, AspeedVUARTSide),
        VMSTATE_UINT8(scr, AspeedVUARTSide),
        VMSTATE_UINT8(fcr, AspeedVUARTSide),
        VMSTATE_UINT8(dll, AspeedVUARTSide),
        VMSTATE_UINT8(dlh, AspeedVUARTSide),
        VMSTATE_BOOL(thr_ipending, AspeedVUARTSide),
        VMSTATE_BOOL(timeout_ipending, AspeedVUARTSide),
        VMSTATE_UINT8(recv_fifo_itl, AspeedVUARTSide),
        VMSTATE_TIMER_PTR(fifo_timeout_timer, AspeedVUARTSide),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_aspeed_vuart = {
    .name = TYPE_ASPEED_VUART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(host, AspeedVUARTState, 1, vmstate_aspeed_vuart_side,
                       AspeedVUARTSide),
        VMSTATE_STRUCT(slave, AspeedVUARTState, 1, vmstate_aspeed_vuart_side,
                       AspeedVUARTSide),
        VMSTATE_FIFO8(h2s, AspeedVUARTState),
        VMSTATE_FIFO8(s2h, AspeedVUARTState),
        VMSTATE_UINT32(gcra, AspeedVUARTState),
        VMSTATE_UINT32(gcrb, AspeedVUARTState),
        VMSTATE_UINT32(addrl, AspeedVUARTState),
        VMSTATE_UINT32(addrh, AspeedVUARTState),
        VMSTATE_UINT32(gcrf, AspeedVUARTState),
        VMSTATE_UINT32(gcrg, AspeedVUARTState),
        VMSTATE_UINT8(slave_thr, AspeedVUARTState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property aspeed_vuart_properties[] = {
    DEFINE_PROP_CHR("chardev", AspeedVUARTState, host_chr),
};

static void aspeed_vuart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = aspeed_vuart_realize;
    rc->phases.hold = aspeed_vuart_reset_hold;
    dc->vmsd = &vmstate_aspeed_vuart;
    dc->desc = "ASPEED AST2700 eSPI Virtual UART";
    device_class_set_props(dc, aspeed_vuart_properties);
}

static const TypeInfo aspeed_vuart_info = {
    .name = TYPE_ASPEED_VUART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AspeedVUARTState),
    .class_init = aspeed_vuart_class_init,
};

static void aspeed_vuart_register_types(void)
{
    type_register_static(&aspeed_vuart_info);
}

type_init(aspeed_vuart_register_types)
