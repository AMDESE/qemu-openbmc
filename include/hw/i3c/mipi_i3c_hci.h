/*
 * MIPI I3C Host Controller Interface (HCI) base model
 *
 * Models the controller-side state machine and register interface of a
 * MIPI I3C HCI in PIO mode: command, response and IBI queues, the RX/TX
 * data ports, the Device Address Table (DAT), the Device Characteristics
 * Table (DCT) and the Extended Capabilities list.  Vendor implementations
 * derive from this type and use the MIPII3CHCIClass hooks for vendor
 * register windows and addressing conventions.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I3C_MIPI_I3C_HCI_H
#define HW_I3C_MIPI_I3C_HCI_H

#include "hw/core/sysbus.h"
#include "hw/core/registerfields.h"
#include "hw/i3c/i3c.h"
#include "qom/object.h"

#define TYPE_MIPI_I3C_HCI "mipi-i3c-hci"
OBJECT_DECLARE_TYPE(MIPII3CHCIState, MIPII3CHCIClass, MIPI_I3C_HCI)

/* The controller's I3C bus; it takes IBIs from the targets on it. */
#define TYPE_MIPI_I3C_HCI_BUS "mipi-i3c-hci-bus"

/*
 * Default register layout and identity; see MIPII3CHCIClass.  The HCI
 * registers are always at offset 0.
 */
#define MIPI_I3C_HCI_DEFAULT_VERSION         0x100      /* HCI 1.0 */
#define MIPI_I3C_HCI_DEFAULT_CAPABILITIES    0x00004002
#define MIPI_I3C_HCI_DEFAULT_PIO_OFFSET      0x100
#define MIPI_I3C_HCI_DEFAULT_DAT_OFFSET      0x200
#define MIPI_I3C_HCI_DEFAULT_DAT_ENTRIES     16
#define MIPI_I3C_HCI_DEFAULT_DCT_OFFSET      0x280
#define MIPI_I3C_HCI_DEFAULT_DCT_ENTRIES     16
#define MIPI_I3C_HCI_DEFAULT_EXT_CAPS_OFFSET 0x380
#define MIPI_I3C_HCI_DEFAULT_MMIO_SIZE       0x400

/* Register block sizes, in 32-bit words. */
#define MIPI_I3C_HCI_HCI_NR_REGS   (0x80 / 4)
#define MIPI_I3C_HCI_PIO_NR_REGS   (0x30 / 4)
/* Bytes of the window given to the Extended Capabilities list. */
#define MIPI_I3C_HCI_EXT_CAPS_SIZE 0x80

/* Table capacity; a class may use fewer entries. */
#define MIPI_I3C_HCI_MAX_DAT_ENTRIES 127
#define MIPI_I3C_HCI_MAX_DCT_ENTRIES 16
/* A DAT entry is two words, a DCT entry four. */
#define MIPI_I3C_HCI_DAT_NR_REGS   (MIPI_I3C_HCI_MAX_DAT_ENTRIES * 2)
#define MIPI_I3C_HCI_DCT_NR_REGS   (MIPI_I3C_HCI_MAX_DCT_ENTRIES * 4)

/* Queue and buffer depths. */
#define MIPI_I3C_HCI_RESP_FIFO_SIZE   8
#define MIPI_I3C_HCI_RX_FIFO_DWORDS   64
#define MIPI_I3C_HCI_IBI_FIFO_SIZE    32
#define MIPI_I3C_HCI_TX_BUF_SIZE      512

/* Response descriptor */
FIELD(MIPI_I3C_HCI_RESP, DATA_LEN, 0, 22)
FIELD(MIPI_I3C_HCI_RESP, TID, 24, 4)
FIELD(MIPI_I3C_HCI_RESP, STATUS, 28, 4)

/* Response status codes */
#define MIPI_I3C_HCI_RESP_SUCCESS          0x0
#define MIPI_I3C_HCI_RESP_ERR_ADDR_HEADER  0x4
#define MIPI_I3C_HCI_RESP_ERR_NACK         0x5
#define MIPI_I3C_HCI_RESP_ERR_OVL          0x6

static inline uint32_t mipi_i3c_hci_resp(uint32_t status, uint32_t tid,
                                         uint32_t data_len)
{
    uint32_t resp = 0;

    resp = FIELD_DP32(resp, MIPI_I3C_HCI_RESP, STATUS, status);
    resp = FIELD_DP32(resp, MIPI_I3C_HCI_RESP, TID, tid);
    return FIELD_DP32(resp, MIPI_I3C_HCI_RESP, DATA_LEN, data_len);
}

/**
 * struct MIPII3CHCIState - generic MIPI I3C HCI controller state
 *
 * Holds the standard register blocks (HCI, PIO, DAT, DCT), the PIO
 * command/response/IBI/RX/TX queues, and the I3CBus that targets attach to.
 */
struct MIPII3CHCIState {
    SysBusDevice parent_obj;

    MemoryRegion mr;
    qemu_irq     irq;
    uint8_t      id;        /* controller index, used in logs and traces */
    I3CBus      *bus;

    /* Standard register blocks */
    uint32_t hci_regs[MIPI_I3C_HCI_HCI_NR_REGS];
    uint32_t pio_regs[MIPI_I3C_HCI_PIO_NR_REGS];
    uint32_t dat[MIPI_I3C_HCI_DAT_NR_REGS];
    uint32_t dct[MIPI_I3C_HCI_DCT_NR_REGS];

    /* A command is written to the command queue port as two words. */
    uint32_t cmd_word0;
    bool     cmd_have_word0;

    /* Response queue (circular). */
    uint32_t resp_fifo[MIPI_I3C_HCI_RESP_FIFO_SIZE];
    uint32_t resp_rd, resp_wr;

    /* RX data queue (circular, in words). */
    uint32_t rx_fifo[MIPI_I3C_HCI_RX_FIFO_DWORDS];
    uint32_t rx_rd, rx_wr;

    /*
     * IBI status and data queue (circular, in words).  Each event is a
     * status descriptor followed by its data words.
     */
    uint32_t ibi_fifo[MIPI_I3C_HCI_IBI_FIFO_SIZE];
    uint32_t ibi_rd, ibi_wr;

    /* TX data written to the data port ahead of a command. */
    uint8_t  tx_buf[MIPI_I3C_HCI_TX_BUF_SIZE];
    uint32_t tx_len;

    /* IBI a target on the bus is raising. */
    bool     ibi_active;
    uint8_t  ibi_da;
    uint8_t  ibi_mdb;
    bool     ibi_has_mdb;

};

/**
 * struct MIPII3CHCIClass - vendor hooks
 *
 * The base class sets the MIPI_I3C_HCI_DEFAULT_* layout and identity; a
 * vendor class can override them in its class_init:
 * @mmio_size: size of the register window
 * @version: HCI_VERSION; from 1.1 on, table sizes are given in entries
 * @capabilities: HC_CAPABILITIES
 * @pio_offset: offset of the PIO registers
 * @dat_offset, @dat_entries: offset and size of the DAT, at most
 *   MIPI_I3C_HCI_MAX_DAT_ENTRIES entries
 * @dct_offset, @dct_entries: offset and size of the DCT, at most
 *   MIPI_I3C_HCI_MAX_DCT_ENTRIES entries
 * @ext_caps_offset: offset of the Extended Capabilities list
 *
 * Every hook is optional unless stated otherwise.
 */
struct MIPII3CHCIClass {
    SysBusDeviceClass parent_class;

    uint32_t mmio_size;
    uint32_t version;
    uint32_t capabilities;
    uint32_t pio_offset;
    uint32_t dat_offset;
    uint32_t dat_entries;
    uint32_t dct_offset;
    uint32_t dct_entries;
    uint32_t ext_caps_offset;

    /*
     * Return the read-only Extended Capabilities list, found at
     * @ext_caps_offset.  Default: empty.
     */
    const uint32_t *(*get_extcaps)(MIPII3CHCIState *s, size_t *nwords);

    /*
     * Handle accesses outside the standard blocks.  Default: logged as an
     * invalid access.
     */
    uint64_t (*ext_read)(MIPII3CHCIState *s, hwaddr off, unsigned size);
    void     (*ext_write)(MIPII3CHCIState *s, hwaddr off, uint64_t v,
                          unsigned size);

    /* Reset vendor state after the base state has been reset. */
    void     (*ext_reset)(MIPII3CHCIState *s);

    /*
     * Translate a command's DEV_INDEX into the target's dynamic address.
     * Default: the address held in the DAT entry at @dev_idx.
     */
    uint8_t  (*dat_to_da)(MIPII3CHCIState *s, uint8_t dev_idx);

    /*
     * DAT entry of the target at dynamic address @da, or -1 if no entry
     * is for it.  Default: the I3C entry whose address field holds @da.
     */
    int      (*da_to_dat)(MIPII3CHCIState *s, uint8_t da);

    /*
     * Dynamic address to assign to the @iter'th (0-based) device of an
     * address assignment command whose DEV_INDEX is @dev_idx_base.
     * Default: the address software stored in DAT entry
     * @dev_idx_base + @iter.  Returns 0xff when no address is left, which
     * ends the assignment.
     */
    uint8_t  (*entdaa_next_da)(MIPII3CHCIState *s, uint8_t dev_idx_base,
                                uint8_t iter);

    /* Called after an IBI is queued. */
    void     (*post_ibi)(MIPII3CHCIState *s, uint8_t da);

    /*
     * Called when a command completes, for vendor "transfer done" status.
     * @ok is false if the command was NACKed or failed.
     */
    void     (*post_cmd)(MIPII3CHCIState *s, bool ok, bool is_read);
};

I3CBus *mipi_i3c_hci_get_bus(MIPII3CHCIState *s);

/* True if an enabled PIO interrupt is pending. */
bool mipi_i3c_hci_pio_irq_pending(MIPII3CHCIState *s);

/*
 * Queue an In-Band Interrupt from @da that did not come through the
 * I3CBus.  Returns true if the IBI was queued; the SIR_REJECT bit of the
 * DAT entry for @da still applies.
 */
bool mipi_i3c_hci_inject_ibi(MIPII3CHCIState *s, uint8_t da,
                              uint8_t ibi_byte, bool has_byte);

/*
 * Complete a read the controller started on its own, such as an automatic
 * read after an IBI, as a command completes: queue @rx and @resp and update
 * the PIO status.  If @rx does not fit in the RX queue, none of it is queued
 * and the response reports an overflow instead.
 */
void mipi_i3c_hci_complete_autocmd(MIPII3CHCIState *s, const uint8_t *rx,
                                   uint32_t rx_len, uint32_t resp);

extern const VMStateDescription vmstate_mipi_i3c_hci;

#endif /* HW_I3C_MIPI_I3C_HCI_H */
