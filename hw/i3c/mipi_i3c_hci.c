/*
 * MIPI I3C Host Controller Interface (HCI) base model
 *
 * Default register map of one controller; a vendor class can move the
 * blocks after the HCI registers and size the tables (MIPII3CHCIClass):
 *   0x000-0x07f  HCI registers (HC_CONTROL, interrupts, *_SECTION, ...)
 *   0x100-0x12f  PIO registers (command, response, data and IBI ports, ...)
 *   0x200-0x27f  Device Address Table (DAT), 16 entries of 8 bytes
 *   0x280-0x37f  Device Characteristics Table (DCT), 16 entries of 16 bytes
 *   0x380-0x3ff  Extended Capabilities, provided by the subclass
 *   elsewhere    Vendor registers, handled by the subclass
 *
 * Targets attach to the controller's I3CBus (mipi_i3c_hci_get_bus()), and
 * commands reach them through the I3CBus API.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/registerfields.h"
#include "hw/i3c/i3c.h"
#include "hw/i3c/mipi_i3c_hci.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "trace.h"

/* HCI registers, at the start of the window */
REG32(HCI_VERSION, 0x00)
REG32(HC_CONTROL, 0x04)
    FIELD(HC_CONTROL, ABORT, 29, 1)
REG32(HC_CAPABILITIES, 0x0c)
REG32(RESET_CONTROL, 0x10)
    FIELD(RESET_CONTROL, SOFT_RST, 0, 1)
    FIELD(RESET_CONTROL, CMD_QUEUE_RST, 1, 1)
    FIELD(RESET_CONTROL, RESP_QUEUE_RST, 2, 1)
    FIELD(RESET_CONTROL, TX_FIFO_RST, 3, 1)
    FIELD(RESET_CONTROL, RX_FIFO_RST, 4, 1)
    FIELD(RESET_CONTROL, IBI_QUEUE_RST, 5, 1)
REG32(PRESENT_STATE, 0x14)
    FIELD(PRESENT_STATE, CURRENT_MASTER, 2, 1)
REG32(INTR_STATUS, 0x20)
REG32(DAT_SECTION, 0x30)
    FIELD(DAT_SECTION, TABLE_OFFSET, 0, 12)
    FIELD(DAT_SECTION, TABLE_SIZE, 12, 7)
REG32(DCT_SECTION, 0x34)
    FIELD(DCT_SECTION, TABLE_OFFSET, 0, 12)
    FIELD(DCT_SECTION, TABLE_SIZE, 12, 7)
    FIELD(DCT_SECTION, TABLE_INDEX, 19, 5)
REG32(RING_HEADERS_SECTION, 0x38)
REG32(PIO_SECTION, 0x3c)
REG32(EXT_CAPS_SECTION, 0x40)

#define HCI_REGS_SIZE            0x80


/* PIO registers, relative to the PIO block */
REG32(PIO_CMD_QUEUE_PORT, 0x00)
REG32(PIO_RESP_QUEUE_PORT, 0x04)
REG32(PIO_XFER_DATA_PORT, 0x08)
REG32(PIO_IBI_PORT, 0x0c)
REG32(PIO_DATA_BUFFER_THLD_CTRL, 0x14)
    FIELD(PIO_DATA_BUFFER_THLD_CTRL, RX_BUF_THLD, 8, 3)
REG32(PIO_QUEUE_SIZE, 0x18)
    FIELD(PIO_QUEUE_SIZE, CR_QUEUE_SIZE, 0, 8)
    FIELD(PIO_QUEUE_SIZE, IBI_STATUS_SIZE, 8, 8)
    FIELD(PIO_QUEUE_SIZE, RX_DATA_BUFFER_SIZE, 16, 8)
    FIELD(PIO_QUEUE_SIZE, TX_DATA_BUFFER_SIZE, 24, 8)
REG32(PIO_ALT_QUEUE_SIZE, 0x1c)
REG32(PIO_INTR_STATUS, 0x20)
    FIELD(PIO_INTR_STATUS, TX_THLD, 0, 1)
    FIELD(PIO_INTR_STATUS, RX_THLD, 1, 1)
    FIELD(PIO_INTR_STATUS, IBI_STATUS_THLD, 2, 1)
    FIELD(PIO_INTR_STATUS, CMD_QUEUE_READY, 3, 1)
    FIELD(PIO_INTR_STATUS, RESP_READY, 4, 1)
    FIELD(PIO_INTR_STATUS, TRANSFER_ERR, 9, 1)
REG32(PIO_INTR_SIGNAL_ENABLE, 0x28)
REG32(PIO_INTR_FORCE, 0x2c)

/*
 * QUEUE_SIZE reports the queue depths: the TX and RX buffer sizes are
 * encoded as 4 * (2 << n) bytes, the IBI queue size is in words and the
 * command/response queue size in entries.  TX 512 bytes (n = 6), RX 256
 * bytes (n = 5), IBI 32 words.
 */
#define PIO_QUEUE_SIZE_RESET                                         \
    ((6 << R_PIO_QUEUE_SIZE_TX_DATA_BUFFER_SIZE_SHIFT) |             \
     (5 << R_PIO_QUEUE_SIZE_RX_DATA_BUFFER_SIZE_SHIFT) |             \
     (MIPI_I3C_HCI_IBI_FIFO_SIZE << R_PIO_QUEUE_SIZE_IBI_STATUS_SIZE_SHIFT) | \
     (MIPI_I3C_HCI_RESP_FIFO_SIZE << R_PIO_QUEUE_SIZE_CR_QUEUE_SIZE_SHIFT))

/*
 * Command descriptor.  DEV_INDEX is 7 bits wide (W0[22:16]) so that it can
 * carry a dynamic address.
 */
FIELD(CMD0, ATTR, 0, 3)
FIELD(CMD0, TID, 3, 4)
FIELD(CMD0, CCC_CODE, 7, 8)
FIELD(CMD0, CP, 15, 1)
FIELD(CMD0, DEV_INDEX, 16, 7)
FIELD(CMD0, IMM_DTT, 23, 3)
FIELD(CMD0, DEV_COUNT, 26, 4)          /* address assignment only */
FIELD(CMD0, RNW, 29, 1)
FIELD(CMD0, ROC, 30, 1)
FIELD(CMD1, DATA_LEN, 16, 16)

#define CMD_ATTR_REGULAR         0x0
#define CMD_ATTR_IMMEDIATE       0x1
#define CMD_ATTR_ADDRASSIGN      0x2
#define CMD_ATTR_CCC             0x3
#define CMD_ATTR_INTERNAL        0x7

/* IBI status descriptor */
FIELD(IBI_STATUS, DATA_LEN, 0, 8)
FIELD(IBI_STATUS, IBI_ID, 8, 8)
FIELD(IBI_STATUS, LAST_STATUS, 24, 1)

/*
 * DAT entry, word 0.  The model acts on the static address, on SIR_REJECT
 * to drop IBIs and on the I2C device flag; the other fields are stored for
 * software to read back.
 */
FIELD(DAT_W0, STATIC_ADDR, 0, 7)
FIELD(DAT_W0, SIR_REJECT, 13, 1)
FIELD(DAT_W0, DYNAMIC_ADDR, 16, 7)
FIELD(DAT_W0, I2C_DEVICE, 31, 1)

static void mipi_i3c_hci_update_irq(MIPII3CHCIState *s)
{
    uint32_t status = s->pio_regs[R_PIO_INTR_STATUS];
    uint32_t sig_en = s->pio_regs[R_PIO_INTR_SIGNAL_ENABLE];
    int level = !!(status & sig_en);

    trace_mipi_i3c_hci_irq(s->id, level, status, sig_en);
    qemu_set_irq(s->irq, level);
}

static void mipi_i3c_hci_set_transfer_err(MIPII3CHCIState *s)
{
    s->pio_regs[R_PIO_INTR_STATUS] |= R_PIO_INTR_STATUS_TRANSFER_ERR_MASK;
    mipi_i3c_hci_update_irq(s);
}

static uint32_t mipi_i3c_hci_dat_entries(MIPII3CHCIState *s)
{
    return MIPI_I3C_HCI_GET_CLASS(s)->dat_entries;
}

/* Before HCI 1.1, a table size is given in words, from 1.1 on in entries. */
static uint32_t mipi_i3c_hci_table_size(MIPII3CHCIClass *klass,
                                        uint32_t entries, uint32_t words)
{
    return klass->version < 0x110 ? entries * words : entries;
}

bool mipi_i3c_hci_pio_irq_pending(MIPII3CHCIState *s)
{
    return s->pio_regs[R_PIO_INTR_STATUS] &
           s->pio_regs[R_PIO_INTR_SIGNAL_ENABLE];
}

static void mipi_i3c_hci_push_resp(MIPII3CHCIState *s, uint32_t resp)
{
    uint32_t next = (s->resp_wr + 1) % MIPI_I3C_HCI_RESP_FIFO_SIZE;

    if (next == s->resp_rd) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: i3c%u response queue overflow\n", __func__, s->id);
        mipi_i3c_hci_set_transfer_err(s);
        return;
    }
    trace_mipi_i3c_hci_resp_push(s->id, resp);
    s->resp_fifo[s->resp_wr] = resp;
    s->resp_wr = next;
    s->pio_regs[R_PIO_INTR_STATUS] |= R_PIO_INTR_STATUS_RESP_READY_MASK;
}

static uint32_t mipi_i3c_hci_pop_resp(MIPII3CHCIState *s)
{
    uint32_t resp;

    if (s->resp_rd == s->resp_wr) {
        return 0;
    }
    resp = s->resp_fifo[s->resp_rd];
    s->resp_rd = (s->resp_rd + 1) % MIPI_I3C_HCI_RESP_FIFO_SIZE;
    trace_mipi_i3c_hci_resp_pop(s->id, resp);
    if (s->resp_rd == s->resp_wr) {
        s->pio_regs[R_PIO_INTR_STATUS] &= ~R_PIO_INTR_STATUS_RESP_READY_MASK;
    }
    return resp;
}

static void mipi_i3c_hci_push_ibi(MIPII3CHCIState *s, uint32_t word)
{
    uint32_t next = (s->ibi_wr + 1) % MIPI_I3C_HCI_IBI_FIFO_SIZE;

    if (next == s->ibi_rd) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: i3c%u IBI queue overflow\n", __func__, s->id);
        mipi_i3c_hci_set_transfer_err(s);
        return;
    }
    s->ibi_fifo[s->ibi_wr] = word;
    s->ibi_wr = next;
}

static void mipi_i3c_hci_push_rx(MIPII3CHCIState *s, const uint8_t *buf,
                                  uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i += 4) {
        uint32_t word = 0;
        uint32_t next = (s->rx_wr + 1) % MIPI_I3C_HCI_RX_FIFO_DWORDS;

        memcpy(&word, buf + i, MIN(4u, len - i));
        if (next == s->rx_rd) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: i3c%u RX queue overflow\n", __func__, s->id);
            mipi_i3c_hci_set_transfer_err(s);
            return;
        }
        s->rx_fifo[s->rx_wr] = word;
        s->rx_wr = next;
    }
}

static void mipi_i3c_hci_check_thresholds(MIPII3CHCIState *s)
{
    /*
     * RX_BUF_THLD is encoded: 0 -> 1 word, 1 -> 4, 2 -> 8, 3 -> 16,
     * 4 -> 32, 5 -> 64, and larger values are capped at the RX queue depth.
     */
    static const uint32_t rx_buf_thld_words[8] = {
        1, 4, 8, 16, 32, 64, 64, 64
    };
    uint32_t thld = ARRAY_FIELD_EX32(s->pio_regs, PIO_DATA_BUFFER_THLD_CTRL,
                                     RX_BUF_THLD);
    uint32_t rx_thld = MIN(rx_buf_thld_words[thld],
                           (uint32_t)MIPI_I3C_HCI_RX_FIFO_DWORDS);
    uint32_t rx_count = (s->rx_wr - s->rx_rd) % MIPI_I3C_HCI_RX_FIFO_DWORDS;
    uint32_t new_bits = 0;

    if (rx_count >= rx_thld) {
        new_bits |= R_PIO_INTR_STATUS_RX_THLD_MASK;
    }
    /* The TX threshold is met once the buffer has drained. */
    if (s->tx_len == 0) {
        new_bits |= R_PIO_INTR_STATUS_TX_THLD_MASK;
    }

    if (new_bits) {
        s->pio_regs[R_PIO_INTR_STATUS] |= new_bits;
        mipi_i3c_hci_update_irq(s);
    }
}

void mipi_i3c_hci_complete_autocmd(MIPII3CHCIState *s, const uint8_t *rx,
                                   uint32_t rx_len, uint32_t resp)
{
    uint32_t rx_count = (s->rx_wr - s->rx_rd) % MIPI_I3C_HCI_RX_FIFO_DWORDS;
    uint32_t rx_room = MIPI_I3C_HCI_RX_FIFO_DWORDS - 1 - rx_count;

    /* The response cannot describe part of the data: queue all or none. */
    if (DIV_ROUND_UP(rx_len, 4) > rx_room) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: i3c%u no RX queue room for %u bytes\n",
                      __func__, s->id, rx_len);
        rx_len = 0;
        resp = mipi_i3c_hci_resp(MIPI_I3C_HCI_RESP_ERR_OVL,
                                 FIELD_EX32(resp, MIPI_I3C_HCI_RESP, TID), 0);
    }
    if (rx_len) {
        mipi_i3c_hci_push_rx(s, rx, rx_len);
    }
    mipi_i3c_hci_check_thresholds(s);
    mipi_i3c_hci_push_resp(s, resp);
    mipi_i3c_hci_update_irq(s);
}

/*
 * Bus transfers.  A device whose DAT entry has the I2C device flag set is
 * reached through the legacy I2C bus at its static address; any other
 * device through I3C SDR transfers.
 */
static bool mipi_i3c_hci_dat_is_i2c(MIPII3CHCIState *s, uint8_t dev_idx,
                                    uint8_t *addr)
{
    uint32_t w0;

    if (dev_idx >= mipi_i3c_hci_dat_entries(s)) {
        return false;
    }
    w0 = s->dat[dev_idx * 2];
    if (!FIELD_EX32(w0, DAT_W0, I2C_DEVICE)) {
        return false;
    }
    *addr = FIELD_EX32(w0, DAT_W0, STATIC_ADDR);
    return true;
}

static bool mipi_i3c_hci_i2c_write(MIPII3CHCIState *s, uint8_t addr,
                                   const uint8_t *data, uint32_t len)
{
    uint32_t i;

    if (legacy_i2c_start_send(s->bus, addr) != 0) {
        return false;
    }
    for (i = 0; i < len; i++) {
        if (legacy_i2c_send(s->bus, data[i]) != 0) {
            legacy_i2c_end_transfer(s->bus);
            return false;
        }
    }
    legacy_i2c_end_transfer(s->bus);
    return true;
}

static bool mipi_i3c_hci_i2c_read(MIPII3CHCIState *s, uint8_t addr,
                                  uint8_t *data, uint32_t len,
                                  uint32_t *out_len)
{
    uint32_t i;

    if (legacy_i2c_start_recv(s->bus, addr) != 0) {
        return false;
    }
    for (i = 0; i < len; i++) {
        data[i] = legacy_i2c_recv(s->bus);
    }
    legacy_i2c_end_transfer(s->bus);
    *out_len = len;
    return true;
}

static bool mipi_i3c_hci_write_data(MIPII3CHCIState *s, uint8_t dev_idx,
                                    uint8_t da, const uint8_t *data,
                                    uint32_t len)
{
    uint32_t sent = 0;
    uint8_t i2c_addr;
    int rc;

    if (mipi_i3c_hci_dat_is_i2c(s, dev_idx, &i2c_addr)) {
        return mipi_i3c_hci_i2c_write(s, i2c_addr, data, len);
    }

    if (i3c_start_transfer(s->bus, da, false) != 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: i3c%u no device at 0x%02x\n",
                      __func__, s->id, da);
        return false;
    }
    rc = i3c_send(s->bus, data, len, &sent);
    i3c_end_transfer(s->bus);
    return rc == 0 && sent == len;
}

static bool mipi_i3c_hci_read_data(MIPII3CHCIState *s, uint8_t dev_idx,
                                   uint8_t da, uint8_t *data, uint32_t len,
                                   uint32_t *out_len)
{
    uint32_t recv = 0;
    uint8_t i2c_addr;
    int rc;

    if (mipi_i3c_hci_dat_is_i2c(s, dev_idx, &i2c_addr)) {
        return mipi_i3c_hci_i2c_read(s, i2c_addr, data, len, out_len);
    }

    if (i3c_start_transfer(s->bus, da, true) != 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: i3c%u no device at 0x%02x\n",
                      __func__, s->id, da);
        return false;
    }
    rc = i3c_recv(s->bus, data, len, &recv);
    i3c_end_transfer(s->bus);
    if (rc != 0) {
        return false;
    }
    *out_len = recv;
    return true;
}

static bool mipi_i3c_hci_ccc_xfer(MIPII3CHCIState *s, uint8_t ccc_id,
                                  bool rnw, uint8_t da, uint8_t *data,
                                  uint32_t len, uint32_t *out_len)
{
    uint32_t actual = 0;
    bool ok;

    if (i3c_start_transfer(s->bus, I3C_BROADCAST, false) != 0) {
        return false;
    }
    ok = i3c_send_byte(s->bus, ccc_id) == 0 &&
         i3c_start_transfer(s->bus, da, rnw) == 0;
    if (ok) {
        if (rnw) {
            ok = i3c_recv(s->bus, data, len, &actual) == 0;
        } else {
            ok = i3c_send(s->bus, data, len, &actual) == 0;
        }
    }
    i3c_end_transfer(s->bus);
    if (!ok) {
        return false;
    }
    *out_len = actual;
    return true;
}

/* Inline data of an IMMEDIATE command: up to 4 bytes in W1. */
static uint32_t mipi_i3c_hci_imm_data(uint32_t w0, uint32_t w1, uint8_t *buf)
{
    uint32_t len = MIN(FIELD_EX32(w0, CMD0, IMM_DTT), 4u);
    uint32_t i;

    for (i = 0; i < len; i++) {
        buf[i] = extract32(w1, 8 * i, 8);
    }
    return len;
}

static void mipi_i3c_hci_process_cmd(MIPII3CHCIState *s, uint32_t w0,
                                     uint32_t w1)
{
    MIPII3CHCIClass *klass = MIPI_I3C_HCI_GET_CLASS(s);
    uint8_t attr = FIELD_EX32(w0, CMD0, ATTR);
    uint8_t tid = FIELD_EX32(w0, CMD0, TID);
    uint8_t dev_idx = FIELD_EX32(w0, CMD0, DEV_INDEX);
    bool roc = FIELD_EX32(w0, CMD0, ROC);
    bool rnw = FIELD_EX32(w0, CMD0, RNW);
    /*
     * REGULAR and CCC commands give the data length in W1; IMMEDIATE
     * commands hold the data itself in W1 and ADDRASSIGN a device count
     * in W0.
     */
    uint16_t data_len = (attr == CMD_ATTR_REGULAR || attr == CMD_ATTR_CCC) ?
                        FIELD_EX32(w1, CMD1, DATA_LEN) : 0;
    uint32_t resp = 0;
    uint32_t actual_len = 0;
    bool ok = true;
    bool completed = false;

    trace_mipi_i3c_hci_cmd(s->id, attr, tid, dev_idx, rnw, data_len);

    switch (attr) {
    case CMD_ATTR_REGULAR:
    case CMD_ATTR_IMMEDIATE: {
        uint8_t da = klass->dat_to_da(s, dev_idx);

        if (FIELD_EX32(w0, CMD0, CP)) {
            /*
             * A CCC carried by a REGULAR or IMMEDIATE command.  CCC codes
             * 0x00-0x7f are broadcast to every device; 0x80-0xfe are
             * directed at the device at @dev_idx.  The IMMEDIATE form
             * carries up to 4 data bytes in W1, the REGULAR form takes its
             * data from the TX buffer.  Software uses the IMMEDIATE form for
             * short CCC writes such as RSTDAA and DISEC.
             */
            uint8_t ccc_id = FIELD_EX32(w0, CMD0, CCC_CODE);
            bool is_bcst = !(ccc_id & 0x80);
            uint8_t target = is_bcst ? I3C_BROADCAST : da;
            uint8_t ccc_buf[MIPI_I3C_HCI_TX_BUF_SIZE];
            uint32_t buf_len;
            uint32_t got = 0;

            if (attr == CMD_ATTR_IMMEDIATE) {
                buf_len = mipi_i3c_hci_imm_data(w0, w1, ccc_buf);
            } else {
                buf_len = MIN((uint32_t)data_len, sizeof(ccc_buf));
                if (!rnw && s->tx_len > 0) {
                    memcpy(ccc_buf, s->tx_buf, MIN(s->tx_len, buf_len));
                } else {
                    memset(ccc_buf, 0, buf_len);
                }
            }
            s->tx_len = 0;

            ok = mipi_i3c_hci_ccc_xfer(s, ccc_id, rnw, target, ccc_buf,
                                       buf_len, &got);
            if (ok && rnw) {
                buf_len = got;
            }
            if (rnw && ok && buf_len > 0) {
                mipi_i3c_hci_push_rx(s, ccc_buf, buf_len);
                actual_len = buf_len;
            }
            mipi_i3c_hci_check_thresholds(s);
            resp = mipi_i3c_hci_resp(ok ? MIPI_I3C_HCI_RESP_SUCCESS :
                                     MIPI_I3C_HCI_RESP_ERR_ADDR_HEADER,
                                     tid, actual_len);
            completed = true;
            break;
        }

        if (attr == CMD_ATTR_IMMEDIATE && !rnw) {
            uint8_t imm_buf[4];
            uint32_t imm_len = mipi_i3c_hci_imm_data(w0, w1, imm_buf);

            s->tx_len = 0;
            if (imm_len > 0) {
                ok = mipi_i3c_hci_write_data(s, dev_idx, da, imm_buf, imm_len);
            }
            resp = mipi_i3c_hci_resp(ok ? MIPI_I3C_HCI_RESP_SUCCESS :
                                     MIPI_I3C_HCI_RESP_ERR_NACK, tid, 0);
            completed = true;
            break;
        }

        if (rnw) {
            if (data_len > 0) {
                uint8_t rx_buf[MIPI_I3C_HCI_TX_BUF_SIZE];
                uint32_t read_len = MIN(data_len, sizeof(rx_buf));
                uint32_t got = 0;

                ok = mipi_i3c_hci_read_data(s, dev_idx, da, rx_buf, read_len,
                                            &got);
                if (ok) {
                    actual_len = got;
                    mipi_i3c_hci_push_rx(s, rx_buf, got);
                    mipi_i3c_hci_check_thresholds(s);
                }
            }
            resp = mipi_i3c_hci_resp(ok ? MIPI_I3C_HCI_RESP_SUCCESS :
                                     MIPI_I3C_HCI_RESP_ERR_NACK,
                                     tid, actual_len);
        } else {
            /*
             * The TX data port takes whole words, so the TX data can be up
             * to 3 bytes longer than the command's DATA_LEN.  Send exactly
             * DATA_LEN bytes: a payload checksum computed over the declared
             * length would fail with the padding on the wire.
             */
            uint32_t wlen = MIN(s->tx_len, (uint32_t)data_len);

            if (wlen > 0) {
                ok = mipi_i3c_hci_write_data(s, dev_idx, da, s->tx_buf, wlen);
            }
            s->tx_len = 0;
            mipi_i3c_hci_check_thresholds(s);
            resp = mipi_i3c_hci_resp(ok ? MIPI_I3C_HCI_RESP_SUCCESS :
                                     MIPI_I3C_HCI_RESP_ERR_NACK, tid, 0);
        }
        completed = true;
        break;
    }

    case CMD_ATTR_ADDRASSIGN: {
        /*
         * ENTDAA for up to DEV_COUNT devices.  Each round restarts the
         * broadcast to find a device that still has no address, reads its
         * PID, BCR and DCR, and sends the dynamic address it is to take.
         * The DCT receives one entry per device, from entry 0 on.
         */
        uint8_t dev_count = FIELD_EX32(w0, CMD0, DEV_COUNT);
        uint8_t assigned = 0;
        uint32_t status;
        uint8_t i;

        if (i3c_start_transfer(s->bus, I3C_BROADCAST, false) != 0 ||
            i3c_send_byte(s->bus, I3C_CCC_ENTDAA) != 0) {
            ok = false;
            goto entdaa_done;
        }

        trace_mipi_i3c_hci_entdaa_start(s->id, dev_idx, dev_count);
        for (i = 0; i < dev_count; i++) {
            uint8_t pbcr_dcr[I3C_ENTDAA_SIZE] = {};
            uint32_t num_read = 0;
            uint64_t pid;
            uint8_t da;

            da = klass->entdaa_next_da(s, dev_idx, i);
            trace_mipi_i3c_hci_entdaa_next(s->id, i, da);
            /* 0xff: no address left; 0 and 0x7e and up are not valid. */
            if (da == 0xff || da == 0 || da >= I3C_BROADCAST) {
                break;
            }
            if (i3c_start_transfer(s->bus, I3C_BROADCAST, false) != 0) {
                /* Every device has an address. */
                trace_mipi_i3c_hci_entdaa_done(s->id, i);
                break;
            }
            if (i3c_recv(s->bus, pbcr_dcr, I3C_ENTDAA_SIZE, &num_read) != 0 ||
                num_read != I3C_ENTDAA_SIZE ||
                i3c_send_byte(s->bus, da) != 0) {
                ok = false;
                break;
            }

            /* The 48-bit PID comes first, most significant byte first. */
            pid = ldq_be_p(pbcr_dcr) >> 16;
            s->dct[assigned * 4] = pid >> 16;
            s->dct[assigned * 4 + 1] = pid & 0xffff;
            s->dct[assigned * 4 + 2] = (pbcr_dcr[6] << 8) | pbcr_dcr[7];
            s->dct[assigned * 4 + 3] = da;
            assigned++;
        }

entdaa_done:
        i3c_end_transfer(s->bus);
        /*
         * Only a command that assigned every address it was asked to
         * succeeds.  One that ran out of devices completes with
         * ADDR_HEADER and the number of addresses left over, which tells
         * software that enumeration is over.
         */
        status = ok && assigned == dev_count ?
                 MIPI_I3C_HCI_RESP_SUCCESS : MIPI_I3C_HCI_RESP_ERR_ADDR_HEADER;
        resp = mipi_i3c_hci_resp(status, tid, dev_count - assigned);
        actual_len = assigned;
        completed = true;
        break;
    }

    case CMD_ATTR_CCC: {
        uint8_t ccc_id = FIELD_EX32(w0, CMD0, CCC_CODE);
        uint8_t ccc_buf[MIPI_I3C_HCI_TX_BUF_SIZE];
        uint32_t buf_len = MIN(data_len, sizeof(ccc_buf));
        bool bcst = dev_idx >= 0x3f;

        trace_mipi_i3c_hci_ccc(s->id, ccc_id, rnw, dev_idx, bcst, data_len);

        if (!rnw && s->tx_len > 0) {
            memcpy(ccc_buf, s->tx_buf, MIN(s->tx_len, buf_len));
        } else {
            memset(ccc_buf, 0, buf_len);
        }
        s->tx_len = 0;

        if (bcst) {
            /*
             * Targets observe a broadcast CCC through i3c_scan_bus(); only
             * the controller's own state is updated here.
             */
            if (ccc_id == I3C_CCC_RSTDAA) {
                memset(s->dat, 0, sizeof(s->dat));
                memset(s->dct, 0, sizeof(s->dct));
                trace_mipi_i3c_hci_rstdaa(s->id);
            }
        } else {
            uint8_t da = klass->dat_to_da(s, dev_idx);
            uint32_t got = 0;

            ok = mipi_i3c_hci_ccc_xfer(s, ccc_id, rnw, da, ccc_buf, buf_len,
                                       &got);
            if (ok && rnw) {
                buf_len = got;
            }
            if (rnw && ok && buf_len > 0) {
                mipi_i3c_hci_push_rx(s, ccc_buf, buf_len);
                mipi_i3c_hci_check_thresholds(s);
                actual_len = buf_len;
            }
        }
        resp = mipi_i3c_hci_resp(ok ? MIPI_I3C_HCI_RESP_SUCCESS :
                                 MIPI_I3C_HCI_RESP_ERR_NACK, tid, actual_len);
        completed = true;
        break;
    }

    case CMD_ATTR_INTERNAL:
        roc = false;
        completed = false;
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: i3c%u reserved command attribute 0x%x "
                      "(w0 0x%08x w1 0x%08x)\n",
                      __func__, s->id, attr, w0, w1);
        return;
    }

    trace_mipi_i3c_hci_cmd_done(s->id, attr, tid, ok, actual_len);

    if (roc) {
        mipi_i3c_hci_push_resp(s, resp);
    }
    if (completed && klass->post_cmd) {
        klass->post_cmd(s, ok, rnw);
    }
    mipi_i3c_hci_update_irq(s);
}

/* Reset everything but the interrupt line. */
static void mipi_i3c_hci_reset_state(MIPII3CHCIState *s)
{
    MIPII3CHCIClass *klass = MIPI_I3C_HCI_GET_CLASS(s);

    memset(s->hci_regs, 0, sizeof(s->hci_regs));
    s->hci_regs[R_HCI_VERSION] = klass->version;
    s->hci_regs[R_HC_CAPABILITIES] = klass->capabilities;
    s->hci_regs[R_DAT_SECTION] =
        FIELD_DP32(klass->dat_offset, DAT_SECTION, TABLE_SIZE,
                   mipi_i3c_hci_table_size(klass, klass->dat_entries, 2));
    s->hci_regs[R_DCT_SECTION] =
        FIELD_DP32(klass->dct_offset, DCT_SECTION, TABLE_SIZE,
                   mipi_i3c_hci_table_size(klass, klass->dct_entries, 4));
    s->hci_regs[R_PIO_SECTION] = klass->pio_offset;
    s->hci_regs[R_EXT_CAPS_SECTION] = klass->ext_caps_offset;
    /* No DMA ring support: RING_HEADERS_SECTION stays 0. */

    memset(s->pio_regs, 0, sizeof(s->pio_regs));
    s->pio_regs[R_PIO_QUEUE_SIZE] = PIO_QUEUE_SIZE_RESET;
    s->pio_regs[R_PIO_INTR_STATUS] = R_PIO_INTR_STATUS_CMD_QUEUE_READY_MASK;
    s->resp_rd = s->resp_wr = 0;
    s->rx_rd = s->rx_wr = 0;
    s->ibi_rd = s->ibi_wr = 0;
    s->ibi_active = false;
    s->tx_len = 0;
    s->cmd_have_word0 = false;
    s->cmd_word0 = 0;

    memset(s->dat, 0, sizeof(s->dat));
    memset(s->dct, 0, sizeof(s->dct));

    if (klass->ext_reset) {
        klass->ext_reset(s);
    }
}

typedef enum MIPII3CHCIBlock {
    MIPI_I3C_HCI_BLOCK_HCI,
    MIPI_I3C_HCI_BLOCK_PIO,
    MIPI_I3C_HCI_BLOCK_DAT,
    MIPI_I3C_HCI_BLOCK_DCT,
    MIPI_I3C_HCI_BLOCK_EXT_CAPS,
    MIPI_I3C_HCI_BLOCK_VENDOR,
} MIPII3CHCIBlock;

static bool mipi_i3c_hci_in_block(hwaddr offset, uint32_t base, uint32_t size,
                                  hwaddr *rel)
{
    if (offset < base || offset - base >= size) {
        return false;
    }
    *rel = offset - base;
    return true;
}

/* The register block @offset falls in, and in @rel the offset within it. */
static MIPII3CHCIBlock mipi_i3c_hci_decode(MIPII3CHCIState *s, hwaddr offset,
                                           hwaddr *rel)
{
    MIPII3CHCIClass *klass = MIPI_I3C_HCI_GET_CLASS(s);

    if (mipi_i3c_hci_in_block(offset, 0, HCI_REGS_SIZE, rel)) {
        return MIPI_I3C_HCI_BLOCK_HCI;
    }
    if (mipi_i3c_hci_in_block(offset, klass->pio_offset, sizeof(s->pio_regs),
                              rel)) {
        return MIPI_I3C_HCI_BLOCK_PIO;
    }
    if (mipi_i3c_hci_in_block(offset, klass->dat_offset,
                              klass->dat_entries * 8, rel)) {
        return MIPI_I3C_HCI_BLOCK_DAT;
    }
    if (mipi_i3c_hci_in_block(offset, klass->dct_offset,
                              klass->dct_entries * 16, rel)) {
        return MIPI_I3C_HCI_BLOCK_DCT;
    }
    if (mipi_i3c_hci_in_block(offset, klass->ext_caps_offset,
                              MIPI_I3C_HCI_EXT_CAPS_SIZE, rel)) {
        return MIPI_I3C_HCI_BLOCK_EXT_CAPS;
    }
    *rel = offset;
    return MIPI_I3C_HCI_BLOCK_VENDOR;
}

static uint32_t mipi_i3c_hci_read_pio(MIPII3CHCIState *s, hwaddr pio_off)
{
    uint32_t val = 0;

    switch (pio_off) {
    case A_PIO_CMD_QUEUE_PORT:
        break;
    case A_PIO_RESP_QUEUE_PORT:
        val = mipi_i3c_hci_pop_resp(s);
        break;
    case A_PIO_XFER_DATA_PORT:
        if (s->rx_rd != s->rx_wr) {
            val = s->rx_fifo[s->rx_rd];
            s->rx_rd = (s->rx_rd + 1) % MIPI_I3C_HCI_RX_FIFO_DWORDS;
        }
        break;
    case A_PIO_IBI_PORT:
        if (s->ibi_rd != s->ibi_wr) {
            val = s->ibi_fifo[s->ibi_rd];
            s->ibi_rd = (s->ibi_rd + 1) % MIPI_I3C_HCI_IBI_FIFO_SIZE;
            if (s->ibi_rd == s->ibi_wr) {
                s->pio_regs[R_PIO_INTR_STATUS] &=
                    ~R_PIO_INTR_STATUS_IBI_STATUS_THLD_MASK;
            }
        }
        break;
    case A_PIO_QUEUE_SIZE:
        val = PIO_QUEUE_SIZE_RESET;
        break;
    default:
        val = s->pio_regs[pio_off >> 2];
        break;
    }
    return val;
}

static uint64_t mipi_i3c_hci_read(void *opaque, hwaddr offset, unsigned size)
{
    MIPII3CHCIState *s = MIPI_I3C_HCI(opaque);
    MIPII3CHCIClass *klass = MIPI_I3C_HCI_GET_CLASS(s);
    uint32_t val = 0;
    hwaddr rel;

    switch (mipi_i3c_hci_decode(s, offset, &rel)) {
    case MIPI_I3C_HCI_BLOCK_HCI:
        val = s->hci_regs[rel >> 2];
        if (rel == A_PRESENT_STATE) {
            val |= R_PRESENT_STATE_CURRENT_MASTER_MASK;
        }
        break;
    case MIPI_I3C_HCI_BLOCK_PIO:
        val = mipi_i3c_hci_read_pio(s, rel);
        break;
    case MIPI_I3C_HCI_BLOCK_DAT:
        val = s->dat[rel >> 2];
        break;
    case MIPI_I3C_HCI_BLOCK_DCT:
        val = s->dct[rel >> 2];
        break;
    case MIPI_I3C_HCI_BLOCK_EXT_CAPS:
        if (klass->get_extcaps) {
            size_t nwords = 0;
            const uint32_t *extcaps = klass->get_extcaps(s, &nwords);

            if (extcaps && (rel >> 2) < nwords) {
                val = extcaps[rel >> 2];
            }
        }
        break;
    case MIPI_I3C_HCI_BLOCK_VENDOR:
        if (klass->ext_read) {
            val = klass->ext_read(s, offset, size);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: i3c%u invalid read at 0x%" HWADDR_PRIx "\n",
                          __func__, s->id, offset);
        }
        break;
    }

    trace_mipi_i3c_hci_read(s->id, offset, val);
    return val;
}

static void mipi_i3c_hci_write_hci(MIPII3CHCIState *s, hwaddr offset,
                                   uint32_t val)
{
    MIPII3CHCIClass *klass = MIPI_I3C_HCI_GET_CLASS(s);

    switch (offset) {
    case A_HCI_VERSION:
    case A_HC_CAPABILITIES:
    case A_PRESENT_STATE:
    case A_DAT_SECTION:
    case A_RING_HEADERS_SECTION:
    case A_PIO_SECTION:
    case A_EXT_CAPS_SECTION:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: i3c%u write to read-only register 0x%" HWADDR_PRIx
                      "\n", __func__, s->id, offset);
        break;

    case A_RESET_CONTROL:
        if (FIELD_EX32(val, RESET_CONTROL, SOFT_RST)) {
            mipi_i3c_hci_reset_state(s);
            qemu_set_irq(s->irq, 0);
            break;
        }
        if (FIELD_EX32(val, RESET_CONTROL, CMD_QUEUE_RST)) {
            s->cmd_have_word0 = false;
            s->cmd_word0 = 0;
            s->pio_regs[R_PIO_INTR_STATUS] |=
                R_PIO_INTR_STATUS_CMD_QUEUE_READY_MASK;
        }
        if (FIELD_EX32(val, RESET_CONTROL, RESP_QUEUE_RST)) {
            s->resp_rd = s->resp_wr = 0;
            s->pio_regs[R_PIO_INTR_STATUS] &=
                ~R_PIO_INTR_STATUS_RESP_READY_MASK;
        }
        if (FIELD_EX32(val, RESET_CONTROL, TX_FIFO_RST)) {
            s->tx_len = 0;
            mipi_i3c_hci_check_thresholds(s);
        }
        if (FIELD_EX32(val, RESET_CONTROL, RX_FIFO_RST)) {
            s->rx_rd = s->rx_wr = 0;
            s->pio_regs[R_PIO_INTR_STATUS] &= ~R_PIO_INTR_STATUS_RX_THLD_MASK;
        }
        if (FIELD_EX32(val, RESET_CONTROL, IBI_QUEUE_RST)) {
            s->ibi_rd = s->ibi_wr = 0;
            s->pio_regs[R_PIO_INTR_STATUS] &=
                ~R_PIO_INTR_STATUS_IBI_STATUS_THLD_MASK;
        }
        if (val) {
            mipi_i3c_hci_update_irq(s);
        }
        break;

    case A_HC_CONTROL:
        s->hci_regs[R_HC_CONTROL] = val & ~R_HC_CONTROL_ABORT_MASK;
        if (FIELD_EX32(val, HC_CONTROL, ABORT)) {
            s->cmd_have_word0 = false;
            s->cmd_word0 = 0;
            s->tx_len = 0;
            s->resp_rd = s->resp_wr = 0;
            s->pio_regs[R_PIO_INTR_STATUS] &=
                ~R_PIO_INTR_STATUS_RESP_READY_MASK;
            if (klass->post_cmd) {
                klass->post_cmd(s, true, false);
            }
        }
        break;

    case A_INTR_STATUS:
        s->hci_regs[R_INTR_STATUS] &= ~val;
        break;

    case A_DCT_SECTION:
        /*
         * Only the DCT index is writable.  It is kept for software to read
         * back; ENTDAA fills the DCT from entry 0.
         */
        s->hci_regs[R_DCT_SECTION] =
            FIELD_DP32(s->hci_regs[R_DCT_SECTION], DCT_SECTION, TABLE_INDEX,
                       FIELD_EX32(val, DCT_SECTION, TABLE_INDEX));
        break;

    default:
        s->hci_regs[offset >> 2] = val;
        break;
    }
}

static void mipi_i3c_hci_write_pio(MIPII3CHCIState *s, hwaddr pio_off,
                                   uint32_t val)
{
    switch (pio_off) {
    case A_PIO_CMD_QUEUE_PORT:
        if (!s->cmd_have_word0) {
            s->cmd_word0 = val;
            s->cmd_have_word0 = true;
        } else {
            uint32_t w0 = s->cmd_word0;

            s->cmd_have_word0 = false;
            s->cmd_word0 = 0;
            mipi_i3c_hci_process_cmd(s, w0, val);
        }
        break;

    case A_PIO_RESP_QUEUE_PORT:
    case A_PIO_QUEUE_SIZE:
    case A_PIO_ALT_QUEUE_SIZE:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: i3c%u write to read-only PIO register 0x%"
                      HWADDR_PRIx "\n", __func__, s->id, pio_off);
        break;

    case A_PIO_XFER_DATA_PORT:
        if (s->tx_len + 4 <= MIPI_I3C_HCI_TX_BUF_SIZE) {
            memcpy(s->tx_buf + s->tx_len, &val, 4);
            s->tx_len += 4;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: i3c%u TX buffer overflow\n",
                          __func__, s->id);
            mipi_i3c_hci_set_transfer_err(s);
        }
        break;

    case A_PIO_INTR_STATUS:
        s->pio_regs[R_PIO_INTR_STATUS] &= ~val;
        mipi_i3c_hci_update_irq(s);
        break;

    case A_PIO_INTR_SIGNAL_ENABLE:
        /*
         * The interrupt is level-triggered: enabling a signal whose status
         * bit is already set raises it at once.  A driver may queue its
         * first command before it enables the signals.
         */
        s->pio_regs[R_PIO_INTR_SIGNAL_ENABLE] = val;
        mipi_i3c_hci_update_irq(s);
        break;

    case A_PIO_INTR_FORCE:
        s->pio_regs[R_PIO_INTR_STATUS] |= val;
        mipi_i3c_hci_update_irq(s);
        break;

    default:
        s->pio_regs[pio_off >> 2] = val;
        break;
    }
}

static void mipi_i3c_hci_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    MIPII3CHCIState *s = MIPI_I3C_HCI(opaque);
    MIPII3CHCIClass *klass = MIPI_I3C_HCI_GET_CLASS(s);
    uint32_t val = value;
    hwaddr rel;

    trace_mipi_i3c_hci_write(s->id, offset, val);

    switch (mipi_i3c_hci_decode(s, offset, &rel)) {
    case MIPI_I3C_HCI_BLOCK_HCI:
        mipi_i3c_hci_write_hci(s, rel, val);
        break;
    case MIPI_I3C_HCI_BLOCK_PIO:
        mipi_i3c_hci_write_pio(s, rel, val);
        break;
    case MIPI_I3C_HCI_BLOCK_DAT:
        /* A word 0 write without a static address keeps the old one. */
        if (!(rel & 4) && !FIELD_EX32(val, DAT_W0, STATIC_ADDR)) {
            val |= s->dat[rel >> 2] & R_DAT_W0_STATIC_ADDR_MASK;
        }
        s->dat[rel >> 2] = val;
        break;
    case MIPI_I3C_HCI_BLOCK_DCT:
        s->dct[rel >> 2] = val;
        break;
    case MIPI_I3C_HCI_BLOCK_EXT_CAPS:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: i3c%u write to read-only extended capability at 0x%"
                      HWADDR_PRIx "\n", __func__, s->id, offset);
        break;
    case MIPI_I3C_HCI_BLOCK_VENDOR:
        if (klass->ext_write) {
            klass->ext_write(s, offset, value, size);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: i3c%u invalid write at 0x%" HWADDR_PRIx
                          " (value 0x%08x)\n", __func__, s->id, offset, val);
        }
        break;
    }
}

static const MemoryRegionOps mipi_i3c_hci_mr_ops = {
    .read = mipi_i3c_hci_read,
    .write = mipi_i3c_hci_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mipi_i3c_hci_reset_enter(Object *obj, ResetType type)
{
    mipi_i3c_hci_reset_state(MIPI_I3C_HCI(obj));
}

static void mipi_i3c_hci_reset_hold(Object *obj, ResetType type)
{
    MIPII3CHCIState *s = MIPI_I3C_HCI(obj);

    qemu_set_irq(s->irq, 0);
}

static void mipi_i3c_hci_realize(DeviceState *dev, Error **errp)
{
    MIPII3CHCIState *s = MIPI_I3C_HCI(dev);
    MIPII3CHCIClass *klass = MIPI_I3C_HCI_GET_CLASS(s);
    g_autofree char *name = g_strdup_printf(TYPE_MIPI_I3C_HCI ".%d", s->id);

    if (klass->dat_entries > MIPI_I3C_HCI_MAX_DAT_ENTRIES ||
        klass->dct_entries > MIPI_I3C_HCI_MAX_DCT_ENTRIES) {
        error_setg(errp, "%s: DAT or DCT larger than the model supports",
                   object_get_typename(OBJECT(dev)));
        return;
    }

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    memory_region_init_io(&s->mr, OBJECT(s), &mipi_i3c_hci_mr_ops, s, name,
                          klass->mmio_size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mr);

    s->bus = i3c_init_bus_type(TYPE_MIPI_I3C_HCI_BUS, DEVICE(s), name);
}

I3CBus *mipi_i3c_hci_get_bus(MIPII3CHCIState *s)
{
    return s->bus;
}

/* Whether an IBI from @da is taken; 0 is not a target's address. */
static bool mipi_i3c_hci_ibi_allowed(MIPII3CHCIState *s, uint8_t da)
{
    MIPII3CHCIClass *klass = MIPI_I3C_HCI_GET_CLASS(s);
    int idx;

    if (!da) {
        return true;
    }
    idx = klass->da_to_dat(s, da);
    if (idx < 0) {
        trace_mipi_i3c_hci_ibi_suppress(s->id, da, "no DAT entry");
        return false;
    }
    if (FIELD_EX32(s->dat[idx * 2], DAT_W0, SIR_REJECT)) {
        trace_mipi_i3c_hci_ibi_suppress(s->id, da, "SIR_REJECT set");
        return false;
    }
    return true;
}

bool mipi_i3c_hci_inject_ibi(MIPII3CHCIState *s, uint8_t da,
                              uint8_t ibi_byte, bool has_byte)
{
    MIPII3CHCIClass *klass = MIPI_I3C_HCI_GET_CLASS(s);
    uint32_t desc = 0;

    if (!mipi_i3c_hci_ibi_allowed(s, da)) {
        return false;
    }

    desc = FIELD_DP32(desc, IBI_STATUS, LAST_STATUS, 1);
    desc = FIELD_DP32(desc, IBI_STATUS, IBI_ID, da ? (da << 1) | 1 : 0);
    desc = FIELD_DP32(desc, IBI_STATUS, DATA_LEN, has_byte);

    trace_mipi_i3c_hci_ibi(s->id, da, ibi_byte, has_byte, desc);

    mipi_i3c_hci_push_ibi(s, desc);
    if (has_byte) {
        mipi_i3c_hci_push_ibi(s, ibi_byte);
    }

    s->pio_regs[R_PIO_INTR_STATUS] |= R_PIO_INTR_STATUS_IBI_STATUS_THLD_MASK;
    mipi_i3c_hci_update_irq(s);

    if (klass->post_ibi) {
        klass->post_ibi(s, da);
    }
    return true;
}

static int mipi_i3c_hci_default_da_to_dat(MIPII3CHCIState *s, uint8_t da)
{
    uint32_t i;

    for (i = 0; i < mipi_i3c_hci_dat_entries(s); i++) {
        uint32_t w0 = s->dat[i * 2];

        if (!FIELD_EX32(w0, DAT_W0, I2C_DEVICE) &&
            FIELD_EX32(w0, DAT_W0, DYNAMIC_ADDR) == da) {
            return i;
        }
    }
    return -1;
}

static MIPII3CHCIState *mipi_i3c_hci_from_bus(I3CBus *bus)
{
    return MIPI_I3C_HCI(BUS(bus)->parent);
}

/*
 * IBIs from the targets on the bus.  A target interrupt is ACKed unless the
 * DAT entry of its address rejects it.  Its first data byte is kept as the
 * mandatory data byte and any further payload is dropped.  Hot-join and
 * controller role requests are NACKed.
 */
static int mipi_i3c_hci_bus_ibi_handle(I3CBus *bus, uint8_t addr,
                                       bool is_recv)
{
    MIPII3CHCIState *s = mipi_i3c_hci_from_bus(bus);

    if (!is_recv || addr == I3C_HJ_ADDR) {
        qemu_log_mask(LOG_UNIMP, "%s: i3c%u %s request from 0x%02x NACKed\n",
                      __func__, s->id,
                      addr == I3C_HJ_ADDR ? "hot-join" : "controller role",
                      addr);
        return -1;
    }
    if (!mipi_i3c_hci_ibi_allowed(s, addr)) {
        return -1;
    }
    s->ibi_active = true;
    s->ibi_da = addr;
    s->ibi_has_mdb = false;
    return 0;
}

static int mipi_i3c_hci_bus_ibi_recv(I3CBus *bus, uint8_t data)
{
    MIPII3CHCIState *s = mipi_i3c_hci_from_bus(bus);

    if (!s->ibi_active) {
        return -1;
    }
    if (!s->ibi_has_mdb) {
        s->ibi_mdb = data;
        s->ibi_has_mdb = true;
    } else {
        qemu_log_mask(LOG_UNIMP,
                      "%s: i3c%u IBI payload after the first byte dropped\n",
                      __func__, s->id);
    }
    return 0;
}

static int mipi_i3c_hci_bus_ibi_finish(I3CBus *bus)
{
    MIPII3CHCIState *s = mipi_i3c_hci_from_bus(bus);

    if (s->ibi_active) {
        s->ibi_active = false;
        mipi_i3c_hci_inject_ibi(s, s->ibi_da, s->ibi_mdb, s->ibi_has_mdb);
    }
    return 0;
}

static uint8_t mipi_i3c_hci_default_dat_to_da(MIPII3CHCIState *s,
                                               uint8_t dev_idx)
{
    if (dev_idx >= mipi_i3c_hci_dat_entries(s)) {
        return 0;
    }
    return FIELD_EX32(s->dat[dev_idx * 2], DAT_W0, DYNAMIC_ADDR);
}

static uint8_t mipi_i3c_hci_default_entdaa_next_da(MIPII3CHCIState *s,
                                                    uint8_t dev_idx_base,
                                                    uint8_t iter)
{
    uint32_t slot = dev_idx_base + iter;

    if (slot >= mipi_i3c_hci_dat_entries(s)) {
        return 0xff;
    }
    return FIELD_EX32(s->dat[slot * 2], DAT_W0, DYNAMIC_ADDR);
}

const VMStateDescription vmstate_mipi_i3c_hci = {
    .name = TYPE_MIPI_I3C_HCI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(hci_regs, MIPII3CHCIState,
                             MIPI_I3C_HCI_HCI_NR_REGS),
        VMSTATE_UINT32_ARRAY(pio_regs, MIPII3CHCIState,
                             MIPI_I3C_HCI_PIO_NR_REGS),
        VMSTATE_UINT32_ARRAY(dat, MIPII3CHCIState, MIPI_I3C_HCI_DAT_NR_REGS),
        VMSTATE_UINT32_ARRAY(dct, MIPII3CHCIState, MIPI_I3C_HCI_DCT_NR_REGS),
        VMSTATE_UINT32(cmd_word0, MIPII3CHCIState),
        VMSTATE_BOOL(cmd_have_word0, MIPII3CHCIState),
        VMSTATE_UINT32_ARRAY(resp_fifo, MIPII3CHCIState,
                             MIPI_I3C_HCI_RESP_FIFO_SIZE),
        VMSTATE_UINT32(resp_rd, MIPII3CHCIState),
        VMSTATE_UINT32(resp_wr, MIPII3CHCIState),
        VMSTATE_UINT32_ARRAY(rx_fifo, MIPII3CHCIState,
                             MIPI_I3C_HCI_RX_FIFO_DWORDS),
        VMSTATE_UINT32(rx_rd, MIPII3CHCIState),
        VMSTATE_UINT32(rx_wr, MIPII3CHCIState),
        VMSTATE_UINT32_ARRAY(ibi_fifo, MIPII3CHCIState,
                             MIPI_I3C_HCI_IBI_FIFO_SIZE),
        VMSTATE_UINT32(ibi_rd, MIPII3CHCIState),
        VMSTATE_UINT32(ibi_wr, MIPII3CHCIState),
        VMSTATE_UINT8_ARRAY(tx_buf, MIPII3CHCIState,
                            MIPI_I3C_HCI_TX_BUF_SIZE),
        VMSTATE_UINT32(tx_len, MIPII3CHCIState),
        VMSTATE_END_OF_LIST(),
    },
};

static const Property mipi_i3c_hci_props[] = {
    DEFINE_PROP_UINT8("ctrl-id", MIPII3CHCIState, id, 0),
};

static void mipi_i3c_hci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    MIPII3CHCIClass *mc = MIPI_I3C_HCI_CLASS(klass);

    dc->desc = "MIPI I3C Host Controller Interface";
    /* Machines with a platform bus can create the generic controller. */
    dc->user_creatable = true;
    dc->realize = mipi_i3c_hci_realize;
    dc->vmsd = &vmstate_mipi_i3c_hci;
    rc->phases.enter = mipi_i3c_hci_reset_enter;
    rc->phases.hold = mipi_i3c_hci_reset_hold;
    device_class_set_props(dc, mipi_i3c_hci_props);

    mc->mmio_size = MIPI_I3C_HCI_DEFAULT_MMIO_SIZE;
    mc->version = MIPI_I3C_HCI_DEFAULT_VERSION;
    mc->capabilities = MIPI_I3C_HCI_DEFAULT_CAPABILITIES;
    mc->pio_offset = MIPI_I3C_HCI_DEFAULT_PIO_OFFSET;
    mc->dat_offset = MIPI_I3C_HCI_DEFAULT_DAT_OFFSET;
    mc->dat_entries = MIPI_I3C_HCI_DEFAULT_DAT_ENTRIES;
    mc->dct_offset = MIPI_I3C_HCI_DEFAULT_DCT_OFFSET;
    mc->dct_entries = MIPI_I3C_HCI_DEFAULT_DCT_ENTRIES;
    mc->ext_caps_offset = MIPI_I3C_HCI_DEFAULT_EXT_CAPS_OFFSET;
    mc->dat_to_da = mipi_i3c_hci_default_dat_to_da;
    mc->da_to_dat = mipi_i3c_hci_default_da_to_dat;
    mc->entdaa_next_da = mipi_i3c_hci_default_entdaa_next_da;
}

static void mipi_i3c_hci_bus_class_init(ObjectClass *klass, const void *data)
{
    I3CBusClass *bc = I3C_BUS_CLASS(klass);

    bc->ibi_handle = mipi_i3c_hci_bus_ibi_handle;
    bc->ibi_recv = mipi_i3c_hci_bus_ibi_recv;
    bc->ibi_finish = mipi_i3c_hci_bus_ibi_finish;
}

static const TypeInfo mipi_i3c_hci_types[] = {
    {
        .name = TYPE_MIPI_I3C_HCI_BUS,
        .parent = TYPE_I3C_BUS,
        .class_init = mipi_i3c_hci_bus_class_init,
    },
    {
        .name = TYPE_MIPI_I3C_HCI,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MIPII3CHCIState),
        .class_size = sizeof(MIPII3CHCIClass),
        .class_init = mipi_i3c_hci_class_init,
    },
};

DEFINE_TYPES(mipi_i3c_hci_types)
