/*
 * QTest testcase for the ASPEED AST2700 MIPI I3C HCI controller
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/host-utils.h"
#include "libqtest.h"

/* i3c0; each of the 16 controllers takes 4KB */
#define AST2700_I3C_HCI_BASE    0x14c20000
#define I3C_HCI_CTRL_SIZE       0x1000

#define HCI_REG_VERSION         0x000
#define HCI_REG_HC_CONTROL      0x004
#define HCI_REG_HC_CAPABILITIES 0x00c
#define HCI_REG_RESET_CONTROL   0x010
#define HCI_REG_PRESENT_STATE   0x014
#define HCI_REG_DAT_SECTION     0x030
#define HCI_REG_DCT_SECTION     0x034
#define HCI_REG_RING_HDR_SEC    0x038
#define HCI_REG_PIO_SECTION     0x03c
#define HCI_REG_EXT_CAPS_SEC    0x040

#define PRESENT_STATE_CURRENT_MASTER  (1u << 2)

/* HCI 1.1: DAT of 127 entries at 0x100, DCT of 16 entries at 0x500 */
#define HCI_VERSION_RESET       0x00000110u
#define HC_CAPABILITIES_RESET   0x00000008u
#define DAT_SECTION_RESET       0x0007f100u
#define DCT_SECTION_RESET       0x00010500u
#define PIO_SECTION_RESET       0x000000d0u
#define EXT_CAPS_SECTION_RESET  0x00000f00u
#define RING_HDR_SECTION_RESET  0x00000000u
#define DCT_TABLE_INDEX_SHIFT   19

#define RESET_CTRL_SOFT_RST       (1u << 0)
#define RESET_CTRL_CMD_QUEUE_RST  (1u << 1)
#define RESET_CTRL_RESP_QUEUE_RST (1u << 2)

#define PIO_BASE                0x0d0
#define PIO_REG_CMD_QUEUE       (PIO_BASE + 0x000)
#define PIO_REG_RESP_QUEUE      (PIO_BASE + 0x004)
#define PIO_REG_XFER_DATA       (PIO_BASE + 0x008)
#define PIO_REG_IBI_PORT        (PIO_BASE + 0x00c)
#define PIO_REG_QUEUE_SIZE      (PIO_BASE + 0x018)
#define PIO_REG_INTR_STATUS     (PIO_BASE + 0x020)

#define PIO_QUEUE_SIZE_RESET    0x06052008u

#define PIO_INTR_IBI_THLD       (1u << 2)
#define PIO_INTR_CMD_QUEUE_RDY  (1u << 3)
#define PIO_INTR_RESP_READY     (1u << 4)

/* Device Address and Device Characteristics Tables */
#define DAT_BASE                0x100
#define DAT_W0_SIR_REJECT       (1u << 13)
#define DCT_BASE                0x500

/* ASPEED INHOUSE registers holding the ENTDAA address bitmap */
#define INHOUSE_DAA_INDEX0      0xd10

#define CMD_ATTR_REGULAR        0x0u
#define CMD_ATTR_IMMEDIATE      0x1u
#define CMD_ATTR_ADDRASSIGN     0x2u
#define CMD_ATTR_CCC            0x3u
#define CMD0_TOC                (1u << 31)
#define CMD0_ROC                (1u << 30)
#define CMD0_RNW                (1u << 29)
#define CMD0_DEV_COUNT_SHIFT    26
#define CMD0_IMM_DTT_SHIFT      23
#define CMD0_DEV_INDEX_SHIFT    16
#define CMD0_CCC_CODE_SHIFT     7
#define CMD0_TID_SHIFT          3
#define CMD1_DATA_LEN_SHIFT     16

#define RESP_STATUS_SHIFT       28
#define RESP_TID_SHIFT          24
#define RESP_DATA_LEN_MASK      0xffffu
#define RESP_SUCCESS            0x0u
#define RESP_ERR_ADDR_HEADER    0x4u
#define RESP_ERR_NACK           0x5u

#define IBI_STATUS_LAST         (1u << 24)
#define IBI_STATUS_ID_SHIFT     8

#define CCC_ENEC_DIRECT         0x80
#define CCC_SETNEWDA            0x88
#define CCC_GETPID              0x8d
#define CCC_GETBCR              0x8e
#define ENEC_ENINT              0x01

#define I3C_HCI_CMD_QUEUE_SIZE  7

/* Commands may complete asynchronously, so completions are polled for */
#define POLL_TIMEOUT_US (5 * 1000 * 1000)

/* Two mock targets on i3c0 */
#define TGT_A_PID               0x0123456789abULL
#define TGT_A_BCR               0x02
#define TGT_A_DCR               0xa5
#define TGT_B_PID               0x0a0b0c0d0e0fULL
#define TGT_B_BCR               0x04
#define TGT_B_DCR               0x3c
/* Target A raises an IBI when it receives this byte */
#define TGT_A_IBI_MAGIC         0x5a
/* Dynamic addresses handed out, lowest first, by ENTDAA */
#define DA_FIRST                0x09
#define DA_SECOND               0x0a
/* Addresses whose DAT entries lie beyond the first 16 */
#define DA_HIGH                 0x30
/* Address a target is moved to with SETNEWDA */
#define DA_NEW                  0x20

static uint32_t i3c_readl(QTestState *s, int ctrl, uint32_t off)
{
    return qtest_readl(s, AST2700_I3C_HCI_BASE + ctrl * I3C_HCI_CTRL_SIZE +
                       off);
}

static void i3c_writel(QTestState *s, int ctrl, uint32_t off, uint32_t val)
{
    qtest_writel(s, AST2700_I3C_HCI_BASE + ctrl * I3C_HCI_CTRL_SIZE + off,
                val);
}

static QTestState *i3c_hci_start(void)
{
    return qtest_init("-machine ast2700-evb");
}

/* @b_opts: extra properties for target B */
static QTestState *i3c_hci_start_with_targets_opts(const char *b_opts)
{
    return qtest_initf("-machine ast2700-evb "
                       "-device mock-i3c-target,bus=mipi-i3c-hci.0,"
                       "pid=0x%" PRIx64 ",bcr=0x%x,dcr=0x%x,"
                       "ibi-magic-num=0x%x "
                       "-device mock-i3c-target,bus=mipi-i3c-hci.0,"
                       "pid=0x%" PRIx64 ",bcr=0x%x,dcr=0x%x%s",
                       (uint64_t)TGT_A_PID, TGT_A_BCR, TGT_A_DCR,
                       TGT_A_IBI_MAGIC,
                       (uint64_t)TGT_B_PID, TGT_B_BCR, TGT_B_DCR, b_opts);
}

static QTestState *i3c_hci_start_with_targets(void)
{
    return i3c_hci_start_with_targets_opts("");
}

/* Write @byte to the device at DAT index 0 */
static void i3c_queue_immediate_write(QTestState *s, int ctrl, uint8_t tid,
                                     uint8_t byte)
{
    uint32_t w0 = CMD_ATTR_IMMEDIATE | CMD0_TOC | CMD0_ROC |
                 (1u << CMD0_IMM_DTT_SHIFT) | ((uint32_t)tid << CMD0_TID_SHIFT);

    i3c_writel(s, ctrl, PIO_REG_CMD_QUEUE, w0);
    i3c_writel(s, ctrl, PIO_REG_CMD_QUEUE, byte);
}

/* Fail rather than hang if @bits never get set */
static void i3c_wait_intr(QTestState *s, int ctrl, uint32_t bits)
{
    int64_t end = g_get_monotonic_time() + POLL_TIMEOUT_US;

    while ((i3c_readl(s, ctrl, PIO_REG_INTR_STATUS) & bits) != bits) {
        g_assert_true(g_get_monotonic_time() < end);
        g_usleep(1000);
    }
}

/* Run a command with a response on i3c0 and return the response */
static uint32_t i3c_run_cmd(QTestState *s, uint32_t w0, uint32_t w1)
{
    i3c_writel(s, 0, PIO_REG_CMD_QUEUE, w0 | CMD0_TOC | CMD0_ROC);
    i3c_writel(s, 0, PIO_REG_CMD_QUEUE, w1);
    i3c_wait_intr(s, 0, PIO_INTR_RESP_READY);
    return i3c_readl(s, 0, PIO_REG_RESP_QUEUE);
}

static void i3c_assert_resp(uint32_t resp, uint32_t status, uint32_t tid,
                            uint32_t len)
{
    g_assert_cmphex((resp >> RESP_STATUS_SHIFT) & 0xf, ==, status);
    g_assert_cmphex((resp >> RESP_TID_SHIFT) & 0xf, ==, tid);
    g_assert_cmpuint(resp & RESP_DATA_LEN_MASK, ==, len);
}

/*
 * Assign the dynamic addresses @addrs to @count devices with ENTDAA, through
 * the DAA_INDEX0-3 bitmap.
 */
static uint32_t i3c_entdaa(QTestState *s, const uint8_t *addrs, int count)
{
    uint32_t bitmap[4] = {};
    int i;

    for (i = 0; i < count; i++) {
        bitmap[addrs[i] / 32] |= 1u << (addrs[i] % 32);
    }
    for (i = 0; i < 4; i++) {
        i3c_writel(s, 0, INHOUSE_DAA_INDEX0 + i * 4, bitmap[i]);
    }
    return i3c_run_cmd(s, CMD_ATTR_ADDRASSIGN | (1u << CMD0_TID_SHIFT) |
                       ((uint32_t)count << CMD0_DEV_COUNT_SHIFT), 0);
}

/* The dynamic address of the target that is not at @da */
static uint8_t i3c_other_da(uint8_t da)
{
    return da == DA_FIRST ? DA_SECOND : DA_FIRST;
}

/* Run a directed CCC read of @len bytes from @da; return the response */
static uint32_t i3c_ccc_read(QTestState *s, uint8_t tid, uint8_t ccc,
                             uint8_t da, uint32_t len)
{
    return i3c_run_cmd(s, CMD_ATTR_CCC | ((uint32_t)tid << CMD0_TID_SHIFT) |
                       ((uint32_t)ccc << CMD0_CCC_CODE_SHIFT) |
                       ((uint32_t)da << CMD0_DEV_INDEX_SHIFT) | CMD0_RNW,
                       len << CMD1_DATA_LEN_SHIFT);
}

/*
 * Enumerate both targets at @first and @first + 1 and return the dynamic
 * address of target A.
 */
static uint8_t i3c_enumerate_targets_at(QTestState *s, uint8_t first)
{
    const uint8_t addrs[] = { first, first + 1 };
    uint32_t resp = i3c_entdaa(s, addrs, 2);
    uint64_t pid;

    i3c_assert_resp(resp, RESP_SUCCESS, 1, 0);
    pid = ((uint64_t)i3c_readl(s, 0, DCT_BASE) << 16) |
          i3c_readl(s, 0, DCT_BASE + 4);
    return pid == TGT_A_PID ? first : first + 1;
}

static uint8_t i3c_enumerate_targets(QTestState *s)
{
    return i3c_enumerate_targets_at(s, DA_FIRST);
}

static void test_reset_values(void)
{
    QTestState *s = i3c_hci_start();

    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_VERSION), ==, HCI_VERSION_RESET);
    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_HC_CAPABILITIES), ==,
                    HC_CAPABILITIES_RESET);
    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_DAT_SECTION), ==,
                    DAT_SECTION_RESET);
    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_DCT_SECTION), ==,
                    DCT_SECTION_RESET);
    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_RING_HDR_SEC), ==,
                    RING_HDR_SECTION_RESET);
    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_PIO_SECTION), ==,
                    PIO_SECTION_RESET);
    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_EXT_CAPS_SEC), ==,
                    EXT_CAPS_SECTION_RESET);
    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_HC_CONTROL), ==, 0);
    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_PRESENT_STATE), ==,
                    PRESENT_STATE_CURRENT_MASTER);

    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_QUEUE_SIZE), ==,
                    PIO_QUEUE_SIZE_RESET);
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS), ==,
                    PIO_INTR_CMD_QUEUE_RDY);

    qtest_quit(s);
}

/* Software can set the DCT index; the rest of DCT_SECTION is read-only */
static void test_dct_index_writable(void)
{
    QTestState *s = i3c_hci_start();

    i3c_writel(s, 0, HCI_REG_DCT_SECTION, 0xffffffffu);
    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_DCT_SECTION), ==,
                    DCT_SECTION_RESET | (0x1fu << DCT_TABLE_INDEX_SHIFT));
    i3c_writel(s, 0, HCI_REG_DCT_SECTION, 0);
    g_assert_cmphex(i3c_readl(s, 0, HCI_REG_DCT_SECTION), ==,
                    DCT_SECTION_RESET);

    qtest_quit(s);
}

static void test_controllers_independent(void)
{
    QTestState *s = i3c_hci_start();

    i3c_queue_immediate_write(s, 3, 1, 0xaa);
    i3c_wait_intr(s, 3, PIO_INTR_RESP_READY);

    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS), ==,
                    PIO_INTR_CMD_QUEUE_RDY);
    g_assert_cmphex(i3c_readl(s, 5, PIO_REG_INTR_STATUS), ==,
                    PIO_INTR_CMD_QUEUE_RDY);

    qtest_quit(s);
}

/* No target is attached, so every command ends in a NACK */
static void test_command_completes(void)
{
    QTestState *s = i3c_hci_start();
    uint32_t resp;

    i3c_queue_immediate_write(s, 0, 7, 0x42);

    i3c_wait_intr(s, 0, PIO_INTR_RESP_READY);

    resp = i3c_readl(s, 0, PIO_REG_RESP_QUEUE);
    g_assert_cmphex((resp >> RESP_STATUS_SHIFT) & 0xf, ==, RESP_ERR_NACK);
    g_assert_cmphex((resp >> RESP_TID_SHIFT) & 0xf, ==, 7);

    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS) &
                    PIO_INTR_RESP_READY, ==, 0);

    qtest_quit(s);
}

/* A full queue must give back every response, in order */
static void test_cmd_queue_full_depth_drains_in_order(void)
{
    QTestState *s = i3c_hci_start();
    int i;

    for (i = 0; i < I3C_HCI_CMD_QUEUE_SIZE; i++) {
        i3c_queue_immediate_write(s, 0, i, 0x10 + i);
    }

    for (i = 0; i < I3C_HCI_CMD_QUEUE_SIZE; i++) {
        uint32_t resp;

        i3c_wait_intr(s, 0, PIO_INTR_RESP_READY);
        resp = i3c_readl(s, 0, PIO_REG_RESP_QUEUE);
        g_assert_cmphex((resp >> RESP_STATUS_SHIFT) & 0xf, ==, RESP_ERR_NACK);
        g_assert_cmphex((resp >> RESP_TID_SHIFT) & 0xf, ==, (uint32_t)i);
    }

    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS) &
                    PIO_INTR_CMD_QUEUE_RDY, ==, PIO_INTR_CMD_QUEUE_RDY);

    qtest_quit(s);
}

/*
 * Reset the queues while a command may still be running. Its response must
 * not show up afterwards, and the controller must keep working.
 */
static void test_cmd_queue_reset_stays_functional(void)
{
    QTestState *s = i3c_hci_start();
    uint32_t resp;

    i3c_queue_immediate_write(s, 0, 1, 0x55);
    i3c_writel(s, 0, HCI_REG_RESET_CONTROL,
              RESET_CTRL_CMD_QUEUE_RST | RESET_CTRL_RESP_QUEUE_RST);

    i3c_wait_intr(s, 0, PIO_INTR_CMD_QUEUE_RDY);
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS) &
                    PIO_INTR_RESP_READY, ==, 0);

    i3c_queue_immediate_write(s, 0, 2, 0x66);
    i3c_wait_intr(s, 0, PIO_INTR_RESP_READY);
    resp = i3c_readl(s, 0, PIO_REG_RESP_QUEUE);
    g_assert_cmphex((resp >> RESP_STATUS_SHIFT) & 0xf, ==, RESP_ERR_NACK);
    g_assert_cmphex((resp >> RESP_TID_SHIFT) & 0xf, ==, 2);

    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS) &
                    PIO_INTR_RESP_READY, ==, 0);

    qtest_quit(s);
}

static void test_soft_reset_restores_defaults(void)
{
    QTestState *s = i3c_hci_start();

    i3c_queue_immediate_write(s, 0, 3, 0x77);
    i3c_wait_intr(s, 0, PIO_INTR_RESP_READY);

    i3c_writel(s, 0, HCI_REG_RESET_CONTROL, RESET_CTRL_SOFT_RST);

    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS), ==,
                    PIO_INTR_CMD_QUEUE_RDY);
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_QUEUE_SIZE), ==,
                    PIO_QUEUE_SIZE_RESET);

    qtest_quit(s);
}

/*
 * ENTDAA hands the bitmap's addresses out in order and records each
 * device's PID, BCR, DCR and address in the DCT.  A further ENTDAA finds no
 * device left.
 */
static void test_entdaa_assigns_addresses(void)
{
    QTestState *s = i3c_hci_start_with_targets();
    bool seen_a = false, seen_b = false;
    uint32_t resp;
    int i;

    static const uint8_t addrs[] = { DA_FIRST, DA_SECOND };
    static const uint8_t spare = DA_SECOND + 1;

    resp = i3c_entdaa(s, addrs, 2);
    i3c_assert_resp(resp, RESP_SUCCESS, 1, 0);

    for (i = 0; i < 2; i++) {
        uint32_t base = DCT_BASE + i * 16;
        uint64_t pid = ((uint64_t)i3c_readl(s, 0, base) << 16) |
                       i3c_readl(s, 0, base + 4);
        uint32_t bcr_dcr = i3c_readl(s, 0, base + 8);

        g_assert_cmphex(i3c_readl(s, 0, base + 12), ==,
                        i ? DA_SECOND : DA_FIRST);
        if (pid == TGT_A_PID) {
            g_assert_cmphex(bcr_dcr, ==, (TGT_A_BCR << 8) | TGT_A_DCR);
            seen_a = true;
        } else {
            g_assert_cmphex(pid, ==, TGT_B_PID);
            g_assert_cmphex(bcr_dcr, ==, (TGT_B_BCR << 8) | TGT_B_DCR);
            seen_b = true;
        }
    }
    g_assert_true(seen_a && seen_b);

    /* Every device has an address now: one address is left over */
    resp = i3c_entdaa(s, &spare, 1);
    i3c_assert_resp(resp, RESP_ERR_ADDR_HEADER, 1, 1);

    qtest_quit(s);
}

/* Data written to a target with a private write reads back unchanged */
static void test_private_write_read(void)
{
    QTestState *s = i3c_hci_start_with_targets();
    static const uint8_t data[8] = {
        0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef
    };
    uint8_t da = i3c_enumerate_targets(s);
    uint8_t got[8];
    uint32_t dev = (uint32_t)da << CMD0_DEV_INDEX_SHIFT;
    uint32_t len = sizeof(data) << CMD1_DATA_LEN_SHIFT;
    uint32_t resp;

    i3c_writel(s, 0, PIO_REG_XFER_DATA, ldl_le_p(data));
    i3c_writel(s, 0, PIO_REG_XFER_DATA, ldl_le_p(data + 4));
    resp = i3c_run_cmd(s, CMD_ATTR_REGULAR | (2u << CMD0_TID_SHIFT) | dev,
                       len);
    i3c_assert_resp(resp, RESP_SUCCESS, 2, 0);

    resp = i3c_run_cmd(s, CMD_ATTR_REGULAR | (3u << CMD0_TID_SHIFT) | dev |
                       CMD0_RNW, len);
    i3c_assert_resp(resp, RESP_SUCCESS, 3, sizeof(data));
    stl_le_p(got, i3c_readl(s, 0, PIO_REG_XFER_DATA));
    stl_le_p(got + 4, i3c_readl(s, 0, PIO_REG_XFER_DATA));
    g_assert_cmpmem(got, sizeof(got), data, sizeof(data));

    qtest_quit(s);
}

/* A directed GETPID returns the target's PID, most significant byte first */
static void test_getpid(void)
{
    QTestState *s = i3c_hci_start_with_targets();
    uint8_t da = i3c_enumerate_targets(s);
    uint8_t got[8];
    uint32_t resp;
    int i;

    resp = i3c_run_cmd(s, CMD_ATTR_CCC | (4u << CMD0_TID_SHIFT) |
                       (CCC_GETPID << CMD0_CCC_CODE_SHIFT) |
                       ((uint32_t)da << CMD0_DEV_INDEX_SHIFT) | CMD0_RNW,
                       6u << CMD1_DATA_LEN_SHIFT);
    i3c_assert_resp(resp, RESP_SUCCESS, 4, 6);
    stl_le_p(got, i3c_readl(s, 0, PIO_REG_XFER_DATA));
    stl_le_p(got + 4, i3c_readl(s, 0, PIO_REG_XFER_DATA));
    for (i = 0; i < 6; i++) {
        g_assert_cmphex(got[i], ==, (TGT_A_PID >> (8 * (5 - i))) & 0xff);
    }

    qtest_quit(s);
}

/*
 * Enable target A's interrupts with a directed ENEC, then make it raise an
 * IBI, which shows up in the IBI queue unless its DAT entry rejects it.
 */
static void i3c_target_a_raise_ibi(QTestState *s, uint8_t da)
{
    uint32_t dev = (uint32_t)da << CMD0_DEV_INDEX_SHIFT;
    uint32_t resp;

    i3c_writel(s, 0, PIO_REG_XFER_DATA, ENEC_ENINT);
    resp = i3c_run_cmd(s, CMD_ATTR_CCC | (5u << CMD0_TID_SHIFT) |
                       (CCC_ENEC_DIRECT << CMD0_CCC_CODE_SHIFT) | dev,
                       1u << CMD1_DATA_LEN_SHIFT);
    i3c_assert_resp(resp, RESP_SUCCESS, 5, 0);

    resp = i3c_run_cmd(s, CMD_ATTR_IMMEDIATE | (6u << CMD0_TID_SHIFT) | dev |
                       (1u << CMD0_IMM_DTT_SHIFT), TGT_A_IBI_MAGIC);
    i3c_assert_resp(resp, RESP_SUCCESS, 6, 0);

    /* The target raises its IBI 1 ms after the magic byte */
    qtest_clock_step(s, 2 * 1000 * 1000);
}

static void test_ibi(void)
{
    QTestState *s = i3c_hci_start_with_targets();
    uint8_t da = i3c_enumerate_targets(s);

    i3c_target_a_raise_ibi(s, da);

    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS) & PIO_INTR_IBI_THLD,
                    ==, PIO_INTR_IBI_THLD);
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_IBI_PORT), ==,
                    IBI_STATUS_LAST |
                    (((uint32_t)da << 1 | 1) << IBI_STATUS_ID_SHIFT));
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS) & PIO_INTR_IBI_THLD,
                    ==, 0);

    qtest_quit(s);
}

static void test_ibi_sir_reject(void)
{
    QTestState *s = i3c_hci_start_with_targets();
    uint8_t da = i3c_enumerate_targets(s);

    i3c_writel(s, 0, DAT_BASE + da * 8, DAT_W0_SIR_REJECT);
    i3c_target_a_raise_ibi(s, da);

    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS) & PIO_INTR_IBI_THLD,
                    ==, 0);
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_IBI_PORT), ==, 0);

    qtest_quit(s);
}

/*
 * A directed CCC enrols every target for its broadcast part.  The targets
 * that drop out at the restart must still see the STOP, or the next CCC
 * addressed to them is taken for a continuation of this one.
 */
static void test_stop_reaches_every_target(void)
{
    QTestState *s = i3c_hci_start_with_targets();
    uint8_t da_a = i3c_enumerate_targets(s);
    uint32_t resp;

    resp = i3c_ccc_read(s, 7, CCC_GETPID, da_a, 6);
    i3c_assert_resp(resp, RESP_SUCCESS, 7, 6);
    i3c_readl(s, 0, PIO_REG_XFER_DATA);
    i3c_readl(s, 0, PIO_REG_XFER_DATA);

    resp = i3c_ccc_read(s, 8, CCC_GETBCR, i3c_other_da(da_a), 1);
    i3c_assert_resp(resp, RESP_SUCCESS, 8, 1);
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_XFER_DATA) & 0xff, ==,
                    TGT_B_BCR);

    qtest_quit(s);
}

/* SETNEWDA moves a target to the address in bits [7:1] of its data byte */
static void test_setnewda(void)
{
    QTestState *s = i3c_hci_start_with_targets();
    static const uint8_t data[4] = { 0xde, 0xad, 0xbe, 0xef };
    uint8_t da = i3c_enumerate_targets(s);
    uint32_t len = sizeof(data) << CMD1_DATA_LEN_SHIFT;
    uint32_t new_dev = (uint32_t)DA_NEW << CMD0_DEV_INDEX_SHIFT;
    uint32_t odd_parity = !(ctpop8(DA_NEW) & 1);
    uint32_t resp;

    i3c_writel(s, 0, PIO_REG_XFER_DATA, (DA_NEW << 1) | odd_parity);
    resp = i3c_run_cmd(s, CMD_ATTR_CCC | (9u << CMD0_TID_SHIFT) |
                       (CCC_SETNEWDA << CMD0_CCC_CODE_SHIFT) |
                       ((uint32_t)da << CMD0_DEV_INDEX_SHIFT),
                       1u << CMD1_DATA_LEN_SHIFT);
    i3c_assert_resp(resp, RESP_SUCCESS, 9, 0);

    /* The old address is gone */
    resp = i3c_run_cmd(s, CMD_ATTR_REGULAR | (10u << CMD0_TID_SHIFT) |
                       ((uint32_t)da << CMD0_DEV_INDEX_SHIFT) | CMD0_RNW, len);
    i3c_assert_resp(resp, RESP_ERR_NACK, 10, 0);

    /* The new one takes a private write and reads it back */
    i3c_writel(s, 0, PIO_REG_XFER_DATA, ldl_le_p(data));
    resp = i3c_run_cmd(s, CMD_ATTR_REGULAR | (11u << CMD0_TID_SHIFT) | new_dev,
                       len);
    i3c_assert_resp(resp, RESP_SUCCESS, 11, 0);
    resp = i3c_run_cmd(s, CMD_ATTR_REGULAR | (12u << CMD0_TID_SHIFT) | new_dev |
                       CMD0_RNW, len);
    i3c_assert_resp(resp, RESP_SUCCESS, 12, sizeof(data));
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_XFER_DATA), ==,
                    (uint32_t)ldl_le_p(data));

    qtest_quit(s);
}

/*
 * Target B refuses GETPID.  It must not NACK the broadcast part of a GETPID
 * addressed to target A, only one addressed to itself.
 */
static void test_broadcast_phase_nack(void)
{
    QTestState *s = i3c_hci_start_with_targets_opts(",nack-ccc=0x8d");
    uint8_t da_a = i3c_enumerate_targets(s);
    uint32_t resp;

    resp = i3c_ccc_read(s, 13, CCC_GETPID, da_a, 6);
    i3c_assert_resp(resp, RESP_SUCCESS, 13, 6);
    i3c_readl(s, 0, PIO_REG_XFER_DATA);
    i3c_readl(s, 0, PIO_REG_XFER_DATA);

    resp = i3c_ccc_read(s, 14, CCC_GETPID, i3c_other_da(da_a), 6);
    i3c_assert_resp(resp, RESP_ERR_NACK, 14, 0);

    qtest_quit(s);
}

/*
 * Targets at addresses whose DAT entries lie past the first 16 work like any
 * other: transfers, IBIs and SIR_REJECT, without touching the DCT.
 */
static void test_high_address(void)
{
    QTestState *s = i3c_hci_start_with_targets();
    uint8_t da = i3c_enumerate_targets_at(s, DA_HIGH);
    uint32_t dev = (uint32_t)da << CMD0_DEV_INDEX_SHIFT;
    uint32_t len = 4u << CMD1_DATA_LEN_SHIFT;
    uint32_t dct[8];
    uint32_t resp;
    int i;

    for (i = 0; i < 8; i++) {
        dct[i] = i3c_readl(s, 0, DCT_BASE + i * 4);
    }
    g_assert_cmphex(dct[3], ==, DA_HIGH);
    g_assert_cmphex(dct[7], ==, DA_HIGH + 1);

    i3c_writel(s, 0, PIO_REG_XFER_DATA, 0x12345678);
    resp = i3c_run_cmd(s, CMD_ATTR_REGULAR | (2u << CMD0_TID_SHIFT) | dev, len);
    i3c_assert_resp(resp, RESP_SUCCESS, 2, 0);
    resp = i3c_run_cmd(s, CMD_ATTR_REGULAR | (3u << CMD0_TID_SHIFT) | dev |
                       CMD0_RNW, len);
    i3c_assert_resp(resp, RESP_SUCCESS, 3, 4);
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_XFER_DATA), ==, 0x12345678);

    i3c_target_a_raise_ibi(s, da);
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_IBI_PORT), ==,
                    IBI_STATUS_LAST |
                    (((uint32_t)da << 1 | 1) << IBI_STATUS_ID_SHIFT));

    i3c_writel(s, 0, DAT_BASE + da * 8, DAT_W0_SIR_REJECT);
    g_assert_cmphex(i3c_readl(s, 0, DAT_BASE + da * 8), ==, DAT_W0_SIR_REJECT);
    i3c_target_a_raise_ibi(s, da);
    g_assert_cmphex(i3c_readl(s, 0, PIO_REG_INTR_STATUS) & PIO_INTR_IBI_THLD,
                    ==, 0);

    for (i = 0; i < 8; i++) {
        g_assert_cmphex(i3c_readl(s, 0, DCT_BASE + i * 4), ==, dct[i]);
    }

    qtest_quit(s);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/ast2700/i3c-hci/reset_values", test_reset_values);
    qtest_add_func("/ast2700/i3c-hci/dct_index_writable",
                   test_dct_index_writable);
    qtest_add_func("/ast2700/i3c-hci/controllers_independent",
                   test_controllers_independent);
    qtest_add_func("/ast2700/i3c-hci/command_completes",
                   test_command_completes);
    qtest_add_func("/ast2700/i3c-hci/cmd_queue_full_depth_drains_in_order",
                   test_cmd_queue_full_depth_drains_in_order);
    qtest_add_func("/ast2700/i3c-hci/cmd_queue_reset_stays_functional",
                   test_cmd_queue_reset_stays_functional);
    qtest_add_func("/ast2700/i3c-hci/soft_reset_restores_defaults",
                   test_soft_reset_restores_defaults);

    if (qtest_has_device("mock-i3c-target")) {
        qtest_add_func("/ast2700/i3c-hci/mock/entdaa_assigns_addresses",
                       test_entdaa_assigns_addresses);
        qtest_add_func("/ast2700/i3c-hci/mock/private_write_read",
                       test_private_write_read);
        qtest_add_func("/ast2700/i3c-hci/mock/getpid", test_getpid);
        qtest_add_func("/ast2700/i3c-hci/mock/ibi", test_ibi);
        qtest_add_func("/ast2700/i3c-hci/mock/ibi_sir_reject",
                       test_ibi_sir_reject);
        qtest_add_func("/ast2700/i3c-hci/mock/stop_reaches_every_target",
                       test_stop_reaches_every_target);
        qtest_add_func("/ast2700/i3c-hci/mock/setnewda", test_setnewda);
        qtest_add_func("/ast2700/i3c-hci/mock/broadcast_phase_nack",
                       test_broadcast_phase_nack);
        qtest_add_func("/ast2700/i3c-hci/mock/high_address", test_high_address);
    }

    return g_test_run();
}
