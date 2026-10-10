/*
 * QTest testcase for the generic MIPI I3C HCI controller
 *
 * The controller is created on the platform bus of the virt machine and
 * driven the standard way: dynamic addresses are given in the DAT, and
 * commands name a target by its DAT index.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

/* First device on the virt platform bus */
#define HCI_BASE                0x0c000000

#define HCI_REG_VERSION         0x000
#define HCI_REG_HC_CAPABILITIES 0x00c
#define HCI_REG_DAT_SECTION     0x030
#define HCI_REG_DCT_SECTION     0x034
#define HCI_REG_RING_HDR_SEC    0x038
#define HCI_REG_PIO_SECTION     0x03c
#define HCI_REG_EXT_CAPS_SEC    0x040

/* HCI 1.0: table sizes in words, 16 DAT and 16 DCT entries */
#define HCI_VERSION_RESET       0x00000100u
#define HC_CAPABILITIES_RESET   0x00004002u
#define DAT_SECTION_RESET       0x00020200u
#define DCT_SECTION_RESET       0x00040280u
#define PIO_SECTION_RESET       0x00000100u
#define EXT_CAPS_SECTION_RESET  0x00000380u

#define PIO_BASE                0x100
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

#define DAT_BASE                0x200
#define DAT_W0_SIR_REJECT       (1u << 13)
#define DAT_W0_DYNAMIC_ADDR_SHIFT 16
#define DCT_BASE                0x280
#define EXT_CAPS_BASE           0x380

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

#define IBI_STATUS_LAST         (1u << 24)
#define IBI_STATUS_ID_SHIFT     8

#define CCC_ENEC_DIRECT         0x80
#define CCC_GETPID              0x8d
#define ENEC_ENINT              0x01

#define POLL_TIMEOUT_US (5 * 1000 * 1000)

#define TGT_A_PID               0x0123456789abULL
#define TGT_B_PID               0x0a0b0c0d0e0fULL
#define TGT_A_IBI_MAGIC         0x5a
/*
 * The targets get DAT entries 2 and 3, holding addresses unrelated to the
 * entry numbers.
 */
#define DAT_IDX_FIRST           2
#define DA_FIRST                0x20

static uint32_t hci_readl(QTestState *s, uint32_t off)
{
    return qtest_readl(s, HCI_BASE + off);
}

static void hci_writel(QTestState *s, uint32_t off, uint32_t val)
{
    qtest_writel(s, HCI_BASE + off, val);
}

static QTestState *hci_start(void)
{
    return qtest_initf("-machine virt -device mipi-i3c-hci "
                       "-device mock-i3c-target,bus=mipi-i3c-hci.0,"
                       "pid=0x%" PRIx64 ",ibi-magic-num=0x%x "
                       "-device mock-i3c-target,bus=mipi-i3c-hci.0,"
                       "pid=0x%" PRIx64,
                       (uint64_t)TGT_A_PID, TGT_A_IBI_MAGIC,
                       (uint64_t)TGT_B_PID);
}

/* Fail rather than hang if @bits never get set */
static void hci_wait_intr(QTestState *s, uint32_t bits)
{
    int64_t end = g_get_monotonic_time() + POLL_TIMEOUT_US;

    while ((hci_readl(s, PIO_REG_INTR_STATUS) & bits) != bits) {
        g_assert_true(g_get_monotonic_time() < end);
        g_usleep(1000);
    }
}

static uint32_t hci_run_cmd(QTestState *s, uint32_t w0, uint32_t w1)
{
    hci_writel(s, PIO_REG_CMD_QUEUE, w0 | CMD0_TOC | CMD0_ROC);
    hci_writel(s, PIO_REG_CMD_QUEUE, w1);
    hci_wait_intr(s, PIO_INTR_RESP_READY);
    return hci_readl(s, PIO_REG_RESP_QUEUE);
}

static void hci_assert_resp(uint32_t resp, uint32_t status, uint32_t tid,
                            uint32_t len)
{
    g_assert_cmphex((resp >> RESP_STATUS_SHIFT) & 0xf, ==, status);
    g_assert_cmphex((resp >> RESP_TID_SHIFT) & 0xf, ==, tid);
    g_assert_cmpuint(resp & RESP_DATA_LEN_MASK, ==, len);
}

static uint32_t dat_w0(uint8_t idx)
{
    return DAT_BASE + idx * 8;
}

/*
 * Give the two targets DA_FIRST and DA_FIRST + 1 through DAT entries
 * DAT_IDX_FIRST and DAT_IDX_FIRST + 1, and return the DAT index of target A.
 */
static uint8_t hci_assign_addresses(QTestState *s)
{
    uint32_t resp;
    uint64_t pid;
    int i;

    for (i = 0; i < 2; i++) {
        hci_writel(s, dat_w0(DAT_IDX_FIRST + i),
                   (uint32_t)(DA_FIRST + i) << DAT_W0_DYNAMIC_ADDR_SHIFT);
    }
    resp = hci_run_cmd(s, CMD_ATTR_ADDRASSIGN | (1u << CMD0_TID_SHIFT) |
                       (DAT_IDX_FIRST << CMD0_DEV_INDEX_SHIFT) |
                       (2u << CMD0_DEV_COUNT_SHIFT), 0);
    hci_assert_resp(resp, RESP_SUCCESS, 1, 0);

    pid = ((uint64_t)hci_readl(s, DCT_BASE) << 16) | hci_readl(s, DCT_BASE + 4);
    return pid == TGT_A_PID ? DAT_IDX_FIRST : DAT_IDX_FIRST + 1;
}

static void test_reset_values(void)
{
    QTestState *s = hci_start();

    g_assert_cmphex(hci_readl(s, HCI_REG_VERSION), ==, HCI_VERSION_RESET);
    g_assert_cmphex(hci_readl(s, HCI_REG_HC_CAPABILITIES), ==,
                    HC_CAPABILITIES_RESET);
    g_assert_cmphex(hci_readl(s, HCI_REG_DAT_SECTION), ==, DAT_SECTION_RESET);
    g_assert_cmphex(hci_readl(s, HCI_REG_DCT_SECTION), ==, DCT_SECTION_RESET);
    g_assert_cmphex(hci_readl(s, HCI_REG_RING_HDR_SEC), ==, 0);
    g_assert_cmphex(hci_readl(s, HCI_REG_PIO_SECTION), ==, PIO_SECTION_RESET);
    g_assert_cmphex(hci_readl(s, HCI_REG_EXT_CAPS_SEC), ==,
                    EXT_CAPS_SECTION_RESET);
    /* No Extended Capabilities: the list ends at once */
    g_assert_cmphex(hci_readl(s, EXT_CAPS_BASE), ==, 0);
    g_assert_cmphex(hci_readl(s, PIO_REG_QUEUE_SIZE), ==, PIO_QUEUE_SIZE_RESET);
    g_assert_cmphex(hci_readl(s, PIO_REG_INTR_STATUS), ==,
                    PIO_INTR_CMD_QUEUE_RDY);

    qtest_quit(s);
}

/* ENTDAA takes the addresses from the DAT and records the targets in the DCT */
static void test_entdaa_from_dat(void)
{
    QTestState *s = hci_start();
    bool seen_a = false, seen_b = false;
    int i;

    hci_assign_addresses(s);
    for (i = 0; i < 2; i++) {
        uint32_t base = DCT_BASE + i * 16;
        uint64_t pid = ((uint64_t)hci_readl(s, base) << 16) |
                       hci_readl(s, base + 4);

        g_assert_cmphex(hci_readl(s, base + 12), ==, DA_FIRST + i);
        if (pid == TGT_A_PID) {
            seen_a = true;
        } else {
            g_assert_cmphex(pid, ==, TGT_B_PID);
            seen_b = true;
        }
    }
    g_assert_true(seen_a && seen_b);

    qtest_quit(s);
}

/* Transfers and directed CCCs name the target by its DAT index */
static void test_transfers_by_dat_index(void)
{
    QTestState *s = hci_start();
    uint8_t idx = hci_assign_addresses(s);
    uint32_t dev = (uint32_t)idx << CMD0_DEV_INDEX_SHIFT;
    uint32_t len = 4u << CMD1_DATA_LEN_SHIFT;
    uint8_t got[8];
    uint32_t resp;
    int i;

    hci_writel(s, PIO_REG_XFER_DATA, 0xcafef00d);
    resp = hci_run_cmd(s, CMD_ATTR_REGULAR | (2u << CMD0_TID_SHIFT) | dev, len);
    hci_assert_resp(resp, RESP_SUCCESS, 2, 0);
    resp = hci_run_cmd(s, CMD_ATTR_REGULAR | (3u << CMD0_TID_SHIFT) | dev |
                       CMD0_RNW, len);
    hci_assert_resp(resp, RESP_SUCCESS, 3, 4);
    g_assert_cmphex(hci_readl(s, PIO_REG_XFER_DATA), ==, 0xcafef00d);

    resp = hci_run_cmd(s, CMD_ATTR_CCC | (4u << CMD0_TID_SHIFT) |
                       (CCC_GETPID << CMD0_CCC_CODE_SHIFT) | dev | CMD0_RNW,
                       6u << CMD1_DATA_LEN_SHIFT);
    hci_assert_resp(resp, RESP_SUCCESS, 4, 6);
    stl_le_p(got, hci_readl(s, PIO_REG_XFER_DATA));
    stl_le_p(got + 4, hci_readl(s, PIO_REG_XFER_DATA));
    for (i = 0; i < 6; i++) {
        g_assert_cmphex(got[i], ==, (TGT_A_PID >> (8 * (5 - i))) & 0xff);
    }

    qtest_quit(s);
}

/* Enable target A's interrupts and send it the byte that triggers an IBI */
static void hci_target_a_arm_ibi(QTestState *s, uint8_t idx)
{
    uint32_t dev = (uint32_t)idx << CMD0_DEV_INDEX_SHIFT;
    uint32_t resp;

    hci_writel(s, PIO_REG_XFER_DATA, ENEC_ENINT);
    resp = hci_run_cmd(s, CMD_ATTR_CCC | (5u << CMD0_TID_SHIFT) |
                       (CCC_ENEC_DIRECT << CMD0_CCC_CODE_SHIFT) | dev,
                       1u << CMD1_DATA_LEN_SHIFT);
    hci_assert_resp(resp, RESP_SUCCESS, 5, 0);
    resp = hci_run_cmd(s, CMD_ATTR_IMMEDIATE | (6u << CMD0_TID_SHIFT) | dev |
                       (1u << CMD0_IMM_DTT_SHIFT), TGT_A_IBI_MAGIC);
    hci_assert_resp(resp, RESP_SUCCESS, 6, 0);
}

static void hci_target_a_raise_ibi(QTestState *s, uint8_t idx)
{
    hci_target_a_arm_ibi(s, idx);
    /* The target raises its IBI 1 ms after the magic byte */
    qtest_clock_step(s, 2 * 1000 * 1000);
}

/*
 * An IBI is matched to the DAT entry holding the target's address: it is
 * taken, dropped when that entry sets SIR_REJECT, and dropped when no
 * entry holds the address.
 */
static void test_ibi_by_dat_lookup(void)
{
    QTestState *s = hci_start();
    uint8_t idx = hci_assign_addresses(s);
    uint8_t da = DA_FIRST + idx - DAT_IDX_FIRST;
    uint32_t w0 = (uint32_t)da << DAT_W0_DYNAMIC_ADDR_SHIFT;

    hci_target_a_raise_ibi(s, idx);
    g_assert_cmphex(hci_readl(s, PIO_REG_IBI_PORT), ==,
                    IBI_STATUS_LAST |
                    (((uint32_t)da << 1 | 1) << IBI_STATUS_ID_SHIFT));

    hci_writel(s, dat_w0(idx), w0 | DAT_W0_SIR_REJECT);
    hci_target_a_raise_ibi(s, idx);
    g_assert_cmphex(hci_readl(s, PIO_REG_INTR_STATUS) & PIO_INTR_IBI_THLD,
                    ==, 0);

    /*
     * Arm the IBI, then move the entry to another address before the
     * target raises it: no entry holds the target's address any more.
     */
    hci_writel(s, dat_w0(idx), w0);
    hci_target_a_arm_ibi(s, idx);
    hci_writel(s, dat_w0(idx), (uint32_t)(DA_FIRST + 8) <<
               DAT_W0_DYNAMIC_ADDR_SHIFT);
    qtest_clock_step(s, 2 * 1000 * 1000);
    g_assert_cmphex(hci_readl(s, PIO_REG_INTR_STATUS) & PIO_INTR_IBI_THLD,
                    ==, 0);

    /* Back at its address, the target's IBI is taken again */
    hci_writel(s, dat_w0(idx), w0);
    hci_target_a_raise_ibi(s, idx);
    g_assert_cmphex(hci_readl(s, PIO_REG_IBI_PORT), ==,
                    IBI_STATUS_LAST |
                    (((uint32_t)da << 1 | 1) << IBI_STATUS_ID_SHIFT));

    qtest_quit(s);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    if (!qtest_has_machine("virt") || !qtest_has_device("mipi-i3c-hci") ||
        !qtest_has_device("mock-i3c-target")) {
        g_test_skip("virt, mipi-i3c-hci or mock-i3c-target missing");
        return g_test_run();
    }

    qtest_add_func("/mipi-i3c-hci/reset_values", test_reset_values);
    qtest_add_func("/mipi-i3c-hci/entdaa_from_dat", test_entdaa_from_dat);
    qtest_add_func("/mipi-i3c-hci/transfers_by_dat_index",
                   test_transfers_by_dat_index);
    qtest_add_func("/mipi-i3c-hci/ibi_by_dat_lookup", test_ibi_by_dat_lookup);

    return g_test_run();
}
