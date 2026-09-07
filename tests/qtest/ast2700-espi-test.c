/*
 * QTest testcase for the ASPEED AST2700 eSPI controller
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define AST2700_ESPI0_BASE      0x14c05000
#define AST2700_ESPI1_BASE      0x14c06000
#define AST2700_TAFSB0_BASE     0x14c38000
#define AST2700_TAFSC_BASE      0x140e0000
#define AST2700_FMC_BASE        0x14000000
#define AST2700_FMC_FLASH_BASE  0x100000000ULL
#define AST2700_DRAM_BASE       0x400000000ULL

/* Global registers */
#define ESPI_CTRL               0x000
#define ESPI_STS                0x004
#define ESPI_INT_STS            0x008
#define ESPI_INT_EN             0x00c
#define ESPI_DEV_ID             0x010
#define ESPI_CAP_GEN            0x014
#define ESPI_CAP_CH0            0x018
#define ESPI_CAP_CH1            0x01c
#define ESPI_CAP_CH2            0x020
#define ESPI_CAP_CH3_0          0x024
#define ESPI_CAP_CH3_1          0x028
#define ESPI_DEV_STS            0x030
#define ESPI_DBG_CMD            0x040
#define ESPI_DBG_RES            0x044
#define ESPI_CH_ACC_CTRL        0x04c

#define ESPI_INT_STS_RST_DEASSERT (1u << 31)
#define ESPI_INT_STS_RST_ASSERT   (1u << 30)

/* Channel register blocks */
#define ESPI_CH0_CTRL           0x100
#define ESPI_CH0_STS            0x104
#define ESPI_CH1_CTRL           0x200
#define ESPI_CH1_STS            0x204
#define ESPI_CH1_INT_STS        0x208
#define ESPI_CH1_INT_EN         0x20c
#define ESPI_CH2_CTRL           0x300
#define ESPI_CH2_STS            0x304
#define ESPI_CH3_CTRL           0x400
#define ESPI_CH3_STS            0x404
#define ESPI_CH3_INT_STS        0x408
#define ESPI_CH3_INT_EN         0x40c
#define ESPI_CH3_RX_CTRL        0x418
#define ESPI_CH3_RX_DATA        0x41c
#define ESPI_CH3_TX_DMAL        0x420
#define ESPI_CH3_TX_DMAH        0x424
#define ESPI_CH3_TX_CTRL        0x428
#define ESPI_CH3_TX_DATA        0x42c
#define ESPI_CH3_TADDRL         0x430
#define ESPI_CH3_TADDRH         0x434
#define ESPI_CH3_MASKL          0x438
#define ESPI_CH3_MASKH          0x43c
#define ESPI_FLTR_CTRL          0x500
#define ESPI_FLTR_SADDR0        0x510
#define ESPI_FLTR_EADDR0        0x514

/* Virtual wire System Event registers and the registers beside them */
#define ESPI_CH1_EVT0           0x210
#define ESPI_CH1_EVT1           0x230
#define ESPI_EVT_INT_EN         0x04
#define ESPI_EVT_INT_T0         0x08
#define ESPI_EVT_INT_T1         0x0c
#define ESPI_EVT_INT_T2         0x10
#define ESPI_EVT_INT_STS        0x14

#define ESPI_CH1_CTRL_READY     (1u << 0)
#define ESPI_CH1_CTRL_SW_RDY    (1u << 1)
#define ESPI_CH1_INT_SYS_EVT0   (1u << 0)
#define ESPI_CH1_INT_GPIO       (1u << 1)
#define ESPI_CH1_INT_SYS_EVT1   (1u << 2)

#define ESPI_CH3_CTRL_TX_RST    (1u << 31)  /* active low */
#define ESPI_CH3_CTRL_RX_RST    (1u << 30)  /* active low */
#define ESPI_CH3_CTRL_TX_DMA_EN (1u << 17)
#define ESPI_CH3_CTRL_HW_RST    (1u << 15)  /* active low */
#define ESPI_CH3_CTRL_MODE_SW   (1u << 8)
#define ESPI_CH3_CTRL_MODE_HW   (2u << 8)
#define ESPI_CH3_CTRL_SW_RDY    (1u << 5)
#define ESPI_CH3_CTRL_READY     (1u << 4)
#define ESPI_CH3_CTRL_ARMED     (ESPI_CH3_CTRL_TX_RST | ESPI_CH3_CTRL_RX_RST | \
                                 ESPI_CH3_CTRL_HW_RST | ESPI_CH3_CTRL_SW_RDY)
#define ESPI_CH3_INT_RX_CMPLT   (1u << 0)
#define ESPI_CH3_INT_TX_CMPLT   (1u << 1)
#define ESPI_CH3_INT_TX_ERR     (1u << 4)
#define ESPI_CHAN_STS_CTRL_EN   (1u << 0)
#define ESPI_CHAN_STS_CHAN_EN   (1u << 4)
#define ESPI_CYC_PEND           (1u << 31)

/* Configuration addresses and the flash channel's ready bit */
#define CFG_GEN                 0x08
#define CFG_CH1                 0x20
#define CFG_CH3                 0x40
#define CFG_DEV_ID              0x04
#define CFG_CHAN_ENABLE         (1u << 0)
#define CFG_CHAN_READY          (1u << 1)

/* eSPI status word */
#define STS_VW_FREE             (1u << 2)
#define STS_FLASH_C_FREE        (1u << 8)
#define STS_FLASH_NP_FREE       (1u << 9)
#define STS_FLASH_C_AVAIL       (1u << 12)

/* Flash cycle and completion types */
#define FLASH_SUC_CMPLT         0x06
#define FLASH_SUC_CMPLT_D_ONLY  0x0f

/* TAFS/SAFS bridge and global controller */
#define TAFSB_CFG               0x000
#define TAFSB_CBASE             0x020
#define TAFSB_MBASE             0x024
#define TAFSB_MISC              0x070
#define TAFSC_ERASE_CTRL        0xc100
/* FMC configuration: bit 16 lets chip select 0 be written */
#define FMC_CONF                0x00
#define FMC_CONF_CE0_WRITE      (1u << 16)

#define HOST_PATH               "/machine/peripheral/host"

static uint32_t espi_readl(QTestState *s, uint32_t off)
{
    return qtest_readl(s, AST2700_ESPI0_BASE + off);
}

static void espi_writel(QTestState *s, uint32_t off, uint32_t val)
{
    qtest_writel(s, AST2700_ESPI0_BASE + off, val);
}

static QTestState *espi_start(void)
{
    return qtest_init("-machine ast2700-evb");
}

/* The machine with a mock host on the first controller */
static QTestState *espi_start_host(void)
{
    return qtest_init("-machine ast2700-evb "
                      "-device aspeed-espi-mock-host,id=host,"
                      "espi=/machine/soc/espi[0]");
}

static void host_check(QDict *rsp)
{
    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
}

static void host_set_int(QTestState *s, const char *prop, int64_t val)
{
    host_check(qtest_qmp(s, "{ 'execute': 'qom-set', 'arguments': {"
                         " 'path': %s, 'property': %s, 'value': %" PRId64
                         " } }", HOST_PATH, prop, val));
}

static void host_set_bool(QTestState *s, const char *prop, bool val)
{
    host_check(qtest_qmp(s, "{ 'execute': 'qom-set', 'arguments': {"
                         " 'path': %s, 'property': %s, 'value': %i } }",
                         HOST_PATH, prop, val));
}

static void host_set_str(QTestState *s, const char *prop, const char *val)
{
    host_check(qtest_qmp(s, "{ 'execute': 'qom-set', 'arguments': {"
                         " 'path': %s, 'property': %s, 'value': %s } }",
                         HOST_PATH, prop, val));
}

static QDict *host_get(QTestState *s, const char *prop)
{
    QDict *rsp = qtest_qmp(s, "{ 'execute': 'qom-get', 'arguments': {"
                           " 'path': %s, 'property': %s } }", HOST_PATH, prop);

    g_assert(qdict_haskey(rsp, "return"));
    return rsp;
}

static uint64_t host_get_int(QTestState *s, const char *prop)
{
    QDict *rsp = host_get(s, prop);
    uint64_t val = qdict_get_int(rsp, "return");

    qobject_unref(rsp);
    return val;
}

static bool host_get_bool(QTestState *s, const char *prop)
{
    QDict *rsp = host_get(s, prop);
    bool val = qdict_get_bool(rsp, "return");

    qobject_unref(rsp);
    return val;
}

static char *host_get_str(QTestState *s, const char *prop)
{
    QDict *rsp = host_get(s, prop);
    char *val = g_strdup(qdict_get_str(rsp, "return"));

    qobject_unref(rsp);
    return val;
}

/* Configuration cycles from the host */
static uint32_t host_get_config(QTestState *s, uint32_t addr)
{
    host_set_int(s, "cfg-addr", addr);
    return host_get_int(s, "config");
}

static bool host_set_config(QTestState *s, uint32_t addr, uint32_t val)
{
    host_set_int(s, "cfg-addr", addr);
    host_set_int(s, "config", val);
    return host_get_bool(s, "config-ok");
}

/* The host enables the flash channel and software arms it in @mode */
static void flash_enable(QTestState *s, uint32_t mode)
{
    g_assert_true(host_set_config(s, CFG_CH3, host_get_config(s, CFG_CH3) |
                                  CFG_CHAN_ENABLE));
    espi_writel(s, ESPI_CH3_CTRL, ESPI_CH3_CTRL_ARMED | mode);
}

/* Reset values of the global, channel and filter registers */
static void test_reset_values(void)
{
    QTestState *s = espi_start();
    int i;

    g_assert_cmphex(espi_readl(s, ESPI_CTRL), ==, 0x00000006);
    g_assert_cmphex(espi_readl(s, ESPI_STS), ==, 0x00000001);
    g_assert_cmphex(espi_readl(s, ESPI_DEV_ID), ==, 0x00000001);
    g_assert_cmphex(espi_readl(s, ESPI_CAP_GEN), ==, 0x230c000f);
    g_assert_cmphex(espi_readl(s, ESPI_CAP_CH0), ==, 0x00001110);
    g_assert_cmphex(espi_readl(s, ESPI_CAP_CH1), ==, 0x00000700);
    g_assert_cmphex(espi_readl(s, ESPI_CAP_CH2), ==, 0x00000110);
    g_assert_cmphex(espi_readl(s, ESPI_CAP_CH3_0), ==, 0x00031924);
    g_assert_cmphex(espi_readl(s, ESPI_CAP_CH3_1), ==, 0x00006401);
    g_assert_cmphex(espi_readl(s, ESPI_DEV_STS), ==, 0x01040104);
    g_assert_cmphex(espi_readl(s, ESPI_DBG_CMD), ==, 0x0000ff00);
    g_assert_cmphex(espi_readl(s, ESPI_DBG_RES), ==, 0x000000ff);
    g_assert_cmphex(espi_readl(s, ESPI_CH_ACC_CTRL), ==, 0x00000007);

    g_assert_cmphex(espi_readl(s, ESPI_CH0_CTRL), ==, 0xf0000350);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_STS), ==, 0xfd000001);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_CTRL), ==, 0x800001f0);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_STS), ==, 0x00000001);
    g_assert_cmphex(espi_readl(s, ESPI_CH2_CTRL), ==, 0xc0000001);
    g_assert_cmphex(espi_readl(s, ESPI_CH2_STS), ==, 0x00000001);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_CTRL), ==, 0xc0008201);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_STS), ==, 0x00001001);

    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0), ==, 0x070c0600);

    g_assert_cmphex(espi_readl(s, ESPI_CH3_TADDRL), ==, 0x00000000);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_TADDRH), ==, 0x00000020);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_MASKL), ==, 0xff000000);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_MASKH), ==, 0xffffffff);

    /* 4, 32 and 64 KiB erase: one bit per power of two KiB */
    g_assert_cmphex((espi_readl(s, ESPI_CAP_CH3_1) >> 8) & 0xff, ==,
                    (1u << 2) | (1u << 5) | (1u << 6));

    g_assert_cmphex(espi_readl(s, ESPI_FLTR_CTRL), ==, 0x00200000);
    for (i = 0; i < 16; i++) {
        g_assert_cmphex(espi_readl(s, ESPI_FLTR_SADDR0 + i * 8), ==,
                        0xffffc006);
        g_assert_cmphex(espi_readl(s, ESPI_FLTR_EADDR0 + i * 8), ==, 0);
    }

    g_assert_cmphex(espi_readl(s, ESPI_INT_STS), ==, 0);

    qtest_quit(s);
}

/* An offset with no register reads as zero */
static void test_unimplemented_reads_zero(void)
{
    QTestState *s = espi_start();

    g_assert_cmphex(espi_readl(s, 0x02c), ==, 0);
    g_assert_cmphex(espi_readl(s, 0x350), ==, 0);
    g_assert_cmphex(espi_readl(s, 0x460), ==, 0);
    g_assert_cmphex(espi_readl(s, 0x5f0), ==, 0);
    g_assert_cmphex(espi_readl(s, 0xffc), ==, 0);

    qtest_quit(s);
}

/* Read-only registers keep the reset value; writable fields take writes */
static void test_read_only(void)
{
    QTestState *s = espi_start();

    espi_writel(s, ESPI_CAP_CH3_0, 0xdeadbeef);
    g_assert_cmphex(espi_readl(s, ESPI_CAP_CH3_0), ==, 0x00031924);
    espi_writel(s, ESPI_DEV_ID, 0xdeadbeef);
    g_assert_cmphex(espi_readl(s, ESPI_DEV_ID), ==, 0x00000001);
    espi_writel(s, ESPI_DEV_STS, 0xdeadbeef);
    g_assert_cmphex(espi_readl(s, ESPI_DEV_STS), ==, 0x01040104);
    espi_writel(s, ESPI_DBG_CMD, 0xdeadbeef);
    g_assert_cmphex(espi_readl(s, ESPI_DBG_CMD), ==, 0x0000ff00);

    /* Only the RTC support bit of the general capabilities is writable */
    espi_writel(s, ESPI_CAP_GEN, 0);
    g_assert_cmphex(espi_readl(s, ESPI_CAP_GEN), ==, 0x030c000f);

    /* Software can narrow the erase sizes it advertises */
    espi_writel(s, ESPI_CAP_CH3_1, 0x00000401);
    g_assert_cmphex(espi_readl(s, ESPI_CAP_CH3_1), ==, 0x00000401);

    qtest_quit(s);
}

/* Reserved bits read back as zero; the counters in the status register */
static void test_register_masks(void)
{
    QTestState *s = espi_start();

    espi_writel(s, ESPI_CTRL, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CTRL), ==, 0x000001ff);
    espi_writel(s, ESPI_INT_EN, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_INT_EN), ==, 0xc0000000);
    espi_writel(s, ESPI_CH1_INT_EN, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_INT_EN), ==, 0x0000000f);
    espi_writel(s, ESPI_CH3_INT_EN, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_INT_EN), ==, 0x0000001f);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_EN, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_EN), ==,
                    0x0000ffff);
    espi_writel(s, ESPI_CH1_EVT1 + ESPI_EVT_INT_EN, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT1 + ESPI_EVT_INT_EN), ==,
                    0x000fffff);

    /* The abort counter is writable and the reset counter clears on 0xEA */
    espi_writel(s, ESPI_STS, 0x50000000);
    g_assert_cmphex(espi_readl(s, ESPI_STS), ==, 0x50000001);
    qtest_set_irq_in(s, "/machine/soc/espi[0]", "espi-reset", 0, 1);
    qtest_set_irq_in(s, "/machine/soc/espi[0]", "espi-reset", 0, 0);
    qtest_set_irq_in(s, "/machine/soc/espi[0]", "espi-reset", 0, 1);
    qtest_set_irq_in(s, "/machine/soc/espi[0]", "espi-reset", 0, 0);
    g_assert_cmphex(espi_readl(s, ESPI_STS), ==, 0x50020001);
    espi_writel(s, ESPI_STS, 0x50120000);
    g_assert_cmphex(espi_readl(s, ESPI_STS), ==, 0x50020001);
    espi_writel(s, ESPI_STS, 0x00ea0000);
    g_assert_cmphex(espi_readl(s, ESPI_STS), ==, 0x00000001);

    qtest_quit(s);
}

/* The channel drives its ready bits and the status mirrors */
static void test_derived_fields(void)
{
    QTestState *s = espi_start();
    uint32_t ctrl;

    ctrl = espi_readl(s, ESPI_CH3_CTRL);
    espi_writel(s, ESPI_CH3_CTRL, ctrl | ESPI_CH3_CTRL_READY);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_CTRL) & ESPI_CH3_CTRL_READY, ==, 0);
    espi_writel(s, ESPI_CH3_CTRL, ctrl | ESPI_CH3_CTRL_SW_RDY);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_CTRL) & ESPI_CH3_CTRL_SW_RDY, ==,
                    ESPI_CH3_CTRL_SW_RDY);

    ctrl = espi_readl(s, ESPI_CH1_CTRL);
    espi_writel(s, ESPI_CH1_CTRL, ctrl | ESPI_CH1_CTRL_READY);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_CTRL) & ESPI_CH1_CTRL_READY, ==, 0);

    /* Each channel status mirrors the link enable and its channel enable */
    g_assert_cmphex(espi_readl(s, ESPI_CH0_STS) & 0x11, ==, 0x01);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_STS) & 0x11, ==, 0x01);

    qtest_quit(s);
}

/*
 * The flash mapping window: the low target and mask words take only the
 * top byte.
 */
static void test_flash_window(void)
{
    QTestState *s = espi_start();

    espi_writel(s, ESPI_CH3_MASKH, 0xffffffff);
    espi_writel(s, ESPI_CH3_MASKL, 0xfe123456);
    espi_writel(s, ESPI_CH3_TADDRH, 0x00000001);
    espi_writel(s, ESPI_CH3_TADDRL, 0x80abcdef);

    g_assert_cmphex(espi_readl(s, ESPI_CH3_MASKH), ==, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_MASKL), ==, 0xfe000000);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_TADDRH), ==, 0x00000001);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_TADDRL), ==, 0x80000000);

    qtest_quit(s);
}

/* Configuration cycles reach the capability registers and enable channels */
static void test_config(void)
{
    QTestState *s = espi_start_host();

    g_assert_cmphex(host_get_config(s, CFG_GEN), ==, 0x230c000f);
    g_assert_cmphex(host_get_config(s, CFG_CH3), ==, 0x00031924);
    g_assert_false(host_set_config(s, CFG_DEV_ID, 0x2));

    /* Enabling a channel shows in its status; ready needs software too */
    g_assert_true(host_set_config(s, CFG_CH3, 0x00031924 | CFG_CHAN_ENABLE));
    g_assert_cmphex(espi_readl(s, ESPI_CH3_STS) & ESPI_CHAN_STS_CHAN_EN, ==,
                    ESPI_CHAN_STS_CHAN_EN);
    g_assert_cmphex(host_get_config(s, CFG_CH3) & CFG_CHAN_READY, ==, 0);
    espi_writel(s, ESPI_CH3_CTRL, ESPI_CH3_CTRL_ARMED | ESPI_CH3_CTRL_MODE_HW);
    g_assert_cmphex(host_get_config(s, CFG_CH3) & CFG_CHAN_READY, ==,
                    CFG_CHAN_READY);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_CTRL) & ESPI_CH3_CTRL_READY, ==,
                    ESPI_CH3_CTRL_READY);

    /* The virtual wire channel's ready bit follows the same way */
    g_assert_true(host_set_config(s, CFG_CH1, host_get_config(s, CFG_CH1) |
                                  CFG_CHAN_ENABLE));
    espi_writel(s, ESPI_CH1_CTRL, ESPI_CH1_CTRL_SW_RDY);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_CTRL) & ESPI_CH1_CTRL_READY, ==,
                    ESPI_CH1_CTRL_READY);

    qtest_quit(s);
}

/*
 * The status word: the current one and the one the host last read, with
 * alert raised while they differ.
 */
static void test_status_word(void)
{
    QTestState *s = espi_start_host();
    uint32_t idle = STS_VW_FREE | STS_FLASH_C_FREE;
    uint32_t armed = idle | STS_FLASH_NP_FREE;

    g_assert_cmphex(host_get_int(s, "status"), ==, idle);
    g_assert_false(host_get_bool(s, "alert"));

    flash_enable(s, ESPI_CH3_CTRL_MODE_HW);
    g_assert_cmphex(espi_readl(s, ESPI_DEV_STS), ==, (armed << 16) | idle);
    g_assert_true(host_get_bool(s, "alert"));
    g_assert_cmpuint(host_get_int(s, "alert-count"), ==, 1);

    g_assert_cmphex(host_get_int(s, "status"), ==, armed);
    g_assert_false(host_get_bool(s, "alert"));
    g_assert_cmphex(espi_readl(s, ESPI_DEV_STS), ==, (armed << 16) | armed);

    qtest_quit(s);
}

/* Software owns some System Event wires and the host the others */
static void test_vw_event_ownership(void)
{
    QTestState *s = espi_start();

    espi_writel(s, ESPI_CH1_CTRL, ESPI_CH1_CTRL_SW_RDY);

    /* System Event 0: only the target's wires take a write */
    espi_writel(s, ESPI_CH1_EVT0, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0), ==, 0x0ffd0600);

    /* System Event 1: the BMC generic byte and Suspend Acknowledge */
    espi_writel(s, ESPI_CH1_EVT1, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT1), ==, 0xff100000);

    qtest_quit(s);
}

/*
 * Each System Event register reports through its own channel interrupt bit;
 * the GPIO and System Event 1 bits clear through each other's position.
 */
static void test_vw_int_sts(void)
{
    QTestState *s = espi_start_host();
    uint32_t evt0 = espi_readl(s, ESPI_CH1_EVT0);

    espi_writel(s, ESPI_CH1_CTRL, ESPI_CH1_CTRL_SW_RDY);

    /* System Event 0, PLTRST# on both edges, reports in bit 0 */
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_T2, 1u << 5);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_EN, 1u << 5);
    host_set_int(s, "vw-evt0", evt0 ^ (1u << 5));
    g_assert_cmphex(espi_readl(s, ESPI_CH1_INT_STS), ==,
                    ESPI_CH1_INT_SYS_EVT0);
    espi_writel(s, ESPI_CH1_INT_STS, ESPI_CH1_INT_SYS_EVT0);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_INT_STS), ==, 0);

    /* System Event 1, a host generic wire, reports in bit 2 */
    espi_writel(s, ESPI_CH1_EVT1 + ESPI_EVT_INT_T2, 1u << 8);
    espi_writel(s, ESPI_CH1_EVT1 + ESPI_EVT_INT_EN, 1u << 8);
    host_set_int(s, "vw-evt1", 1u << 8);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT1 + ESPI_EVT_INT_STS), ==,
                    1u << 8);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_INT_STS), ==,
                    ESPI_CH1_INT_SYS_EVT1);

    /* Writing its own bit leaves it pending; writing the GPIO bit clears it */
    espi_writel(s, ESPI_CH1_INT_STS, ESPI_CH1_INT_SYS_EVT1);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_INT_STS), ==,
                    ESPI_CH1_INT_SYS_EVT1);
    espi_writel(s, ESPI_CH1_INT_STS, ESPI_CH1_INT_GPIO);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_INT_STS), ==, 0);

    qtest_quit(s);
}

/* Trigger types of the wires the host drives */
static void test_vw_trigger_types(void)
{
    QTestState *s = espi_start_host();
    uint32_t evt0 = espi_readl(s, ESPI_CH1_EVT0);
    uint32_t wire = 1u << 5;    /* PLTRST# */

    espi_writel(s, ESPI_CH1_CTRL, ESPI_CH1_CTRL_SW_RDY);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_EN, wire);

    /* Falling edge by default */
    host_set_int(s, "vw-evt0", evt0 | wire);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, 0);
    host_set_int(s, "vw-evt0", evt0);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, wire);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_INT_STS), ==,
                    ESPI_CH1_INT_SYS_EVT0);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS, wire);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, 0);

    /* Rising edge */
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_T0, wire);
    host_set_int(s, "vw-evt0", evt0 | wire);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, wire);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS, wire);
    host_set_int(s, "vw-evt0", evt0);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, 0);

    /* Both edges */
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_T2, wire);
    host_set_int(s, "vw-evt0", evt0 | wire);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, wire);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS, wire);
    host_set_int(s, "vw-evt0", evt0);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, wire);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS, wire);

    /* High level: pending as soon as it is selected, and while held */
    host_set_int(s, "vw-evt0", evt0 | wire);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS, wire);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_T2, 0);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_T1, wire);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, wire);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS, wire);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, wire);
    host_set_int(s, "vw-evt0", evt0);
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS, wire);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, 0);

    /* Low level */
    espi_writel(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_T0, 0);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0 + ESPI_EVT_INT_STS), ==, wire);

    qtest_quit(s);
}

/* A wire software drives goes to the host as a virtual wire group */
static void test_vw_notify(void)
{
    QTestState *s = espi_start_host();

    espi_writel(s, ESPI_CH1_CTRL, ESPI_CH1_CTRL_SW_RDY);
    g_assert_cmpuint(host_get_int(s, "vw-count"), ==, 0);

    /* Boot done is wire 0 of index 5: valid bit 4, level bit 0 */
    espi_writel(s, ESPI_CH1_EVT0, espi_readl(s, ESPI_CH1_EVT0) | (1u << 20));
    g_assert_cmpuint(host_get_int(s, "vw-count"), ==, 1);
    g_assert_cmphex(host_get_int(s, "vw-index"), ==, 5);
    g_assert_cmphex(host_get_int(s, "vw-data"), ==, 0x11);

    /* The host sees the wire in its copy of the register */
    g_assert_cmphex(host_get_int(s, "vw-evt0") & (1u << 20), ==, 1u << 20);

    qtest_quit(s);
}

/*
 * Software mode: a host read reaches software through the receive
 * registers, and the completion software queues stays pending until the
 * host collects it.
 */
static void test_flash_software(void)
{
    QTestState *s = espi_start_host();
    static const uint8_t data[8] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    };
    g_autofree char *got = NULL;
    int i;

    flash_enable(s, ESPI_CH3_CTRL_MODE_SW);

    host_set_int(s, "flash-addr", 0x00123456);
    host_set_int(s, "flash-tag", 3);
    host_set_int(s, "flash-read", sizeof(data));
    g_assert_true(host_get_bool(s, "flash-accepted"));

    g_assert_cmphex(espi_readl(s, ESPI_CH3_INT_STS), ==,
                    ESPI_CH3_INT_RX_CMPLT);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_RX_CTRL), ==,
                    ESPI_CYC_PEND | (8u << 12) | (3u << 8) | 0x00);
    /* The address leads the receive buffer, most significant byte first */
    g_assert_cmphex(espi_readl(s, ESPI_CH3_RX_DATA), ==, 0x00);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_RX_DATA), ==, 0x12);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_RX_DATA), ==, 0x34);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_RX_DATA), ==, 0x56);
    espi_writel(s, ESPI_CH3_RX_CTRL, ESPI_CYC_PEND);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_RX_CTRL) & ESPI_CYC_PEND, ==, 0);

    for (i = 0; i < sizeof(data); i++) {
        espi_writel(s, ESPI_CH3_TX_DATA, data[i]);
    }
    espi_writel(s, ESPI_CH3_TX_CTRL, ESPI_CYC_PEND | (8u << 12) | (3u << 8) |
                FLASH_SUC_CMPLT_D_ONLY);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_TX_CTRL) & ESPI_CYC_PEND, ==,
                    ESPI_CYC_PEND);
    g_assert_cmphex(host_get_int(s, "status") & STS_FLASH_C_AVAIL, ==,
                    STS_FLASH_C_AVAIL);

    g_assert_true(host_get_bool(s, "flash-poll"));
    g_assert_cmphex(host_get_int(s, "flash-rsp-cyc"), ==,
                    FLASH_SUC_CMPLT_D_ONLY);
    g_assert_cmphex(host_get_int(s, "flash-rsp-tag"), ==, 3);
    got = host_get_str(s, "flash-data");
    g_assert_cmpstr(got, ==, "0123456789abcdef");

    g_assert_cmphex(espi_readl(s, ESPI_CH3_TX_CTRL) & ESPI_CYC_PEND, ==, 0);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_INT_STS) & ESPI_CH3_INT_TX_CMPLT,
                    ==, ESPI_CH3_INT_TX_CMPLT);
    g_assert_false(host_get_bool(s, "flash-poll"));

    qtest_quit(s);
}

/*
 * A 32-byte completion sets length bit 5, the position of the transmit DMA
 * enable in the channel control register; with DMA off it must still go out
 * through the FIFO.
 */
static void test_flash_completion_len32(void)
{
    QTestState *s = espi_start_host();
    g_autofree char *got = NULL;
    int i;

    flash_enable(s, ESPI_CH3_CTRL_MODE_SW);
    espi_writel(s, ESPI_CH3_TX_DMAL, 0xffffffff);
    espi_writel(s, ESPI_CH3_TX_DMAH, 0xffffffff);

    for (i = 0; i < 32; i++) {
        espi_writel(s, ESPI_CH3_TX_DATA, i);
    }
    espi_writel(s, ESPI_CH3_TX_CTRL, ESPI_CYC_PEND | (32u << 12) |
                FLASH_SUC_CMPLT_D_ONLY);

    g_assert_cmphex(espi_readl(s, ESPI_CH3_CTRL) & ESPI_CH3_CTRL_TX_DMA_EN,
                    ==, 0);
    g_assert_true(host_get_bool(s, "flash-poll"));
    got = host_get_str(s, "flash-data");
    g_assert_cmpstr(got, ==, "000102030405060708090a0b0c0d0e0f"
                    "101112131415161718191a1b1c1d1e1f");

    qtest_quit(s);
}

/* Transmitting while the host has not enabled the channel is an error */
static void test_flash_tx_error(void)
{
    QTestState *s = espi_start_host();

    espi_writel(s, ESPI_CH3_CTRL, ESPI_CH3_CTRL_ARMED | ESPI_CH3_CTRL_MODE_SW);
    espi_writel(s, ESPI_CH3_TX_CTRL, ESPI_CYC_PEND | FLASH_SUC_CMPLT);

    g_assert_cmphex(espi_readl(s, ESPI_CH3_TX_CTRL) & ESPI_CYC_PEND, ==, 0);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_INT_STS) & ESPI_CH3_INT_TX_ERR, ==,
                    ESPI_CH3_INT_TX_ERR);
    g_assert_false(host_get_bool(s, "flash-poll"));

    qtest_quit(s);
}

/* Point the bridge and the channel's mapping window at @base, 16 MiB */
static void flash_map(QTestState *s, uint64_t base, bool spi)
{
    qtest_writel(s, AST2700_TAFSB0_BASE + TAFSB_CFG, spi ? 1 : 0);
    qtest_writel(s, AST2700_TAFSB0_BASE + TAFSB_CBASE, AST2700_FMC_BASE);
    qtest_writel(s, AST2700_TAFSB0_BASE + TAFSB_MBASE, (uint32_t)base);
    qtest_writel(s, AST2700_TAFSB0_BASE + TAFSB_MISC,
                 (uint32_t)(base >> 32) << 8);

    if (spi) {
        qtest_writel(s, AST2700_FMC_BASE + FMC_CONF,
                     qtest_readl(s, AST2700_FMC_BASE + FMC_CONF) |
                     FMC_CONF_CE0_WRITE);
    }

    espi_writel(s, ESPI_CH3_TADDRL, (uint32_t)base);
    espi_writel(s, ESPI_CH3_TADDRH, base >> 32);
    espi_writel(s, ESPI_CH3_MASKL, 0xff000000);
    espi_writel(s, ESPI_CH3_MASKH, 0xffffffff);
}

/* Hardware mode on memory: reads, writes, and erases with the global fill */
static void test_flash_hw_memory(void)
{
    QTestState *s = espi_start_host();
    uint64_t base = AST2700_DRAM_BASE + 0x01000000;
    uint8_t buf[4];
    g_autofree char *got = NULL;

    flash_map(s, base, false);
    flash_enable(s, ESPI_CH3_CTRL_MODE_HW);
    qtest_memwrite(s, base + 0x2000, "\xca\xfe\xf0\x0d", 4);

    host_set_int(s, "flash-addr", 0x2000);
    host_set_int(s, "flash-read", 4);
    g_assert_true(host_get_bool(s, "flash-accepted"));
    g_assert_cmphex(host_get_int(s, "flash-rsp-cyc"), ==,
                    FLASH_SUC_CMPLT_D_ONLY);
    got = host_get_str(s, "flash-data");
    g_assert_cmpstr(got, ==, "cafef00d");
    host_set_bool(s, "flash-done", true);

    host_set_int(s, "flash-addr", 0x3000);
    host_set_str(s, "flash-data", "11223344");
    host_set_bool(s, "flash-write", true);
    g_assert_cmphex(host_get_int(s, "flash-rsp-cyc"), ==, FLASH_SUC_CMPLT);
    qtest_memread(s, base + 0x3000, buf, 4);
    g_assert_cmpmem(buf, 4, "\x11\x22\x33\x44", 4);

    /* A 4 KiB erase fills with zeroes, or with ones if the global says so */
    host_set_int(s, "flash-erase", 0);
    g_assert_cmphex(host_get_int(s, "flash-rsp-cyc"), ==, FLASH_SUC_CMPLT);
    qtest_memread(s, base + 0x3000, buf, 4);
    g_assert_cmpmem(buf, 4, "\x00\x00\x00\x00", 4);
    qtest_writel(s, AST2700_TAFSC_BASE + TAFSC_ERASE_CTRL, 1u << 1);
    host_set_int(s, "flash-erase", 0);
    qtest_memread(s, base + 0x3000, buf, 4);
    g_assert_cmpmem(buf, 4, "\xff\xff\xff\xff", 4);

    /* Hardware servicing raises nothing for software */
    g_assert_cmphex(espi_readl(s, ESPI_CH3_INT_STS), ==, 0);

    qtest_quit(s);
}

/*
 * Hardware mode through the SPI controller: the bridge sends each operation
 * to the flash on the FMC's first chip select, and the flash window reads
 * back what it wrote.
 */
static void test_flash_hw_spi(void)
{
    QTestState *s = espi_start_host();
    uint8_t buf[4];
    g_autofree char *got = NULL;
    g_autofree char *erased = NULL;

    flash_map(s, AST2700_FMC_FLASH_BASE, true);
    flash_enable(s, ESPI_CH3_CTRL_MODE_HW);

    host_set_int(s, "flash-addr", 0x10000);
    host_set_int(s, "flash-erase", 0);
    g_assert_cmphex(host_get_int(s, "flash-rsp-cyc"), ==, FLASH_SUC_CMPLT);

    host_set_str(s, "flash-data", "a5a55a5a");
    host_set_bool(s, "flash-write", true);
    g_assert_cmphex(host_get_int(s, "flash-rsp-cyc"), ==, FLASH_SUC_CMPLT);

    host_set_int(s, "flash-read", 4);
    g_assert_cmphex(host_get_int(s, "flash-rsp-cyc"), ==,
                    FLASH_SUC_CMPLT_D_ONLY);
    got = host_get_str(s, "flash-data");
    g_assert_cmpstr(got, ==, "a5a55a5a");
    host_set_bool(s, "flash-done", true);

    qtest_memread(s, AST2700_FMC_FLASH_BASE + 0x10000, buf, 4);
    g_assert_cmpmem(buf, 4, "\xa5\xa5\x5a\x5a", 4);

    host_set_int(s, "flash-erase", 0);
    host_set_int(s, "flash-read", 4);
    erased = host_get_str(s, "flash-data");
    g_assert_cmpstr(erased, ==, "ffffffff");

    qtest_quit(s);
}

/* Both controllers exist, and they are independent */
static void test_two_instances(void)
{
    QTestState *s = espi_start();

    g_assert_cmphex(qtest_readl(s, AST2700_ESPI1_BASE + ESPI_CAP_CH3_1), ==,
                    0x00006401);
    qtest_writel(s, AST2700_ESPI0_BASE + ESPI_CTRL, 0x5);
    g_assert_cmphex(qtest_readl(s, AST2700_ESPI1_BASE + ESPI_CTRL), ==,
                    0x00000006);

    qtest_quit(s);
}

/* An eSPI reset tears every channel down and is reported to software */
static void test_link_reset(void)
{
    QTestState *s = espi_start_host();

    flash_enable(s, ESPI_CH3_CTRL_MODE_HW);
    espi_writel(s, ESPI_CH3_INT_EN, 0x1f);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_STS) & ESPI_CHAN_STS_CHAN_EN, ==,
                    ESPI_CHAN_STS_CHAN_EN);

    qtest_set_irq_in(s, "/machine/soc/espi[0]", "espi-reset", 0, 1);
    g_assert_cmphex(espi_readl(s, ESPI_INT_STS) & ESPI_INT_STS_RST_ASSERT, ==,
                    ESPI_INT_STS_RST_ASSERT);
    qtest_set_irq_in(s, "/machine/soc/espi[0]", "espi-reset", 0, 0);

    g_assert_cmphex(espi_readl(s, ESPI_CH3_CTRL) & ESPI_CH3_CTRL_SW_RDY, ==, 0);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_CTRL) & ESPI_CH3_CTRL_READY, ==, 0);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_INT_EN), ==, 0);
    g_assert_cmphex(espi_readl(s, ESPI_CH3_STS) & ESPI_CHAN_STS_CHAN_EN, ==, 0);
    g_assert_cmphex(espi_readl(s, ESPI_INT_STS) & ESPI_INT_STS_RST_DEASSERT,
                    ==, ESPI_INT_STS_RST_DEASSERT);
    g_assert_cmphex(espi_readl(s, ESPI_DEV_STS), ==, 0x01040104);

    qtest_quit(s);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/ast2700/espi/reset_values", test_reset_values);
    qtest_add_func("/ast2700/espi/unimplemented_reads_zero",
                   test_unimplemented_reads_zero);
    qtest_add_func("/ast2700/espi/read_only", test_read_only);
    qtest_add_func("/ast2700/espi/register_masks", test_register_masks);
    qtest_add_func("/ast2700/espi/derived_fields", test_derived_fields);
    qtest_add_func("/ast2700/espi/flash_window", test_flash_window);
    qtest_add_func("/ast2700/espi/vw_event_ownership",
                   test_vw_event_ownership);
    qtest_add_func("/ast2700/espi/two_instances", test_two_instances);

    if (qtest_has_device("aspeed-espi-mock-host")) {
        qtest_add_func("/ast2700/espi/host/config", test_config);
        qtest_add_func("/ast2700/espi/host/status_word", test_status_word);
        qtest_add_func("/ast2700/espi/host/vw_int_sts", test_vw_int_sts);
        qtest_add_func("/ast2700/espi/host/vw_trigger_types",
                       test_vw_trigger_types);
        qtest_add_func("/ast2700/espi/host/vw_notify", test_vw_notify);
        qtest_add_func("/ast2700/espi/host/flash_software",
                       test_flash_software);
        qtest_add_func("/ast2700/espi/host/flash_completion_len32",
                       test_flash_completion_len32);
        qtest_add_func("/ast2700/espi/host/flash_tx_error",
                       test_flash_tx_error);
        qtest_add_func("/ast2700/espi/host/flash_hw_memory",
                       test_flash_hw_memory);
        qtest_add_func("/ast2700/espi/host/flash_hw_spi", test_flash_hw_spi);
        qtest_add_func("/ast2700/espi/host/link_reset", test_link_reset);
    }

    return g_test_run();
}
