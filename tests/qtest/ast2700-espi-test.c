/*
 * QTest testcase for the ASPEED AST2700 eSPI controller
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
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

/* Peripheral channel */
#define ESPI_CH0_INT_STS        0x108
#define ESPI_CH0_INT_EN         0x10c
#define ESPI_CH0_PC_RX_DMAL     0x110
#define ESPI_CH0_PC_RX_CTRL     0x118
#define ESPI_CH0_PC_RX_DATA     0x11c
#define ESPI_CH0_PC_TX_CTRL     0x128
#define ESPI_CH0_PC_TX_DATA     0x12c
#define ESPI_CH0_NP_TX_CTRL     0x138
#define ESPI_CH0_NP_TX_DATA     0x13c
#define ESPI_CH0_MAP0_SRCL      0x140
#define ESPI_CH0_MAP0_SRCH      0x144
#define ESPI_CH0_MAP0_TGTL      0x148
#define ESPI_CH0_MAP0_TGTH      0x14c
#define ESPI_CH0_MAP0_MASKL     0x150
#define ESPI_CH0_MAP0_MASKH     0x154

/* The four queue resets, released */
#define ESPI_CH0_CTRL_RST_ALL   0xf0000000
#define ESPI_CH0_CTRL_PC_RX_RST (1u << 28)
#define ESPI_CH0_CTRL_RD_LOCK   (1u << 7)
#define ESPI_CH0_CTRL_RD_DIS    (1u << 6)
#define ESPI_CH0_CTRL_WR_LOCK   (1u << 5)
#define ESPI_CH0_CTRL_WR_DIS    (1u << 4)
#define ESPI_CH0_CTRL_SW_RDY    (1u << 1)
#define ESPI_CH0_CTRL_READY     (1u << 0)
#define ESPI_CH0_INT_PC_TX_ERR  (1u << 4)
#define ESPI_CH0_INT_PC_TX_CMPLT (1u << 1)
#define ESPI_CH0_INT_PC_RX_CMPLT (1u << 0)
#define ESPI_TX_TRIG            (1u << 31)

#define CFG_CH0                 0x10
#define CFG_CH0_BUS_MASTER_EN   (1u << 2)
#define STS_PC_FREE             (1u << 0)
#define STS_NP_FREE             (1u << 1)

/* Peripheral cycle types, and the names the host side gives I/O cycles */
#define PERIF_MEMRD32           0x00
#define PERIF_MEMWR32           0x01
#define PERIF_SUC_CMPLT_D_ONLY  0x0f
#define PERIF_UNSUC_CMPLT_ONLY  0x0e
#define PERIF_MSG               0x10
#define PERIF_MSG_D             0x11
#define PERIF_MSG_TRIG          0x40
#define PERIF_MSG_D_TRIG        0x41
#define PERIF_IORD              0x20
#define PERIF_IOWR              0x21

/* Memory buffer interface */
#define ESPI_MMBI_CTRL          0x800
#define ESPI_MMBI_INT_STS       0x808
#define ESPI_MMBI_INT_EN        0x80c
#define ESPI_MMBI_RWP0          0x810
#define ESPI_MMBI_RWP1          0x814

/* LPC: KCS and Post Code Capture */
#define AST2700_LPC_BASE        0x14c31000
#define LPC_HICR0               0x000
#define LPC_HICR0_LPC3E         (1u << 7)
#define LPC_HICR0_LPC1E         (1u << 5)
#define LPC_HICR0_PMEE          (1u << 2)
#define LPC_HICR1               0x004
#define LPC_HICR1_PMEB          (1u << 2)
#define LPC_HICR4               0x010
#define LPC_HICR4_LADR12AS      (1u << 7)
#define LPC_HICR4_KCSENBL       (1u << 2)
#define LPC_LADR3H              0x014
#define LPC_LADR3L              0x018
#define LPC_LADR12H             0x01c
#define LPC_LADR12L             0x020
#define LPC_IDR1                0x024
#define LPC_IDR3                0x02c
#define LPC_ODR3                0x038
#define LPC_STR1                0x03c
#define LPC_STR3                0x044
#define LPC_STR_OBF             (1u << 0)
#define LPC_STR_IBF             (1u << 1)
#define LPC_STR_CMD             (1u << 3)
#define LPC_PCCR4               0x0d0
#define LPC_PCCR5               0x0d4
#define LPC_PCCR0               0x130
#define LPC_PCCR1               0x134
#define LPC_PCCR2               0x138
#define LPC_PCCR3               0x13c
#define LPC_PCCR0_EN            (1u << 0)
#define LPC_PCCR0_RXAVA_EN      (1u << 1)
#define LPC_PCCR0_CLR_FIFO      (1u << 7)
#define LPC_PCCR2_DATA_RDY      (1u << 4)
#define LPC_PCCR2_RXOVR         (1u << 3)
#define LPC_PCCR2_RXAVA         (1u << 1)

/* Virtual UART 0, linked into the first eSPI controller */
#define AST2700_VUART0_BASE     0x14c30000
#define VUART_THR               0x00
#define VUART_IER               0x04
#define VUART_FCR               0x08
#define VUART_LSR               0x14
#define VUART_GCRA              0x20
#define VUART_GCRB              0x24
#define VUART_ADDRL             0x28
#define VUART_ADDRH             0x2c
#define VUART_GCRE              0x30
#define VUART_GCRF              0x34
#define VUART_GCRG              0x38
#define VUART_GCRH              0x3c
#define VUART_LSR_DR            0x01
#define VUART_LSR_THRE          0x20
#define VUART_HOST_PORT         0x3f8

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

static uint32_t lpc_readl(QTestState *s, uint32_t off)
{
    return qtest_readl(s, AST2700_LPC_BASE + off);
}

static void lpc_writel(QTestState *s, uint32_t off, uint32_t val)
{
    qtest_writel(s, AST2700_LPC_BASE + off, val);
}

static uint32_t vuart_readl(QTestState *s, uint32_t off)
{
    return qtest_readl(s, AST2700_VUART0_BASE + off);
}

static void vuart_writel(QTestState *s, uint32_t off, uint32_t val)
{
    qtest_writel(s, AST2700_VUART0_BASE + off, val);
}

/* Issue a peripheral cycle from the host; true if the target took it */
static bool host_perif(QTestState *s, uint8_t cyc, uint64_t addr,
                       uint32_t len, const char *hex)
{
    host_set_int(s, "perif-addr", addr);
    host_set_int(s, "perif-len", len);
    if (hex) {
        host_set_str(s, "flash-data", hex);
    }
    host_set_int(s, "perif-cycle", cyc);
    return host_get_bool(s, "perif-accepted");
}

static void host_io_write(QTestState *s, uint32_t port, uint8_t byte)
{
    g_autofree char *hex = g_strdup_printf("%02x", byte);

    g_assert_true(host_perif(s, PERIF_IOWR, port, 1, hex));
}

static uint8_t host_io_read(QTestState *s, uint32_t port)
{
    g_autofree char *hex = NULL;
    unsigned long val;

    g_assert_true(host_perif(s, PERIF_IORD, port, 1, NULL));
    hex = host_get_str(s, "flash-data");
    g_assert_cmpuint(strlen(hex), ==, 2);
    g_assert_cmpint(qemu_strtoul(hex, NULL, 16, &val), ==, 0);
    return val;
}

/*
 * Peripheral channel control: the read-only bits keep the reset value, and
 * bits 7 and 5 are set-only and freeze the memory cycle disables below them.
 */
static void test_perif_ctrl(void)
{
    QTestState *s = espi_start();

    espi_writel(s, ESPI_CH0_CTRL, 0xffffffff & ~(ESPI_CH0_CTRL_RD_LOCK |
                                                 ESPI_CH0_CTRL_WR_LOCK));
    g_assert_cmphex(espi_readl(s, ESPI_CH0_CTRL), ==, 0xf00b8352);

    /* Lock the read disable while set, the write disable while clear */
    espi_writel(s, ESPI_CH0_CTRL, ESPI_CH0_CTRL_RST_ALL |
                ESPI_CH0_CTRL_RD_LOCK | ESPI_CH0_CTRL_RD_DIS);
    espi_writel(s, ESPI_CH0_CTRL, ESPI_CH0_CTRL_RST_ALL |
                ESPI_CH0_CTRL_WR_LOCK);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_CTRL), ==,
                    ESPI_CH0_CTRL_RST_ALL | ESPI_CH0_CTRL_RD_LOCK |
                    ESPI_CH0_CTRL_RD_DIS | ESPI_CH0_CTRL_WR_LOCK);
    espi_writel(s, ESPI_CH0_CTRL, ESPI_CH0_CTRL_RST_ALL |
                ESPI_CH0_CTRL_WR_DIS);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_CTRL), ==,
                    ESPI_CH0_CTRL_RST_ALL | ESPI_CH0_CTRL_RD_LOCK |
                    ESPI_CH0_CTRL_RD_DIS | ESPI_CH0_CTRL_WR_LOCK);

    qtest_quit(s);
}

/* Reserved bits of the peripheral channel registers read as zero */
static void test_perif_masks(void)
{
    QTestState *s = espi_start();

    espi_writel(s, ESPI_CH0_INT_EN, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_INT_EN), ==, 0x000003ff);
    espi_writel(s, ESPI_CH0_PC_RX_DMAL, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_PC_RX_DMAL), ==, 0xfffffffc);
    espi_writel(s, ESPI_CH0_MAP0_SRCL, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_MAP0_SRCL), ==, 0xffff0000);
    espi_writel(s, ESPI_CH0_MAP0_TGTL, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_MAP0_TGTL), ==, 0xffff0000);
    espi_writel(s, ESPI_CH0_MAP0_MASKL, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_MAP0_MASKL), ==, 0xffff0001);
    /* Bus mastering is off, so a trigger only stores the fields */
    espi_writel(s, ESPI_CH0_NP_TX_CTRL, 0x7fffffff);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_NP_TX_CTRL), ==, 0x00ffffff);

    qtest_quit(s);
}

/*
 * Window 0 maps host memory at 0xfedc0000 onto DRAM. Memory cycles complete
 * in hardware, so they work before software is ready, unless software
 * disables them.
 */
static void perif_map_window(QTestState *s, uint32_t src, uint64_t tgt,
                             uint32_t mask)
{
    espi_writel(s, ESPI_CH0_MAP0_SRCL, src);
    espi_writel(s, ESPI_CH0_MAP0_SRCH, 0);
    espi_writel(s, ESPI_CH0_MAP0_TGTL, (uint32_t)tgt);
    espi_writel(s, ESPI_CH0_MAP0_TGTH, tgt >> 32);
    espi_writel(s, ESPI_CH0_MAP0_MASKL, mask | 1);
    espi_writel(s, ESPI_CH0_MAP0_MASKH, 0xffffffff);
}

static void test_perif_memory(void)
{
    QTestState *s = espi_start_host();
    uint64_t dram = AST2700_DRAM_BASE + 0x100000;
    g_autofree char *rd = NULL;

    perif_map_window(s, 0xfedc0000, dram, 0xffff0000);

    /* Both directions come up disabled */
    g_assert_true(host_perif(s, PERIF_MEMWR32, 0xfedc0104, 4, "11223344"));
    g_assert_cmphex(qtest_readl(s, dram + 0x104), ==, 0);

    espi_writel(s, ESPI_CH0_CTRL, ESPI_CH0_CTRL_RST_ALL);
    g_assert_true(host_perif(s, PERIF_MEMWR32, 0xfedc0104, 4, "11223344"));
    g_assert_cmphex(qtest_readl(s, dram + 0x104), ==, 0x44332211);

    qtest_writel(s, dram + 0x200, 0xa5a55a5a);
    g_assert_true(host_perif(s, PERIF_MEMRD32, 0xfedc0200, 4, NULL));
    g_assert_cmphex(host_get_int(s, "flash-rsp-cyc"), ==,
                    PERIF_SUC_CMPLT_D_ONLY);
    rd = host_get_str(s, "flash-data");
    g_assert_cmpstr(rd, ==, "5a5aa5a5");

    /* Outside the window, a read is answered unsuccessfully */
    g_assert_true(host_perif(s, PERIF_MEMRD32, 0xfedd0000, 4, NULL));
    g_assert_cmphex(host_get_int(s, "flash-rsp-cyc"), ==,
                    PERIF_UNSUC_CMPLT_ONLY);

    qtest_quit(s);
}

/*
 * A message reaches software through the Posted Rx registers once software
 * is ready, and holds the queue until software releases it.
 */
static void test_perif_message_rx(void)
{
    QTestState *s = espi_start_host();
    uint8_t got[7];
    int i;

    host_set_int(s, "perif-msg", 0x5a);
    g_assert_false(host_perif(s, PERIF_MSG_D, 0x04030201, 2, "aabb"));

    espi_writel(s, ESPI_CH0_CTRL, ESPI_CH0_CTRL_RST_ALL |
                ESPI_CH0_CTRL_SW_RDY);
    g_assert_true(host_perif(s, PERIF_MSG_D, 0x04030201, 2, "aabb"));
    g_assert_cmphex(espi_readl(s, ESPI_CH0_INT_STS) &
                    ESPI_CH0_INT_PC_RX_CMPLT, ==, ESPI_CH0_INT_PC_RX_CMPLT);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_PC_RX_CTRL), ==,
                    ESPI_CYC_PEND | (2 << 12) | PERIF_MSG_D);

    /* Message code, the four message specific bytes, then the data */
    for (i = 0; i < 7; i++) {
        got[i] = espi_readl(s, ESPI_CH0_PC_RX_DATA);
    }
    g_assert_cmphex(got[0], ==, 0x5a);
    g_assert_cmphex(got[1], ==, 0x01);
    g_assert_cmphex(got[4], ==, 0x04);
    g_assert_cmphex(got[5], ==, 0xaa);
    g_assert_cmphex(got[6], ==, 0xbb);

    /* The queue holds one cycle until software releases it */
    g_assert_cmphex(host_get_int(s, "status") & STS_PC_FREE, ==, 0);
    g_assert_false(host_perif(s, PERIF_MSG, 0, 0, NULL));
    espi_writel(s, ESPI_CH0_PC_RX_CTRL, ESPI_CYC_PEND);
    g_assert_cmphex(host_get_int(s, "status") & STS_PC_FREE, ==,
                    STS_PC_FREE);

    qtest_quit(s);
}

/* Holding the Posted Rx queue in reset and releasing it */
static void test_perif_rx_reset(void)
{
    QTestState *s = espi_start_host();

    espi_writel(s, ESPI_CH0_CTRL, ESPI_CH0_CTRL_RST_ALL |
                ESPI_CH0_CTRL_SW_RDY);
    g_assert_cmphex(host_get_int(s, "status") & STS_PC_FREE, ==,
                    STS_PC_FREE);
    espi_writel(s, ESPI_CH0_CTRL, (ESPI_CH0_CTRL_RST_ALL &
                                   ~ESPI_CH0_CTRL_PC_RX_RST) |
                ESPI_CH0_CTRL_SW_RDY);
    g_assert_cmphex(host_get_int(s, "status") & STS_PC_FREE, ==, 0);
    espi_writel(s, ESPI_CH0_CTRL, ESPI_CH0_CTRL_RST_ALL |
                ESPI_CH0_CTRL_SW_RDY);
    g_assert_cmphex(host_get_int(s, "status") & STS_PC_FREE, ==,
                    STS_PC_FREE);

    qtest_quit(s);
}

/*
 * Software sends a message to the host. It needs bus mastering, and the
 * transmit control only takes the cycle types listed for it.
 */
static void test_perif_tx(void)
{
    QTestState *s = espi_start_host();
    g_autofree char *data = NULL;
    const uint8_t hdr[] = { 0x7e, 0x01, 0x02, 0x03, 0x04, 0xc3 };
    int i;

    espi_writel(s, ESPI_CH0_CTRL, ESPI_CH0_CTRL_RST_ALL |
                ESPI_CH0_CTRL_SW_RDY);

    for (i = 0; i < ARRAY_SIZE(hdr); i++) {
        espi_writel(s, ESPI_CH0_PC_TX_DATA, hdr[i]);
    }
    espi_writel(s, ESPI_CH0_PC_TX_CTRL, ESPI_TX_TRIG | (1 << 12) |
                PERIF_MSG_D_TRIG);
    g_assert_cmphex(espi_readl(s, ESPI_CH0_INT_STS) & ESPI_CH0_INT_PC_TX_ERR,
                    ==, ESPI_CH0_INT_PC_TX_ERR);
    g_assert_cmpuint(host_get_int(s, "pc-count"), ==, 0);
    espi_writel(s, ESPI_CH0_INT_STS, ESPI_CH0_INT_PC_TX_ERR);

    g_assert_true(host_set_config(s, CFG_CH0, host_get_config(s, CFG_CH0) |
                                  CFG_CH0_BUS_MASTER_EN));
    for (i = 0; i < ARRAY_SIZE(hdr); i++) {
        espi_writel(s, ESPI_CH0_PC_TX_DATA, hdr[i]);
    }
    espi_writel(s, ESPI_CH0_PC_TX_CTRL, ESPI_TX_TRIG | (1 << 12) |
                PERIF_MSG_D_TRIG);
    g_assert_cmpuint(host_get_int(s, "pc-count"), ==, 1);
    g_assert_cmphex(host_get_int(s, "pc-cyc"), ==, PERIF_MSG_D);
    g_assert_cmphex(host_get_int(s, "pc-msg"), ==, 0x7e);
    g_assert_cmphex(host_get_int(s, "pc-addr"), ==, 0x04030201);
    data = host_get_str(s, "pc-data");
    g_assert_cmpstr(data, ==, "c3");
    g_assert_cmphex(espi_readl(s, ESPI_CH0_INT_STS) &
                    ESPI_CH0_INT_PC_TX_CMPLT, ==, ESPI_CH0_INT_PC_TX_CMPLT);

    /* A message without data has length 0 */
    for (i = 0; i < ARRAY_SIZE(hdr) - 1; i++) {
        espi_writel(s, ESPI_CH0_PC_TX_DATA, hdr[i]);
    }
    espi_writel(s, ESPI_CH0_PC_TX_CTRL, ESPI_TX_TRIG | PERIF_MSG_TRIG);
    g_assert_cmpuint(host_get_int(s, "pc-count"), ==, 2);
    g_assert_cmphex(host_get_int(s, "pc-cyc"), ==, PERIF_MSG);
    g_assert_cmpuint(host_get_int(s, "pc-len"), ==, 0);
    g_assert_cmphex(host_get_int(s, "pc-addr"), ==, 0x04030201);

    /* The bus encoding of a message is not a valid trigger type */
    espi_writel(s, ESPI_CH0_PC_TX_CTRL, ESPI_TX_TRIG | PERIF_MSG);
    /* Nor is a memory write on the non-posted queue */
    espi_writel(s, ESPI_CH0_NP_TX_CTRL, ESPI_TX_TRIG | (4 << 12) |
                PERIF_MEMWR32);
    g_assert_cmpuint(host_get_int(s, "pc-count"), ==, 2);

    qtest_quit(s);
}

/*
 * A host write of a Host RWP in the memory buffer interface updates the
 * read-only copy in the controller and raises its status bit.
 */
static void test_mmbi(void)
{
    QTestState *s = espi_start_host();
    uint64_t base = AST2700_DRAM_BASE + 0x4000000;

    espi_writel(s, ESPI_MMBI_CTRL, 0xffffff71);
    g_assert_cmphex(espi_readl(s, ESPI_MMBI_CTRL), ==, 0x71);
    espi_writel(s, ESPI_MMBI_INT_EN, 0xffffffff);
    g_assert_cmphex(espi_readl(s, ESPI_MMBI_INT_EN), ==, 0xffff);

    /* One instance, the 64 MiB region behind window 0 */
    espi_writel(s, ESPI_MMBI_CTRL, 0x01);
    espi_writel(s, ESPI_CH0_CTRL, ESPI_CH0_CTRL_RST_ALL);
    perif_map_window(s, 0xf8000000, base, 0xfc000000);

    g_assert_true(host_perif(s, PERIF_MEMWR32, 0xfa000000, 4, "11223344"));
    g_assert_true(host_perif(s, PERIF_MEMWR32, 0xfa000004, 4, "55667788"));
    g_assert_cmphex(espi_readl(s, ESPI_MMBI_RWP0), ==, 0x44332211);
    g_assert_cmphex(espi_readl(s, ESPI_MMBI_RWP1), ==, 0x88776655);
    g_assert_cmphex(espi_readl(s, ESPI_MMBI_INT_STS), ==, 0x3);
    g_assert_cmphex(qtest_readl(s, base + 0x2000000), ==, 0x44332211);

    /* Read-only for the BMC */
    espi_writel(s, ESPI_MMBI_RWP0, 0);
    g_assert_cmphex(espi_readl(s, ESPI_MMBI_RWP0), ==, 0x44332211);
    espi_writel(s, ESPI_MMBI_INT_STS, 0x1);
    g_assert_cmphex(espi_readl(s, ESPI_MMBI_INT_STS), ==, 0x2);

    /* Write protection keeps the enable */
    espi_writel(s, ESPI_MMBI_CTRL, 0x03);
    espi_writel(s, ESPI_MMBI_CTRL, 0x02);
    g_assert_cmphex(espi_readl(s, ESPI_MMBI_CTRL), ==, 0x03);

    qtest_quit(s);
}

/*
 * KCS over host I/O cycles: channel 3 at a programmed address, channel 1 at
 * its reset address with the status port four above the data port.
 */
static void test_kcs(void)
{
    QTestState *s = espi_start_host();

    lpc_writel(s, LPC_LADR3H, 0x0c);
    lpc_writel(s, LPC_LADR3L, 0xa2);
    lpc_writel(s, LPC_HICR4, LPC_HICR4_KCSENBL);
    lpc_writel(s, LPC_HICR0, LPC_HICR0_LPC3E | LPC_HICR0_LPC1E);

    host_io_write(s, 0xca3, 0x61);
    g_assert_cmphex(lpc_readl(s, LPC_STR3), ==, LPC_STR_IBF | LPC_STR_CMD);
    g_assert_cmphex(lpc_readl(s, LPC_IDR3), ==, 0x61);
    lpc_writel(s, LPC_ODR3, 0x5a);
    g_assert_cmphex(host_io_read(s, 0xca3) & LPC_STR_OBF, ==, LPC_STR_OBF);
    g_assert_cmphex(host_io_read(s, 0xca2), ==, 0x5a);
    g_assert_cmphex(lpc_readl(s, LPC_STR3) & LPC_STR_OBF, ==, 0);

    host_io_write(s, 0x64, 0x62);
    g_assert_cmphex(lpc_readl(s, LPC_STR1), ==, LPC_STR_IBF | LPC_STR_CMD);
    host_io_write(s, 0x60, 0x63);
    g_assert_cmphex(lpc_readl(s, LPC_STR1), ==, LPC_STR_IBF);
    g_assert_cmphex(lpc_readl(s, LPC_IDR1), ==, 0x63);

    qtest_quit(s);
}

/* LADR12H/L hold one address per channel, selected by HICR4 */
static void test_kcs_ladr_banks(void)
{
    QTestState *s = espi_start();

    g_assert_cmphex(lpc_readl(s, LPC_LADR12L), ==, 0x00600060);
    g_assert_cmphex(lpc_readl(s, LPC_LADR12H), ==, 0x00620000);

    lpc_writel(s, LPC_HICR4, LPC_HICR4_LADR12AS);
    g_assert_cmphex(lpc_readl(s, LPC_LADR12L), ==, 0x00600062);
    lpc_writel(s, LPC_LADR12H, 0x0c);
    lpc_writel(s, LPC_LADR12L, 0xa8);
    lpc_writel(s, LPC_HICR4, 0);
    g_assert_cmphex(lpc_readl(s, LPC_LADR12L), ==, 0x00600060);
    g_assert_cmphex(lpc_readl(s, LPC_LADR12H), ==, 0x0ca80000);

    qtest_quit(s);
}

/*
 * The LPC PME output, enabled by HICR0[2] and set by HICR1[2], drives the
 * PME# virtual wire low; software cannot write PME# in the System Event
 * register.
 */
static void test_lpc_pme(void)
{
    QTestState *s = espi_start_host();
    uint32_t pme = 1u << 19;

    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0) & pme, ==, pme);
    espi_writel(s, ESPI_CH1_EVT0, espi_readl(s, ESPI_CH1_EVT0) & ~pme);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0) & pme, ==, pme);

    /* The output bit alone does nothing */
    lpc_writel(s, LPC_HICR1, LPC_HICR1_PMEB);
    g_assert_cmpuint(host_get_int(s, "vw-count"), ==, 0);

    /* PME# is wire 3 of index 4: valid bit 7, level bit 3 */
    lpc_writel(s, LPC_HICR0, LPC_HICR0_PMEE);
    g_assert_cmpuint(host_get_int(s, "vw-count"), ==, 1);
    g_assert_cmphex(host_get_int(s, "vw-index"), ==, 4);
    g_assert_cmphex(host_get_int(s, "vw-data"), ==, 0x80);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0) & pme, ==, 0);

    lpc_writel(s, LPC_HICR1, 0);
    g_assert_cmpuint(host_get_int(s, "vw-count"), ==, 2);
    g_assert_cmphex(host_get_int(s, "vw-data"), ==, 0x88);
    g_assert_cmphex(espi_readl(s, ESPI_CH1_EVT0) & pme, ==, pme);

    qtest_quit(s);
}

/* Post codes written to port 80 collect in the 256-byte capture FIFO */
static void test_pcc_fifo(void)
{
    QTestState *s = espi_start_host();
    int i;

    g_assert_cmphex(lpc_readl(s, LPC_PCCR0), ==, 0x60000000);
    g_assert_cmphex(lpc_readl(s, LPC_PCCR5), ==, 0x10000000);
    lpc_writel(s, LPC_PCCR5, 0xffffffff);
    g_assert_cmphex(lpc_readl(s, LPC_PCCR5), ==, 0x1fffffff);
    lpc_writel(s, LPC_PCCR4, 0xffffffff);
    g_assert_cmphex(lpc_readl(s, LPC_PCCR4), ==, 0xfffffff0);

    lpc_writel(s, LPC_PCCR1, 0x80);
    lpc_writel(s, LPC_PCCR0, LPC_PCCR0_EN | LPC_PCCR0_RXAVA_EN);

    host_io_write(s, 0x80, 0x5a);
    g_assert_cmphex(lpc_readl(s, LPC_PCCR2), ==,
                    (1 << 24) | LPC_PCCR2_DATA_RDY | LPC_PCCR2_RXAVA);
    /* Data ready follows the FIFO, writing it has no effect */
    lpc_writel(s, LPC_PCCR2, 0xffffffff);
    g_assert_cmphex(lpc_readl(s, LPC_PCCR2) & LPC_PCCR2_DATA_RDY, ==,
                    LPC_PCCR2_DATA_RDY);
    g_assert_cmphex(lpc_readl(s, LPC_PCCR3), ==, 0x0100015a);
    g_assert_cmphex(lpc_readl(s, LPC_PCCR2), ==, 1 << 24);

    /* One more than the FIFO holds overruns it */
    for (i = 0; i < 257; i++) {
        host_io_write(s, 0x80, i);
    }
    g_assert_cmphex(lpc_readl(s, LPC_PCCR2) & (LPC_PCCR2_RXOVR | 0x60), ==,
                    LPC_PCCR2_RXOVR | 0x60);
    g_assert_cmphex(lpc_readl(s, LPC_PCCR3) & 0xff, ==, 0x00);
    lpc_writel(s, LPC_PCCR2, LPC_PCCR2_RXOVR);
    lpc_writel(s, LPC_PCCR0, LPC_PCCR0_EN | LPC_PCCR0_CLR_FIFO);
    g_assert_cmphex(lpc_readl(s, LPC_PCCR2) & 0xff, ==, 0);

    qtest_quit(s);
}

/* Enable VUART 0 at the given host port, holding the host off while full */
static void vuart_enable(QTestState *s)
{
    vuart_writel(s, VUART_ADDRH, VUART_HOST_PORT >> 8);
    vuart_writel(s, VUART_ADDRL, VUART_HOST_PORT & 0xf8);
    vuart_writel(s, VUART_GCRA, 0x21);
}

/*
 * Bytes cross between the host and the BMC through the two FIFOs. A side's
 * transmitter stays busy until the other side has taken the byte.
 */
static void test_vuart_data(void)
{
    QTestState *s = espi_start_host();

    vuart_enable(s);

    host_io_write(s, VUART_HOST_PORT, 'h');
    g_assert_cmphex(host_io_read(s, VUART_HOST_PORT + 5) & VUART_LSR_THRE,
                    ==, 0);
    g_assert_cmphex(vuart_readl(s, VUART_LSR) & VUART_LSR_DR, ==,
                    VUART_LSR_DR);
    g_assert_cmphex(vuart_readl(s, VUART_THR), ==, 'h');
    g_assert_cmphex(host_io_read(s, VUART_HOST_PORT + 5) & VUART_LSR_THRE,
                    ==, VUART_LSR_THRE);

    vuart_writel(s, VUART_THR, 'b');
    g_assert_cmphex(vuart_readl(s, VUART_LSR) & VUART_LSR_THRE, ==, 0);
    g_assert_cmphex(host_io_read(s, VUART_HOST_PORT + 5) & VUART_LSR_DR, ==,
                    VUART_LSR_DR);
    g_assert_cmphex(host_io_read(s, VUART_HOST_PORT), ==, 'b');
    g_assert_cmphex(vuart_readl(s, VUART_LSR) & VUART_LSR_THRE, ==,
                    VUART_LSR_THRE);

    qtest_quit(s);
}

/* The general control registers that report the state of the two sides */
static void test_vuart_status_regs(void)
{
    QTestState *s = espi_start_host();
    int i;

    vuart_enable(s);

    /* Host IER bits 3:0 and FIFO enable, slave trigger level */
    host_io_write(s, VUART_HOST_PORT + 1, 0x0f);
    vuart_writel(s, VUART_FCR, 0x40);
    g_assert_cmphex(vuart_readl(s, VUART_GCRH), ==, 0x5f);

    /* The FIFOs cannot be turned off, on either side */
    vuart_writel(s, VUART_FCR, 0x00);
    g_assert_cmphex(vuart_readl(s, VUART_FCR) & 0xc0, ==, 0xc0);
    host_io_write(s, VUART_HOST_PORT + 2, 0x00);
    g_assert_cmphex(vuart_readl(s, VUART_GCRH) & 0x10, ==, 0x10);

    /* Slave IER bit 7 is reserved */
    vuart_writel(s, VUART_GCRF, 0x40);
    vuart_writel(s, VUART_IER, 0xff);
    g_assert_cmphex(vuart_readl(s, VUART_IER), ==, 0x0f);
    vuart_writel(s, VUART_IER, 0);
    vuart_writel(s, VUART_GCRF, 0);

    /* GCRG reads back the slave THR, or the timeout width bits */
    vuart_writel(s, VUART_THR, 0x77);
    g_assert_cmphex(vuart_readl(s, VUART_GCRG), ==, 0x77);
    vuart_writel(s, VUART_GCRG, 0xff);
    vuart_writel(s, VUART_GCRF, 0x04);
    g_assert_cmphex(vuart_readl(s, VUART_GCRG), ==, 0x03);
    vuart_writel(s, VUART_GCRF, 0);

    /* GCRE bit 7: the slave transmit FIFO is full */
    g_assert_cmphex(vuart_readl(s, VUART_GCRE) & 0x80, ==, 0);
    for (i = 1; i < 16; i++) {
        vuart_writel(s, VUART_THR, i);
    }
    g_assert_cmphex(vuart_readl(s, VUART_GCRE) & 0x80, ==, 0x80);

    qtest_quit(s);
}

/*
 * The host-side interrupt of the VUART goes to the host as the IRQ virtual
 * wire GCRB selects, once when raised and once when cleared.
 */
static void test_vuart_sirq(void)
{
    QTestState *s = espi_start_host();

    vuart_enable(s);
    vuart_writel(s, VUART_GCRB, 4 << 4);
    host_io_write(s, VUART_HOST_PORT + 1, 0x01);
    g_assert_cmpuint(host_get_int(s, "vw-count"), ==, 0);

    vuart_writel(s, VUART_THR, 'x');
    g_assert_cmpuint(host_get_int(s, "vw-count"), ==, 1);
    g_assert_cmphex(host_get_int(s, "vw-index"), ==, 0);
    g_assert_cmphex(host_get_int(s, "vw-data"), ==, 0x84);

    g_assert_cmphex(host_io_read(s, VUART_HOST_PORT), ==, 'x');
    g_assert_cmpuint(host_get_int(s, "vw-count"), ==, 2);
    g_assert_cmphex(host_get_int(s, "vw-data"), ==, 0x04);

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
    qtest_add_func("/ast2700/espi/perif_ctrl", test_perif_ctrl);
    qtest_add_func("/ast2700/espi/perif_masks", test_perif_masks);
    qtest_add_func("/ast2700/espi/kcs_ladr_banks", test_kcs_ladr_banks);

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
        qtest_add_func("/ast2700/espi/host/perif_memory", test_perif_memory);
        qtest_add_func("/ast2700/espi/host/perif_message_rx",
                       test_perif_message_rx);
        qtest_add_func("/ast2700/espi/host/perif_rx_reset",
                       test_perif_rx_reset);
        qtest_add_func("/ast2700/espi/host/perif_tx", test_perif_tx);
        qtest_add_func("/ast2700/espi/host/mmbi", test_mmbi);
        qtest_add_func("/ast2700/espi/host/kcs", test_kcs);
        qtest_add_func("/ast2700/espi/host/pcc_fifo", test_pcc_fifo);
        qtest_add_func("/ast2700/espi/host/lpc_pme", test_lpc_pme);
        qtest_add_func("/ast2700/espi/host/vuart_data", test_vuart_data);
        qtest_add_func("/ast2700/espi/host/vuart_status_regs",
                       test_vuart_status_regs);
        qtest_add_func("/ast2700/espi/host/vuart_sirq", test_vuart_sirq);
    }

    return g_test_run();
}
