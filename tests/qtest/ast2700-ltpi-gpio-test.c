/*
 * QTest testcase for the ASPEED AST2700 LTPI GPIO controllers
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define LTPI_GPIO0_BASE         0x14c34c00
#define LTPI_GPIO1_BASE         0x14c35c00
#define LTPI_GPIO_CTRL(pair)    ((pair) * 4)
#define LTPI_GPIO_IRQ_STATUS(n) (0x200 + (n) * 4)
#define LTPI_GPIO_NR_PAIRS      112

#define CTRL_OUT_DATA           (1u << 0)
#define CTRL_DIR                (1u << 1)
#define CTRL_IRQ_EN             (1u << 2)
#define CTRL_IRQ_TYPE(t)        ((uint32_t)(t) << 3)
#define CTRL_RST_TOLERANCE      (1u << 6)
#define CTRL_IRQ_STS            (1u << 12)
#define CTRL_IN_DATA            (1u << 13)
/* Bits software can set */
#define CTRL_RW_BITS            0x7fu

#define TYPE_FALLING            0
#define TYPE_RISING             1
#define TYPE_LEVEL_LOW          2
#define TYPE_LEVEL_HIGH         3
#define TYPE_BOTH_EDGES         4

#define AST2700_INTCIO_BASE     0x14c18000
#define INTCIO_GICINT192_EN     (AST2700_INTCIO_BASE + 0x100)
#define INTCIO_GICINT192_STATUS (AST2700_INTCIO_BASE + 0x104)
#define INTCIO_GICINT193_EN     (AST2700_INTCIO_BASE + 0x110)
#define INTCIO_GICINT193_STATUS (AST2700_INTCIO_BASE + 0x114)
#define INTC_LTPI_GPIO_BIT      (1u << 22)

typedef struct {
    const char *path;
    uint64_t base;
    uint64_t intc_en;
    uint64_t intc_status;
} LtpiGpio;

static const LtpiGpio ltpi_gpio[] = {
    { "/machine/soc/ltpi-gpio[0]", LTPI_GPIO0_BASE,
      INTCIO_GICINT192_EN, INTCIO_GICINT192_STATUS },
    { "/machine/soc/ltpi-gpio[1]", LTPI_GPIO1_BASE,
      INTCIO_GICINT193_EN, INTCIO_GICINT193_STATUS },
};

static void set_input(QTestState *s, const LtpiGpio *g, int pair, int level)
{
    qtest_set_irq_in(s, g->path, "ltpi-gpio-in", pair, level);
}

static uint32_t ctrl_read(QTestState *s, const LtpiGpio *g, int pair)
{
    return qtest_readl(s, g->base + LTPI_GPIO_CTRL(pair));
}

static void ctrl_write(QTestState *s, const LtpiGpio *g, int pair,
                       uint32_t val)
{
    qtest_writel(s, g->base + LTPI_GPIO_CTRL(pair), val);
}

static bool pending(QTestState *s, const LtpiGpio *g, int pair)
{
    return ctrl_read(s, g, pair) & CTRL_IRQ_STS;
}

static void system_reset(QTestState *s)
{
    qobject_unref(qtest_qmp(s, "{ 'execute': 'system_reset' }"));
    qtest_qmp_eventwait(s, "RESET");
}

/* Input levels show in IN_DATA, per controller; software cannot write it */
static void test_input_level(void)
{
    QTestState *s = qtest_init("-machine ast2700-evb");

    g_assert_cmphex(ctrl_read(s, &ltpi_gpio[0], 5) & CTRL_IN_DATA, ==, 0);
    set_input(s, &ltpi_gpio[0], 5, 1);
    g_assert_cmphex(ctrl_read(s, &ltpi_gpio[0], 5) & CTRL_IN_DATA, ==,
                    CTRL_IN_DATA);
    g_assert_cmphex(ctrl_read(s, &ltpi_gpio[1], 5) & CTRL_IN_DATA, ==, 0);

    set_input(s, &ltpi_gpio[1], 7, 1);
    g_assert_cmphex(ctrl_read(s, &ltpi_gpio[1], 7) & CTRL_IN_DATA, ==,
                    CTRL_IN_DATA);
    g_assert_cmphex(ctrl_read(s, &ltpi_gpio[0], 7) & CTRL_IN_DATA, ==, 0);

    ctrl_write(s, &ltpi_gpio[0], 5, 0);
    g_assert_cmphex(ctrl_read(s, &ltpi_gpio[0], 5) & CTRL_IN_DATA, ==,
                    CTRL_IN_DATA);

    qtest_quit(s);
}

/* Reserved bits read as zero and the status bit is write-1-to-clear */
static void test_register_bits(void)
{
    QTestState *s = qtest_init("-machine ast2700-evb");
    const LtpiGpio *g = &ltpi_gpio[0];

    ctrl_write(s, g, 9, 0xffffffff);
    g_assert_cmphex(ctrl_read(s, g, 9), ==, CTRL_RW_BITS);

    set_input(s, g, 9, 1);
    g_assert_cmphex(ctrl_read(s, g, 9), ==,
                    CTRL_RW_BITS | CTRL_IRQ_STS | CTRL_IN_DATA);

    /* Writing 0 leaves it pending */
    ctrl_write(s, g, 9, CTRL_RW_BITS);
    g_assert_true(pending(s, g, 9));
    ctrl_write(s, g, 9, CTRL_RW_BITS | CTRL_IRQ_STS);
    g_assert_false(pending(s, g, 9));

    qtest_quit(s);
}

/* Does a @from -> @to transition make an edge trigger of @type pending? */
static bool edge_triggers(QTestState *s, const LtpiGpio *g, int pair,
                          uint32_t type, int from, int to)
{
    bool sts;

    set_input(s, g, pair, from);
    ctrl_write(s, g, pair, CTRL_IRQ_EN | CTRL_IRQ_TYPE(type) | CTRL_IRQ_STS);
    set_input(s, g, pair, to);
    sts = pending(s, g, pair);
    ctrl_write(s, g, pair, CTRL_IRQ_STS);
    return sts;
}

/* Is a level trigger of @type pending while the input is at @level? */
static bool level_triggers(QTestState *s, const LtpiGpio *g, int pair,
                           uint32_t type, int level)
{
    bool sts;

    ctrl_write(s, g, pair, CTRL_IRQ_STS);
    set_input(s, g, pair, level);
    ctrl_write(s, g, pair, CTRL_IRQ_EN | CTRL_IRQ_TYPE(type) | CTRL_IRQ_STS);
    sts = pending(s, g, pair);
    ctrl_write(s, g, pair, CTRL_IRQ_STS);
    return sts;
}

static void test_trigger_types(void)
{
    QTestState *s = qtest_init("-machine ast2700-evb");
    const LtpiGpio *g = &ltpi_gpio[0];
    uint32_t type;

    g_assert_true(edge_triggers(s, g, 3, TYPE_FALLING, 1, 0));
    g_assert_false(edge_triggers(s, g, 3, TYPE_FALLING, 0, 1));
    g_assert_true(edge_triggers(s, g, 3, TYPE_RISING, 0, 1));
    g_assert_false(edge_triggers(s, g, 3, TYPE_RISING, 1, 0));

    /* Any type with bit 2 set triggers on both edges */
    for (type = TYPE_BOTH_EDGES; type < 8; type++) {
        g_assert_true(edge_triggers(s, g, 3, type, 0, 1));
        g_assert_true(edge_triggers(s, g, 3, type, 1, 0));
        g_assert_false(edge_triggers(s, g, 3, type, 1, 1));
    }

    g_assert_true(level_triggers(s, g, 3, TYPE_LEVEL_LOW, 0));
    g_assert_false(level_triggers(s, g, 3, TYPE_LEVEL_LOW, 1));
    g_assert_true(level_triggers(s, g, 3, TYPE_LEVEL_HIGH, 1));
    g_assert_false(level_triggers(s, g, 3, TYPE_LEVEL_HIGH, 0));

    /* No status while the interrupt is disabled */
    ctrl_write(s, g, 3, CTRL_IRQ_TYPE(TYPE_RISING));
    set_input(s, g, 3, 0);
    set_input(s, g, 3, 1);
    g_assert_false(pending(s, g, 3));

    qtest_quit(s);
}

/* A level trigger stays pending until the input leaves the level */
static void test_level_stays_pending(void)
{
    QTestState *s = qtest_init("-machine ast2700-evb");
    const LtpiGpio *g = &ltpi_gpio[1];
    const int pair = 20;

    set_input(s, g, pair, 1);
    ctrl_write(s, g, pair, CTRL_IRQ_EN | CTRL_IRQ_TYPE(TYPE_LEVEL_LOW));
    g_assert_false(pending(s, g, pair));

    set_input(s, g, pair, 0);
    g_assert_true(pending(s, g, pair));
    ctrl_write(s, g, pair,
               CTRL_IRQ_EN | CTRL_IRQ_TYPE(TYPE_LEVEL_LOW) | CTRL_IRQ_STS);
    g_assert_true(pending(s, g, pair));

    set_input(s, g, pair, 1);
    g_assert_true(pending(s, g, pair));
    ctrl_write(s, g, pair,
               CTRL_IRQ_EN | CTRL_IRQ_TYPE(TYPE_LEVEL_LOW) | CTRL_IRQ_STS);
    g_assert_false(pending(s, g, pair));

    qtest_quit(s);
}

/* Status registers mirror 32 pairs each and the INTC1 bit follows them */
static void test_irq_status(void)
{
    static const int pairs[] = { 0, 31, 32, 111 };
    QTestState *s = qtest_init("-machine ast2700-evb");
    int i, j;

    for (i = 0; i < ARRAY_SIZE(ltpi_gpio); i++) {
        const LtpiGpio *g = &ltpi_gpio[i];

        qtest_writel(s, g->intc_en, INTC_LTPI_GPIO_BIT);
        for (j = 0; j < ARRAY_SIZE(pairs); j++) {
            ctrl_write(s, g, pairs[j],
                       CTRL_IRQ_EN | CTRL_IRQ_TYPE(TYPE_RISING));
            set_input(s, g, pairs[j], 1);
        }

        g_assert_cmphex(qtest_readl(s, g->base + LTPI_GPIO_IRQ_STATUS(0)),
                        ==, 0x80000001);
        g_assert_cmphex(qtest_readl(s, g->base + LTPI_GPIO_IRQ_STATUS(1)),
                        ==, 0x00000001);
        g_assert_cmphex(qtest_readl(s, g->base + LTPI_GPIO_IRQ_STATUS(2)),
                        ==, 0);
        g_assert_cmphex(qtest_readl(s, g->base + LTPI_GPIO_IRQ_STATUS(3)),
                        ==, 1u << (111 - 96));
        g_assert_cmphex(qtest_readl(s, g->intc_status) & INTC_LTPI_GPIO_BIT,
                        ==, INTC_LTPI_GPIO_BIT);

        /* The status registers are read-only */
        qtest_writel(s, g->base + LTPI_GPIO_IRQ_STATUS(0), 0xffffffff);
        g_assert_cmphex(qtest_readl(s, g->base + LTPI_GPIO_IRQ_STATUS(0)),
                        ==, 0x80000001);

        for (j = 0; j < ARRAY_SIZE(pairs); j++) {
            ctrl_write(s, g, pairs[j], CTRL_IRQ_STS);
        }
        for (j = 0; j < 4; j++) {
            g_assert_cmphex(qtest_readl(s, g->base +
                                        LTPI_GPIO_IRQ_STATUS(j)), ==, 0);
        }
        qtest_writel(s, g->intc_status, INTC_LTPI_GPIO_BIT);
    }

    qtest_quit(s);
}

/* Output data drives the odd line of the pair */
static void test_output(void)
{
    QTestState *s = qtest_init("-machine ast2700-evb");
    const LtpiGpio *g = &ltpi_gpio[0];

    qtest_irq_intercept_out_named(s, g->path, "ltpi-gpio-out");

    ctrl_write(s, g, 6, CTRL_DIR | CTRL_OUT_DATA);
    g_assert_true(qtest_get_irq(s, 6));
    g_assert_false(qtest_get_irq(s, 7));
    ctrl_write(s, g, 6, CTRL_DIR);
    g_assert_false(qtest_get_irq(s, 6));

    qtest_quit(s);
}

/* Only the 112 implemented pairs have a control register */
static void test_unimplemented_pair(void)
{
    QTestState *s = qtest_init("-machine ast2700-evb");
    const LtpiGpio *g = &ltpi_gpio[0];

    ctrl_write(s, g, LTPI_GPIO_NR_PAIRS - 1, CTRL_OUT_DATA);
    g_assert_cmphex(ctrl_read(s, g, LTPI_GPIO_NR_PAIRS - 1), ==,
                    CTRL_OUT_DATA);

    ctrl_write(s, g, LTPI_GPIO_NR_PAIRS, CTRL_OUT_DATA);
    g_assert_cmphex(ctrl_read(s, g, LTPI_GPIO_NR_PAIRS), ==, 0);

    qtest_quit(s);
}

/* Reset keeps the input levels, and reset tolerant pairs keep the output */
static void test_reset(void)
{
    QTestState *s = qtest_init("-machine ast2700-evb");
    const LtpiGpio *g = &ltpi_gpio[1];

    qtest_irq_intercept_out_named(s, g->path, "ltpi-gpio-out");

    ctrl_write(s, g, 10, CTRL_RST_TOLERANCE | CTRL_DIR | CTRL_OUT_DATA);
    ctrl_write(s, g, 11, CTRL_DIR | CTRL_OUT_DATA | CTRL_IRQ_EN);
    set_input(s, g, 12, 1);
    g_assert_true(qtest_get_irq(s, 10));
    g_assert_true(qtest_get_irq(s, 11));

    system_reset(s);

    g_assert_cmphex(ctrl_read(s, g, 10), ==, CTRL_DIR | CTRL_OUT_DATA);
    g_assert_cmphex(ctrl_read(s, g, 11), ==, 0);
    g_assert_cmphex(ctrl_read(s, g, 12), ==, CTRL_IN_DATA);
    g_assert_true(qtest_get_irq(s, 10));
    g_assert_false(qtest_get_irq(s, 11));

    qtest_quit(s);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/ast2700/ltpi-gpio/input_level", test_input_level);
    qtest_add_func("/ast2700/ltpi-gpio/register_bits", test_register_bits);
    qtest_add_func("/ast2700/ltpi-gpio/trigger_types", test_trigger_types);
    qtest_add_func("/ast2700/ltpi-gpio/level_stays_pending",
                   test_level_stays_pending);
    qtest_add_func("/ast2700/ltpi-gpio/irq_status", test_irq_status);
    qtest_add_func("/ast2700/ltpi-gpio/output", test_output);
    qtest_add_func("/ast2700/ltpi-gpio/unimplemented_pair",
                   test_unimplemented_pair);
    qtest_add_func("/ast2700/ltpi-gpio/reset", test_reset);

    return g_test_run();
}
