/*
 * QTest testcase for the ASPEED AST2700 UART13 and UART14
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define AST2700_UART13_BASE     0x14c33c00
#define AST2700_UART14_BASE     0x14c33d00

/* 16550 registers, four bytes apart */
#define UART_THR                0x00
#define UART_IER                0x04
#define UART_IIR                0x08
#define UART_LSR                0x14

#define UART_IER_THRI           0x02
#define UART_IIR_NO_INT         0x01
#define UART_IIR_THRI           0x02
#define UART_LSR_THRE           0x20
#define UART_LSR_TEMT           0x40

#define AST2700_INTCIO_BASE     0x14c18000
#define INTCIO_GICINT192_EN     (AST2700_INTCIO_BASE + 0x100)
#define INTCIO_GICINT192_STATUS (AST2700_INTCIO_BASE + 0x104)
#define INTCIO_GICINT193_EN     (AST2700_INTCIO_BASE + 0x110)
#define INTCIO_GICINT193_STATUS (AST2700_INTCIO_BASE + 0x114)

#define UART13_14_INTC_BIT      (1u << 23)

/*
 * The default console takes the first serial backend and the other UARTs
 * take the rest in order, so UART13 and UART14 are the 14th and 15th.
 */
#define UART13_SERIAL_INDEX     13

typedef struct {
    uint64_t base;
    uint64_t intc_en;
    uint64_t intc_status;
    int serial_index;
} UartInfo;

static const UartInfo uart13 = {
    AST2700_UART13_BASE, INTCIO_GICINT192_EN, INTCIO_GICINT192_STATUS,
    UART13_SERIAL_INDEX,
};

static const UartInfo uart14 = {
    AST2700_UART14_BASE, INTCIO_GICINT193_EN, INTCIO_GICINT193_STATUS,
    UART13_SERIAL_INDEX + 1,
};

static QTestState *uart_test_init(char **out_paths)
{
    GString *args = g_string_new("-machine ast2700-evb");
    QTestState *qts;
    int i;

    for (i = 0; i < UART13_SERIAL_INDEX; i++) {
        g_string_append(args, " -serial null");
    }
    for (i = 0; i < 2; i++) {
        int fd = g_file_open_tmp("ast2700-uart-XXXXXX", &out_paths[i], NULL);

        g_assert(fd >= 0);
        close(fd);
        g_string_append_printf(args, " -serial file:%s", out_paths[i]);
    }

    qts = qtest_init(args->str);
    g_string_free(args, true);
    return qts;
}

static void test_uart_tx(QTestState *qts, const UartInfo *u,
                         const char *out_path)
{
    const char msg[] = "ltpi";
    g_autofree char *contents = NULL;
    size_t len;
    int i;

    g_assert_cmphex(qtest_readl(qts, u->base + UART_LSR) &
                    (UART_LSR_THRE | UART_LSR_TEMT), ==,
                    UART_LSR_THRE | UART_LSR_TEMT);

    for (i = 0; i < strlen(msg); i++) {
        qtest_writel(qts, u->base + UART_THR, msg[i]);
    }

    g_assert(g_file_get_contents(out_path, &contents, &len, NULL));
    g_assert_cmpmem(contents, len, msg, strlen(msg));
}

static void test_uart_irq(QTestState *qts, const UartInfo *u)
{
    qtest_writel(qts, u->intc_en, UART13_14_INTC_BIT);
    g_assert_cmphex(qtest_readl(qts, u->intc_status) & UART13_14_INTC_BIT,
                    ==, 0);

    /* An empty transmitter raises THRI as soon as it is enabled */
    qtest_writel(qts, u->base + UART_IER, UART_IER_THRI);
    g_assert_cmphex(qtest_readl(qts, u->intc_status) & UART13_14_INTC_BIT,
                    ==, UART13_14_INTC_BIT);

    /* Reading IIR acknowledges THRI */
    g_assert_cmphex(qtest_readl(qts, u->base + UART_IIR) & 0x0f, ==,
                    UART_IIR_THRI);
    g_assert_cmphex(qtest_readl(qts, u->base + UART_IIR) & 0x0f, ==,
                    UART_IIR_NO_INT);

    qtest_writel(qts, u->base + UART_IER, 0);
    qtest_writel(qts, u->intc_status, UART13_14_INTC_BIT);
}

static void test_uart13_14(void)
{
    char *out_paths[2];
    QTestState *qts = uart_test_init(out_paths);
    int i;

    test_uart_tx(qts, &uart13, out_paths[0]);
    test_uart_tx(qts, &uart14, out_paths[1]);
    test_uart_irq(qts, &uart13);
    test_uart_irq(qts, &uart14);

    qtest_quit(qts);
    for (i = 0; i < 2; i++) {
        unlink(out_paths[i]);
        g_free(out_paths[i]);
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/ast2700/uart/uart13_14", test_uart13_14);

    return g_test_run();
}
