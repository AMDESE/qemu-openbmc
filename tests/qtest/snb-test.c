/*
 * QTest testcase for the simulator network bridge (SNB)
 *
 * The test plays the peer: it listens on two Unix sockets, answers QEMU's
 * requests on the master socket from a thread, and sends its own requests
 * on the slave socket.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "qemu/bswap.h"
#include "libqtest.h"

#define AST2700_DRAM_BASE       0x400000000ULL

/* Wire format */
static const uint8_t head_tag[16] = {
    0x65, 0x55, 0xeb, 0x1a, 0xd3, 0x2d, 0x43, 0x43,
    0xb4, 0x10, 0x02, 0x28, 0x84, 0xa0, 0xa5, 0x87,
};

#define MSG(bus, op)            (((uint32_t)(bus) << 24) | (op))
#define RSP(type)               ((type) | 0x80000000U)
#define FLAG_RESPONSE           1

#define MSG_NOP                 MSG(0, 0)
#define MSG_HELLO               MSG(0, 1)
#define MSG_BUS_REGISTER        MSG(0, 3)
#define MSG_I2C_WRITE           MSG(4, 0)
#define MSG_I2C_READ            MSG(4, 1)
#define MSG_I3C_WRITE           MSG(5, 0)
#define MSG_I3C_READ            MSG(5, 1)
#define MSG_I3C_CCC             MSG(5, 2)
#define MSG_I3C_IBI             MSG(5, 3)
#define MSG_I3C_ENUM_I2C        MSG(5, 5)
#define MSG_I3C_ENUM_I3C        MSG(5, 8)
#define MSG_I3C_DA_ASSIGNED     MSG(5, 9)
#define MSG_GPIO_SET            MSG(6, 0)
#define MSG_ESPI_VW_SET         MSG(7, 1)
#define MSG_ESPI_VW_NOTIFY      MSG(7, 2)
#define MSG_ESPI_GET_STATUS     MSG(7, 4)
#define MSG_ESPI_GET_CONFIG     MSG(7, 6)
#define MSG_ESPI_SET_CONFIG     MSG(7, 7)
#define MSG_ESPI_PC_CYCLE       MSG(7, 9)

#define EP_I2C                  2
#define EP_I3C                  4
#define EP_GPIO                 6
#define EP_ESPI                 7

#define WAIT_MS                 5000

static uint32_t le32(const void *p)
{
    return (uint32_t)ldl_le_p(p);
}

typedef struct Pkt {
    uint32_t type;
    uint32_t id;
    uint32_t flags;
    uint32_t ep;
    uint32_t len;
    uint8_t msg[];
} Pkt;

/* The I3C target the peer reports */
#define TGT_PID                 0x0123456789abULL
#define TGT_BCR                 0x02
#define TGT_DCR                 0xa5

typedef struct Peer {
    char *dir;
    char *master_path;
    char *slave_path;
    int master_listen;
    int slave_listen;
    int master;
    int slave;
    GThread *thread;
    /* What QEMU sent on the master socket that the thread did not answer */
    GAsyncQueue *received;
    /* Packets the thread answered, in order, for the test to look at */
    GAsyncQueue *answered;
    unsigned n_i3c_targets;
    int enum_delay_ms;
    uint8_t read_data[8];
    uint32_t next_id;
} Peer;

static int peer_listen(const char *path)
{
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);

    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpuint(strlen(path), <, sizeof(addr.sun_path));
    strcpy(addr.sun_path, path);
    g_assert_cmpint(bind(fd, (struct sockaddr *)&addr, sizeof(addr)), ==, 0);
    g_assert_cmpint(listen(fd, 1), ==, 0);
    return fd;
}

static bool peer_read(int fd, void *buf, size_t len, int timeout_ms)
{
    uint8_t *p = buf;

    while (len) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        ssize_t n;

        if (poll(&pfd, 1, timeout_ms) <= 0) {
            return false;
        }
        n = read(fd, p, len);
        if (n <= 0) {
            return false;
        }
        p += n;
        len -= n;
    }
    return true;
}

static Pkt *peer_recv(int fd, int timeout_ms)
{
    uint8_t hdr[36];
    uint32_t size;
    Pkt *p;

    if (!peer_read(fd, hdr, sizeof(hdr), timeout_ms)) {
        return NULL;
    }
    g_assert_cmpmem(hdr, 16, head_tag, 16);
    size = le32(hdr + 16);
    g_assert_cmpuint(size, >=, 16);
    p = g_malloc(sizeof(*p) + size - 16);
    p->type = le32(hdr + 20);
    p->id = le32(hdr + 24);
    p->flags = le32(hdr + 28);
    p->ep = le32(hdr + 32);
    p->len = size - 16;
    g_assert_true(peer_read(fd, p->msg, p->len, timeout_ms));
    return p;
}

static void peer_send(int fd, uint32_t type, uint32_t id, uint32_t flags,
                      uint32_t ep, const void *msg, size_t len)
{
    g_autofree uint8_t *buf = g_malloc(36 + len);

    memcpy(buf, head_tag, 16);
    stl_le_p(buf + 16, 16 + len);
    stl_le_p(buf + 20, type);
    stl_le_p(buf + 24, id);
    stl_le_p(buf + 28, flags);
    stl_le_p(buf + 32, ep);
    memcpy(buf + 36, msg, len);
    g_assert_cmpint(write(fd, buf, 36 + len), ==, 36 + len);
}

static void peer_answer(Peer *peer, const Pkt *req, const void *msg,
                        size_t len)
{
    peer_send(peer->master, RSP(req->type), req->id, FLAG_RESPONSE, req->ep,
              msg, len);
}

static void peer_answer_status(Peer *peer, const Pkt *req)
{
    uint8_t status[4] = {};

    peer_answer(peer, req, status, sizeof(status));
}

/* Answer QEMU's requests; hand everything else to the test */
static gpointer peer_thread(gpointer opaque)
{
    Peer *peer = opaque;
    Pkt *p;

    while ((p = peer_recv(peer->master, -1))) {
        switch (p->type) {
        case MSG_I3C_ENUM_I3C: {
            uint8_t rsp[4 + 1 + 16 * 11] = {};

            g_usleep(peer->enum_delay_ms * 1000);
            rsp[4] = peer->n_i3c_targets;
            if (peer->n_i3c_targets) {
                rsp[5] = 0;
                stq_le_p(rsp + 6, TGT_PID);
                rsp[14] = TGT_BCR;
                rsp[15] = TGT_DCR;
            }
            peer_answer(peer, p, rsp, sizeof(rsp));
            break;
        }
        case MSG_I3C_ENUM_I2C: {
            uint8_t rsp[4 + 1 + 16 * 2] = {};

            peer_answer(peer, p, rsp, sizeof(rsp));
            break;
        }
        case MSG_I3C_READ: {
            uint8_t rsp[8 + 3] = {};

            stl_le_p(rsp + 4, 3);
            memcpy(rsp + 8, peer->read_data, 3);
            peer_answer(peer, p, rsp, sizeof(rsp));
            break;
        }
        case MSG_I2C_READ: {
            uint32_t len = le32(p->msg + 2);
            g_autofree uint8_t *rsp = g_malloc0(4 + len);

            memcpy(rsp + 4, peer->read_data, MIN(len, 8));
            peer_answer(peer, p, rsp, 4 + len);
            break;
        }
        case MSG_I3C_CCC: {
            uint8_t rsp[8] = {};

            peer_answer(peer, p, rsp, sizeof(rsp));
            break;
        }
        case MSG_BUS_REGISTER:
        case MSG_I2C_WRITE:
        case MSG_I3C_WRITE:
        case MSG_I3C_DA_ASSIGNED:
            peer_answer_status(peer, p);
            g_async_queue_push(peer->answered, p);
            continue;
        default:
            /* Notifications */
            g_async_queue_push(peer->received, p);
            continue;
        }
        g_free(p);
    }
    return NULL;
}

/* Take the next packet of @type from @q, dropping others of no interest */
static Pkt *peer_wait(GAsyncQueue *q, uint32_t type)
{
    for (;;) {
        Pkt *p = g_async_queue_timeout_pop(q, WAIT_MS * 1000);

        g_assert_nonnull(p);
        if (p->type == type) {
            return p;
        }
        g_free(p);
    }
}

static void peer_hello(int fd)
{
    g_autofree Pkt *p = peer_recv(fd, WAIT_MS);
    uint8_t hello[8] = {};

    g_assert_nonnull(p);
    g_assert_cmphex(p->type, ==, MSG_HELLO);
    g_assert_cmpuint(p->len, ==, 8);
    g_assert_cmpuint(lduw_le_p(p->msg), ==, 2);
    g_assert_cmpuint(lduw_le_p(p->msg + 2), ==, 8);
    stw_le_p(hello, 2);
    stw_le_p(hello + 2, 8);
    peer_send(fd, MSG_HELLO, p->id, FLAG_RESPONSE, 0, hello, sizeof(hello));
}

static int peer_accept(int fd)
{
    struct pollfd pfd = { .fd = fd, .events = POLLIN };

    g_assert_cmpint(poll(&pfd, 1, WAIT_MS), ==, 1);
    return accept(fd, NULL, NULL);
}

/* Take QEMU's connection: the slave socket first, then the master socket */
static void peer_connect(Peer *peer)
{
    peer->slave = peer_accept(peer->slave_listen);
    peer_hello(peer->slave);
    peer->master = peer_accept(peer->master_listen);
    peer_hello(peer->master);
    peer->thread = g_thread_new("snb-test-peer", peer_thread, peer);
}

static void peer_disconnect(Peer *peer)
{
    shutdown(peer->master, SHUT_RDWR);
    shutdown(peer->slave, SHUT_RDWR);
    g_thread_join(peer->thread);
    close(peer->master);
    close(peer->slave);
}

static Peer *peer_new(void)
{
    Peer *peer = g_new0(Peer, 1);

    peer->dir = g_dir_make_tmp("snb-test-XXXXXX", NULL);
    peer->master_path = g_build_filename(peer->dir, "master", NULL);
    peer->slave_path = g_build_filename(peer->dir, "slave", NULL);
    peer->master_listen = peer_listen(peer->master_path);
    peer->slave_listen = peer_listen(peer->slave_path);
    peer->received = g_async_queue_new_full(g_free);
    peer->answered = g_async_queue_new_full(g_free);
    peer->next_id = 1000;
    return peer;
}

static void peer_free(Peer *peer)
{
    peer_disconnect(peer);
    close(peer->master_listen);
    close(peer->slave_listen);
    unlink(peer->master_path);
    unlink(peer->slave_path);
    rmdir(peer->dir);
    g_async_queue_unref(peer->received);
    g_async_queue_unref(peer->answered);
    g_free(peer->master_path);
    g_free(peer->slave_path);
    g_free(peer->dir);
    g_free(peer);
}

static char *peer_transport(Peer *peer, const char *id)
{
    return g_strdup_printf("-device snb-transport,id=%s,"
                           "unix-master-path=%s,unix-slave-path=%s",
                           id, peer->master_path, peer->slave_path);
}

/* Send a request on the slave socket and return QEMU's answer */
static Pkt *peer_request(Peer *peer, uint32_t type, uint32_t ep,
                         const void *msg, size_t len)
{
    uint32_t id = peer->next_id++;
    Pkt *p;

    peer_send(peer->slave, type, id, 0, ep, msg, len);
    p = peer_recv(peer->slave, WAIT_MS);
    g_assert_nonnull(p);
    g_assert_cmphex(p->type, ==, RSP(type));
    g_assert_cmpuint(p->id, ==, id);
    g_assert_cmpuint(p->flags, ==, FLAG_RESPONSE);
    g_assert_cmpuint(p->ep, ==, ep);
    return p;
}

/* The peer's NOP is answered; a reconnected peer gets a new handshake */
static void test_handshake(void)
{
    Peer *peer = peer_new();
    g_autofree char *t = peer_transport(peer, "snb0");
    QTestState *s = qtest_initf("-machine ast2700-evb %s "
                                "-device snb-espi-bridge,transport=snb0,"
                                "espi-path=/machine/soc/espi[0],"
                                "bus-name=espi0", t);
    uint8_t nop[4] = {};
    g_autofree Pkt *rsp = NULL;
    Pkt *p;
    int i;

    for (i = 0; i < 2; i++) {
        if (i) {
            peer_disconnect(peer);
        }
        peer_connect(peer);
        p = peer_wait(peer->answered, MSG_BUS_REGISTER);
        g_assert_cmpuint(p->len, ==, 40);
        g_assert_cmpstr((char *)p->msg, ==, "espi0");
        g_assert_cmpuint(le32(p->msg + 32), ==, 7);
        g_assert_cmpuint(le32(p->msg + 36), ==, EP_ESPI);
        g_assert_cmpuint(p->ep, ==, EP_ESPI);
        g_free(p);
    }

    rsp = peer_request(peer, MSG_NOP, 0, nop, sizeof(nop));
    g_assert_cmpuint(rsp->len, ==, 4);
    g_assert_cmpint(le32(rsp->msg), ==, 0);

    qtest_quit(s);
    peer_free(peer);
}

#define ESPI0_BASE              0x14c05000
#define ESPI_CH0_CTRL           0x100
#define ESPI_CH0_MAP0_SRCL      0x140
#define ESPI_CH1_CTRL           0x200
#define ESPI_CH1_EVT0           0x210
#define CFG_GEN                 0x08
#define CFG_DEV_ID              0x04

static void test_espi(void)
{
    Peer *peer = peer_new();
    g_autofree char *t = peer_transport(peer, "snb0");
    QTestState *s = qtest_initf("-machine ast2700-evb %s "
                                "-device snb-espi-bridge,transport=snb0,"
                                "espi-path=/machine/soc/espi[0]", t);
    uint64_t dram = AST2700_DRAM_BASE + 0x100000;
    uint8_t reserved[1] = {};
    uint8_t nop[4] = {};
    uint8_t cfg[6], vw[5], pc[15];
    Pkt *p;

    peer_connect(peer);

    /* The status word, which the status register then shows as read */
    p = peer_request(peer, MSG_ESPI_GET_STATUS, EP_ESPI, reserved, 1);
    g_assert_cmpuint(p->len, ==, 8);
    g_assert_cmpint(le32(p->msg), ==, 0);
    g_assert_cmphex(le32(p->msg + 4), ==,
                    qtest_readl(s, ESPI0_BASE + 0x030) & 0xffff);
    g_free(p);

    /* Configuration: a read, and a write to a read-only dword */
    stw_le_p(cfg, CFG_GEN);
    stl_le_p(cfg + 2, 0);
    p = peer_request(peer, MSG_ESPI_GET_CONFIG, EP_ESPI, cfg, sizeof(cfg));
    g_assert_cmpuint(p->len, ==, 12);
    g_assert_cmpint(le32(p->msg), ==, 0);
    g_assert_cmphex(le32(p->msg + 4), ==, 0x230c000f);
    g_free(p);
    stw_le_p(cfg, CFG_DEV_ID);
    stl_le_p(cfg + 2, 2);
    p = peer_request(peer, MSG_ESPI_SET_CONFIG, EP_ESPI, cfg, sizeof(cfg));
    g_assert_cmpint(le32(p->msg), !=, 0);
    g_free(p);

    /* Wires need the virtual wire channel ready */
    qtest_writel(s, ESPI0_BASE + ESPI_CH1_CTRL, 1u << 1);

    /*
     * Wires the controller drives: index 2, wire 1 high, then wire 0 high
     * alone, which leaves wire 1 as it is
     */
    vw[0] = 2;
    stl_le_p(vw + 1, 0x22);
    peer_send(peer->slave, MSG_ESPI_VW_SET, peer->next_id++, 0, EP_ESPI, vw,
              sizeof(vw));
    stl_le_p(vw + 1, 0x11);
    peer_send(peer->slave, MSG_ESPI_VW_SET, peer->next_id++, 0, EP_ESPI, vw,
              sizeof(vw));
    g_free(peer_request(peer, MSG_NOP, 0, nop, sizeof(nop)));
    g_assert_cmphex(qtest_readl(s, ESPI0_BASE + ESPI_CH1_EVT0) & 3, ==, 3);

    /* A wire the target drives: boot done, wire 0 of index 5 */
    qtest_writel(s, ESPI0_BASE + ESPI_CH1_EVT0,
                 qtest_readl(s, ESPI0_BASE + ESPI_CH1_EVT0) | (1u << 20));
    p = peer_wait(peer->received, MSG_ESPI_VW_NOTIFY);
    g_assert_cmpuint(p->len, ==, 5);
    g_assert_cmpuint(p->msg[0], ==, 5);
    g_assert_cmphex(le32(p->msg + 1), ==, 0x11);
    g_assert_cmpuint(p->ep, ==, EP_ESPI);
    g_free(p);

    /* A memory read through peripheral window 0 */
    qtest_writel(s, ESPI0_BASE + ESPI_CH0_CTRL, 0xf0000000);
    qtest_writel(s, ESPI0_BASE + ESPI_CH0_MAP0_SRCL, 0xfedc0000);
    qtest_writel(s, ESPI0_BASE + ESPI_CH0_MAP0_SRCL + 4, 0);
    qtest_writel(s, ESPI0_BASE + ESPI_CH0_MAP0_SRCL + 8, (uint32_t)dram);
    qtest_writel(s, ESPI0_BASE + ESPI_CH0_MAP0_SRCL + 12, dram >> 32);
    qtest_writel(s, ESPI0_BASE + ESPI_CH0_MAP0_SRCL + 16, 0xffff0001);
    qtest_writel(s, ESPI0_BASE + ESPI_CH0_MAP0_SRCL + 20, 0xffffffff);
    qtest_writel(s, dram + 0x200, 0xa5a55a5a);
    memset(pc, 0, sizeof(pc));
    pc[0] = 0x00;
    pc[1] = 3;
    stq_le_p(pc + 2, 0xfedc0200);
    stl_le_p(pc + 10, 4);
    p = peer_request(peer, MSG_ESPI_PC_CYCLE, EP_ESPI, pc, sizeof(pc));
    g_assert_cmpuint(p->len, ==, 14 + 4);
    g_assert_cmpint(le32(p->msg), ==, 0);
    g_assert_cmphex(p->msg[4], ==, 0x0f);
    g_assert_cmpuint(p->msg[5], ==, 3);
    g_assert_cmphex(le32(p->msg + 14), ==, 0xa5a55a5a);
    g_free(p);

    qtest_quit(s);
    peer_free(peer);
}

#define LTPI_GPIO0_BASE         0x14c34c00
#define LTPI_CTRL_OUT_DATA      (1u << 0)
#define LTPI_CTRL_IN_DATA       (1u << 13)

static void peer_gpio_set(Peer *peer, const char *name, uint8_t level)
{
    uint8_t msg[65] = { level };

    strcpy((char *)msg + 1, name);
    peer_send(peer->slave, MSG_GPIO_SET, peer->next_id++, 0, EP_GPIO, msg,
              sizeof(msg));
}

static void test_gpio(void)
{
    Peer *peer = peer_new();
    g_autofree char *t = peer_transport(peer, "snb0");
    QTestState *s = qtest_initf("-machine ast2700-evb %s "
                                "-device 'snb-gpio-bridge,transport=snb0,"
                                "ltpi-gpio-path=/machine/soc/ltpi-gpio[0],"
                                "line-names=SIG_IN;SIG_OUT_L'", t);
    uint8_t nop[4] = {};
    Pkt *p;

    peer_connect(peer);

    /* Each named output's level on connect: low, so SIG_OUT_L asserted */
    p = peer_wait(peer->received, MSG_GPIO_SET);
    g_assert_cmpstr((char *)p->msg + 1, ==, "SIG_OUT_L");
    g_assert_cmpuint(p->msg[0], ==, 1);
    g_free(p);

    /* The peer drives an input */
    peer_gpio_set(peer, "SIG_IN", 1);
    g_free(peer_request(peer, MSG_NOP, 0, nop, sizeof(nop)));
    g_assert_cmphex(qtest_readl(s, LTPI_GPIO0_BASE) & LTPI_CTRL_IN_DATA, ==,
                    LTPI_CTRL_IN_DATA);
    peer_gpio_set(peer, "SIG_IN", 0);
    g_free(peer_request(peer, MSG_NOP, 0, nop, sizeof(nop)));
    g_assert_cmphex(qtest_readl(s, LTPI_GPIO0_BASE) & LTPI_CTRL_IN_DATA, ==,
                    0);

    /* The BMC drives the output high, which deasserts SIG_OUT_L */
    qtest_writel(s, LTPI_GPIO0_BASE, LTPI_CTRL_OUT_DATA);
    p = peer_wait(peer->received, MSG_GPIO_SET);
    g_assert_cmpstr((char *)p->msg + 1, ==, "SIG_OUT_L");
    g_assert_cmpuint(p->msg[0], ==, 0);
    g_assert_cmpuint(p->ep, ==, EP_GPIO);
    g_free(p);

    qtest_quit(s);
    peer_free(peer);
}

/* The first AST2700 I3C controller */
#define I3C0_BASE               0x14c20000
#define PIO_CMD_QUEUE           0x0d0
#define PIO_RESP_QUEUE          0x0d4
#define PIO_XFER_DATA           0x0d8
#define PIO_IBI_PORT            0x0dc
#define PIO_INTR_STATUS         0x0f0
#define PIO_INTR_RESP_READY     (1u << 4)
#define PIO_INTR_IBI_THLD       (1u << 2)
#define INHOUSE_DAA_INDEX0      0xd10
#define CMD_ATTR_REGULAR        0x0u
#define CMD_ATTR_ADDRASSIGN     0x2u
#define CMD0_TOC                (1u << 31)
#define CMD0_ROC                (1u << 30)
#define CMD0_RNW                (1u << 29)
#define CMD0_DEV_COUNT_SHIFT    26
#define CMD0_DEV_INDEX_SHIFT    16
#define CMD0_TID_SHIFT          3
#define CMD1_DATA_LEN_SHIFT     16
#define RESP_STATUS_SHIFT       28
#define RESP_DATA_LEN_MASK      0xffffu
#define IBI_STATUS_LAST         (1u << 24)
#define IBI_STATUS_ID_SHIFT     8
#define TGT_DA                  0x09

static uint32_t i3c_run_cmd(QTestState *s, uint32_t w0, uint32_t w1)
{
    int64_t end = g_get_monotonic_time() + WAIT_MS * 1000;

    qtest_writel(s, I3C0_BASE + PIO_CMD_QUEUE, w0 | CMD0_TOC | CMD0_ROC);
    qtest_writel(s, I3C0_BASE + PIO_CMD_QUEUE, w1);
    while (!(qtest_readl(s, I3C0_BASE + PIO_INTR_STATUS) &
             PIO_INTR_RESP_READY)) {
        g_assert_true(g_get_monotonic_time() < end);
        g_usleep(1000);
    }
    return qtest_readl(s, I3C0_BASE + PIO_RESP_QUEUE);
}

static void test_i3c(void)
{
    Peer *peer = peer_new();
    g_autofree char *t = peer_transport(peer, "snb0");
    QTestState *s;
    uint32_t dev = (uint32_t)TGT_DA << CMD0_DEV_INDEX_SHIFT;
    uint8_t ibi[3] = { TGT_DA, 0, 0 };
    uint32_t resp;
    Pkt *p;

    peer->n_i3c_targets = 1;
    memcpy(peer->read_data, "\x11\x22\x33", 3);
    s = qtest_initf("-machine ast2700-evb %s "
                    "-device snb-i3c-bus-bridge,transport=snb0,"
                    "ctrl=/machine/soc/i3c-hci/ctrl[0]", t);
    peer_connect(peer);

    /* ENTDAA gives the target its address; the peer is told by PID */
    qtest_writel(s, I3C0_BASE + INHOUSE_DAA_INDEX0, 1u << TGT_DA);
    resp = i3c_run_cmd(s, CMD_ATTR_ADDRASSIGN | (1u << CMD0_TID_SHIFT) |
                       (1u << CMD0_DEV_COUNT_SHIFT), 0);
    g_assert_cmphex(resp >> RESP_STATUS_SHIFT, ==, 0);
    p = peer_wait(peer->answered, MSG_I3C_DA_ASSIGNED);
    g_assert_cmphex(ldq_le_p(p->msg), ==, TGT_PID);
    g_assert_cmphex(p->msg[8], ==, TGT_DA);
    g_assert_cmpuint(p->ep, ==, EP_I3C);
    g_free(p);

    /* A private write and read */
    qtest_writel(s, I3C0_BASE + PIO_XFER_DATA, 0x44332211);
    resp = i3c_run_cmd(s, CMD_ATTR_REGULAR | (2u << CMD0_TID_SHIFT) | dev,
                       4u << CMD1_DATA_LEN_SHIFT);
    g_assert_cmphex(resp >> RESP_STATUS_SHIFT, ==, 0);
    p = peer_wait(peer->answered, MSG_I3C_WRITE);
    g_assert_cmphex(p->msg[0], ==, TGT_DA);
    g_assert_cmpuint(le32(p->msg + 1), ==, 4);
    g_assert_cmphex(le32(p->msg + 5), ==, 0x44332211);
    g_free(p);

    resp = i3c_run_cmd(s, CMD_ATTR_REGULAR | CMD0_RNW |
                       (3u << CMD0_TID_SHIFT) | dev,
                       8u << CMD1_DATA_LEN_SHIFT);
    g_assert_cmphex(resp >> RESP_STATUS_SHIFT, ==, 0);
    g_assert_cmpuint(resp & RESP_DATA_LEN_MASK, ==, 3);
    g_assert_cmphex(qtest_readl(s, I3C0_BASE + PIO_XFER_DATA) & 0xffffff, ==,
                    0x332211);

    /* The peer's in-band interrupt reaches the controller */
    p = peer_request(peer, MSG_I3C_IBI, EP_I3C, ibi, sizeof(ibi));
    g_assert_cmpint(le32(p->msg), ==, 0);
    g_free(p);
    g_assert_cmphex(qtest_readl(s, I3C0_BASE + PIO_INTR_STATUS) &
                    PIO_INTR_IBI_THLD, ==, PIO_INTR_IBI_THLD);
    g_assert_cmphex(qtest_readl(s, I3C0_BASE + PIO_IBI_PORT), ==,
                    IBI_STATUS_LAST |
                    (((uint32_t)TGT_DA << 1 | 1) << IBI_STATUS_ID_SHIFT));

    qtest_quit(s);
    peer_free(peer);
}

/* AST2700 I2C bus 1, driven in byte mode */
#define I2C_GLOBAL_CTRL         0x14c0f00c
#define I2C1_BASE               0x14c0f200
#define I2CD_FUN_CTRL           0x00
#define I2CD_INTR_CTRL          0x0c
#define I2CD_INTR_STS           0x10
#define I2CD_CMD                0x14
#define I2CD_BYTE_BUF           0x20
#define I2CD_MASTER_EN          (1u << 0)
#define I2CD_M_START_CMD        (1u << 0)
#define I2CD_M_TX_CMD           (1u << 1)
#define I2CD_M_S_RX_CMD_LAST    (1u << 4)
#define I2CD_M_STOP_CMD         (1u << 5)
#define I2CD_TX_ACK             (1u << 0)
#define I2CD_RX_DONE            (1u << 2)
#define I2C_TGT                 0x3c

static void i2c_cmd(QTestState *s, uint32_t cmd, uint32_t expect)
{
    qtest_writel(s, I2C1_BASE + I2CD_CMD, cmd);
    g_assert_cmphex(qtest_readl(s, I2C1_BASE + I2CD_INTR_STS) & expect, ==,
                    expect);
    qtest_writel(s, I2C1_BASE + I2CD_INTR_STS, 0xffffffff);
}

static void test_i2c(void)
{
    Peer *peer = peer_new();
    g_autofree char *t = peer_transport(peer, "snb0");
    QTestState *s = qtest_initf("-machine ast2700-evb %s "
                                "-device snb-i2c-bus-bridge,id=i2cb,"
                                "transport=snb0,bus=aspeed.i2c.bus.1,"
                                "addresses=0x3c,read-size=4", t);
    uint8_t alert[65] = { 1 };
    uint8_t nop[4] = {};
    Pkt *p;

    memcpy(peer->read_data, "\x5a\x6b\x7c\x8d", 4);
    qtest_irq_intercept_out_named(s, "/machine/peripheral/i2cb", "alert");
    peer_connect(peer);
    /* The old register layout, with byte commands */
    qtest_writel(s, I2C_GLOBAL_CTRL, 0);
    qtest_writel(s, I2C1_BASE + I2CD_FUN_CTRL, I2CD_MASTER_EN);
    /* Status bits show only for enabled interrupts */
    qtest_writel(s, I2C1_BASE + I2CD_INTR_CTRL, 0x7fff);

    /* Write two bytes */
    qtest_writel(s, I2C1_BASE + I2CD_BYTE_BUF, I2C_TGT << 1);
    i2c_cmd(s, I2CD_M_START_CMD | I2CD_M_TX_CMD, I2CD_TX_ACK);
    qtest_writel(s, I2C1_BASE + I2CD_BYTE_BUF, 0xa1);
    i2c_cmd(s, I2CD_M_TX_CMD, I2CD_TX_ACK);
    qtest_writel(s, I2C1_BASE + I2CD_BYTE_BUF, 0xb2);
    i2c_cmd(s, I2CD_M_TX_CMD, I2CD_TX_ACK);
    i2c_cmd(s, I2CD_M_STOP_CMD, 0);
    p = peer_wait(peer->answered, MSG_I2C_WRITE);
    g_assert_cmphex(p->msg[0], ==, I2C_TGT);
    g_assert_cmpuint(p->msg[1], ==, 1);
    g_assert_cmpuint(le32(p->msg + 2), ==, 2);
    g_assert_cmphex(p->msg[6], ==, 0xa1);
    g_assert_cmphex(p->msg[7], ==, 0xb2);
    g_assert_cmpuint(p->ep, ==, EP_I2C);
    g_free(p);

    /* Read one byte; the bridge fetched read-size bytes at the start */
    qtest_writel(s, I2C1_BASE + I2CD_BYTE_BUF, I2C_TGT << 1 | 1);
    i2c_cmd(s, I2CD_M_START_CMD | I2CD_M_TX_CMD, I2CD_TX_ACK);
    qtest_writel(s, I2C1_BASE + I2CD_CMD, I2CD_M_S_RX_CMD_LAST);
    g_assert_cmphex(qtest_readl(s, I2C1_BASE + I2CD_INTR_STS) & I2CD_RX_DONE,
                    ==, I2CD_RX_DONE);
    g_assert_cmphex((qtest_readl(s, I2C1_BASE + I2CD_BYTE_BUF) >> 8) & 0xff,
                    ==, 0x5a);
    qtest_writel(s, I2C1_BASE + I2CD_INTR_STS, 0xffffffff);
    i2c_cmd(s, I2CD_M_STOP_CMD, 0);

    /* The peer's ALERT_L drives the alert output */
    strcpy((char *)alert + 1, "ALERT_L");
    peer_send(peer->slave, MSG_GPIO_SET, peer->next_id++, 0, EP_I2C, alert,
              sizeof(alert));
    g_free(peer_request(peer, MSG_NOP, 0, nop, sizeof(nop)));
    g_assert_true(qtest_get_irq(s, 0));

    qtest_quit(s);
    peer_free(peer);
}

/*
 * One link's peer is slow to answer while QEMU waits for it with the BQL
 * held; a request on another link is still carried out at once.
 */
static void test_two_links(void)
{
    Peer *a = peer_new();
    Peer *b = peer_new();
    g_autofree char *ta = peer_transport(a, "snb0");
    g_autofree char *tb = peer_transport(b, "snb1");
    QTestState *s = qtest_initf("-machine ast2700-evb %s %s "
                                "-device snb-espi-bridge,transport=snb0,"
                                "espi-path=/machine/soc/espi[0] "
                                "-device snb-i3c-bus-bridge,transport=snb1,"
                                "ctrl=/machine/soc/i3c-hci/ctrl[0]", ta, tb);
    uint8_t reserved[1] = {};
    int64_t start;
    int i;

    peer_connect(a);
    /* Link B's enumeration holds the BQL for 2 s */
    b->enum_delay_ms = 2000;
    peer_connect(b);
    g_usleep(200 * 1000);

    for (i = 0; i < 5; i++) {
        start = g_get_monotonic_time();
        g_free(peer_request(a, MSG_ESPI_GET_STATUS, EP_ESPI, reserved, 1));
        g_assert_cmpint(g_get_monotonic_time() - start, <, 1000 * 1000);
    }

    qtest_quit(s);
    peer_free(a);
    peer_free(b);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    if (qtest_has_device("snb-transport")) {
        qtest_add_func("/snb/handshake", test_handshake);
        qtest_add_func("/snb/espi", test_espi);
        qtest_add_func("/snb/gpio", test_gpio);
        qtest_add_func("/snb/i3c", test_i3c);
        qtest_add_func("/snb/i2c", test_i2c);
        qtest_add_func("/snb/two_links", test_two_links);
    }

    return g_test_run();
}
