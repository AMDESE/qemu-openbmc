/*
 * Simulator network bridge (SNB): one socket of the connection
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <poll.h>
#include "qapi/clone-visitor.h"
#include "qapi/error.h"
#include "qapi/qapi-visit-sockets.h"
#include "qemu/atomic.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/sockets.h"
#include "qemu/timer.h"
#include "system/cpu-timers.h"
#include "snb-internal.h"
#include "trace.h"

const uint8_t snb_head_tag[SNB_HEAD_TAG_SIZE] = {
    0x65, 0x55, 0xeb, 0x1a, 0xd3, 0x2d, 0x43, 0x43,
    0xb4, 0x10, 0x02, 0x28, 0x84, 0xa0, 0xa5, 0x87,
};

#define SNB_MSG_OFFSET      (SNB_PKT_HEADER_SIZE + SNB_PAYLOAD_HDR_SIZE)
#define SNB_TX_BUF_SIZE     (SNB_PKT_HEADER_SIZE + SNB_MAX_PAYLOAD_SIZE)
#define SNB_RX_BUF_SIZE     (SNB_PKT_HEADER_SIZE + SNB_MAX_PAYLOAD_SIZE + \
                             4096)

/*
 * A backstop: the peer answers a request it cannot carry out with an error,
 * so this only expires if it stops answering while the link stays up.
 */
#define SNB_RPC_TIMEOUT_MS          30000
/* A deadline is on the virtual clock, which a timed wait cannot follow */
#define SNB_RPC_DEADLINE_POLL_MS    10
#define SNB_CONNECT_RETRY_MS        1000

/* Request ids; responses echo them. 0 is never used. */
static uint32_t snb_next_id = 1;

static void snb_channel_close(SnbChannel *ch);

static char *snb_addr_str(SocketAddress *addr)
{
    switch (addr->type) {
    case SOCKET_ADDRESS_TYPE_INET:
        return g_strdup_printf("%s:%s", addr->u.inet.host, addr->u.inet.port);
    case SOCKET_ADDRESS_TYPE_UNIX:
        return g_strdup_printf("unix:%s", addr->u.q_unix.path);
    default:
        return g_strdup("?");
    }
}

static void snb_put_header(uint8_t *buf, uint32_t type, uint32_t id,
                           uint32_t flags, uint32_t endpoint_id,
                           uint32_t msg_len)
{
    SnbPayloadHdr *hdr = (SnbPayloadHdr *)(buf + SNB_PKT_HEADER_SIZE);

    memcpy(buf, snb_head_tag, SNB_HEAD_TAG_SIZE);
    stl_le_p(buf + SNB_HEAD_TAG_SIZE, SNB_PAYLOAD_HDR_SIZE + msg_len);
    hdr->msg_type = cpu_to_le32(type);
    hdr->id = cpu_to_le32(id);
    hdr->flags = cpu_to_le32(flags);
    hdr->endpoint_id = cpu_to_le32(endpoint_id);
}

/* Write a whole packet; fails without a word if the link is down */
static int snb_channel_write(SnbChannel *ch, const uint8_t *buf, size_t len)
{
    const SnbPayloadHdr *hdr = (const SnbPayloadHdr *)(buf +
                                                       SNB_PKT_HEADER_SIZE);
    ssize_t ret;

    qemu_mutex_lock(&ch->write_lock);
    if (!qatomic_read(&ch->ready)) {
        qemu_mutex_unlock(&ch->write_lock);
        warn_report_once("snb: the peer is not connected; dropping traffic");
        return -ENOTCONN;
    }
    ret = qemu_write_full(ch->fd, buf, len);
    qemu_mutex_unlock(&ch->write_lock);
    if (ret != len) {
        error_report("snb: %s channel: write failed: %s", ch->name,
                     strerror(errno));
        return -EIO;
    }
    trace_snb_tx(ch->name, le32_to_cpu(hdr->id), le32_to_cpu(hdr->msg_type),
                 le32_to_cpu(hdr->flags), len);
    return 0;
}

void snb_channel_begin(SnbChannel *ch, uint32_t type, uint8_t endpoint_id,
                       uint32_t msg_len)
{
    assert(msg_len <= SNB_MAX_DATA_SIZE);
    snb_put_header(ch->tx_buf, type, qatomic_fetch_inc(&snb_next_id), 0,
                   endpoint_id, msg_len);
}

void *snb_channel_msg(SnbChannel *ch)
{
    return ch->tx_buf + SNB_MSG_OFFSET;
}

/* Answer the request @req with the response built after the header */
static int snb_channel_respond(SnbChannel *ch, const SnbPayloadHdr *req,
                               uint32_t type, uint32_t msg_len)
{
    snb_put_header(ch->tx_buf, type, le32_to_cpu(req->id),
                   SNB_PKT_FLAG_RESPONSE, le32_to_cpu(req->endpoint_id),
                   msg_len);
    return snb_channel_write(ch, ch->tx_buf, SNB_MSG_OFFSET + msg_len);
}

bool snb_channel_notify(SnbChannel *ch, uint32_t type, uint8_t endpoint_id,
                        const void *msg, uint32_t msg_len,
                        const void *data, uint32_t data_len)
{
    size_t len = SNB_MSG_OFFSET + msg_len + data_len;
    g_autofree uint8_t *buf = g_malloc(len);

    snb_put_header(buf, type, qatomic_fetch_inc(&snb_next_id), 0,
                   endpoint_id, msg_len + data_len);
    memcpy(buf + SNB_MSG_OFFSET, msg, msg_len);
    if (data_len) {
        memcpy(buf + SNB_MSG_OFFSET + msg_len, data, data_len);
    }
    return snb_channel_write(ch, buf, len) == 0;
}

/* Receiving */

/* Make @need bytes available from rx_pos on */
static int snb_channel_fill(SnbChannel *ch, size_t need)
{
    struct pollfd fds[2] = {
        { .fd = event_notifier_get_fd(&ch->stop), .events = POLLIN },
        { .fd = ch->fd, .events = POLLIN },
    };

    assert(need <= SNB_RX_BUF_SIZE);
    if (ch->rx_pos + need > SNB_RX_BUF_SIZE) {
        memmove(ch->rx_buf, ch->rx_buf + ch->rx_pos,
                ch->rx_len - ch->rx_pos);
        ch->rx_len -= ch->rx_pos;
        ch->rx_pos = 0;
    }

    while (ch->rx_len - ch->rx_pos < need) {
        ssize_t n;

        if (poll(fds, ARRAY_SIZE(fds), -1) < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -errno;
        }
        if (fds[0].revents) {
            return -ECANCELED;
        }
        n = read(ch->fd, ch->rx_buf + ch->rx_len,
                 SNB_RX_BUF_SIZE - ch->rx_len);
        if (n == 0) {
            return -ECONNRESET;
        }
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            return -errno;
        }
        ch->rx_len += n;
    }
    return 0;
}

/* Skip to the byte after the next head tag */
static int snb_channel_find_head(SnbChannel *ch)
{
    size_t skipped = 0;

    for (;;) {
        int ret = snb_channel_fill(ch, SNB_HEAD_TAG_SIZE);
        size_t avail, i;

        if (ret) {
            return ret;
        }
        avail = ch->rx_len - ch->rx_pos;
        for (i = 0; i + SNB_HEAD_TAG_SIZE <= avail; i++) {
            if (!memcmp(ch->rx_buf + ch->rx_pos + i, snb_head_tag,
                        SNB_HEAD_TAG_SIZE)) {
                if (skipped + i) {
                    trace_snb_resync(ch->name, skipped + i);
                }
                ch->rx_pos += i + SNB_HEAD_TAG_SIZE;
                return 0;
            }
        }
        /* Keep what could still be the start of a head tag */
        skipped += avail - (SNB_HEAD_TAG_SIZE - 1);
        ch->rx_pos += avail - (SNB_HEAD_TAG_SIZE - 1);
    }
}

typedef int (*SnbHandler)(SnbChannel *ch, const SnbPayloadHdr *hdr,
                          const uint8_t *msg, uint32_t len);

typedef enum SnbSizeRule {
    SNB_SIZE_FIXED,         /* exactly .size */
    SNB_SIZE_RSP,           /* at least the status; the request checks it */
    SNB_SIZE_FLASH,         /* SnbMsgEspiFlashCycle and its data */
    SNB_SIZE_PC,            /* SnbMsgEspiPcCycle and its data */
} SnbSizeRule;

typedef struct SnbMsgDesc {
    uint32_t type;
    bool request;           /* sent by the peer, on the slave channel */
    bool bql;
    SnbSizeRule rule;
    uint32_t size;
    SnbHandler handle;
} SnbMsgDesc;

static bool snb_msg_size_ok(const SnbMsgDesc *d, const uint8_t *msg,
                            uint32_t len)
{
    uint32_t data_len = 0;

    switch (d->rule) {
    case SNB_SIZE_FIXED:
        return len == d->size;
    case SNB_SIZE_RSP:
        return len >= sizeof(SnbMsgStatus);
    case SNB_SIZE_FLASH: {
        const SnbMsgEspiFlashCycle *m = (const void *)msg;

        if (len < sizeof(*m)) {
            return false;
        }
        if (m->cyc == SNB_ESPI_FLASH_WRITE) {
            data_len = le32_to_cpu(m->len);
        }
        return data_len <= SNB_ESPI_MAX_PLD && len == sizeof(*m) + data_len;
    }
    case SNB_SIZE_PC: {
        const SnbMsgEspiPcCycle *m = (const void *)msg;

        if (len < sizeof(*m)) {
            return false;
        }
        if (snb_espi_pc_has_data(m->cyc)) {
            data_len = le32_to_cpu(m->len);
        }
        if (!snb_espi_pc_has_len(m->cyc) && m->len) {
            return false;
        }
        return data_len <= SNB_ESPI_MAX_PLD && len == sizeof(*m) + data_len;
    }
    }
    return false;
}

/* Responses to QEMU's requests */

static bool snb_rpc_expired(const SnbRpc *rpc)
{
    return rpc->deadline_ms &&
           qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) >= rpc->deadline_ms;
}

static bool snb_rpc_take(SnbRpc *rpc, const uint8_t *msg, uint32_t len)
{
    uint32_t data_len = rpc->cap;

    if (len < rpc->data_off) {
        return false;
    }
    if (rpc->var_len) {
        data_len = ldl_le_p(msg + rpc->data_off - sizeof(uint32_t));
    }
    if (len - rpc->data_off != data_len) {
        return false;
    }
    rpc->len = MIN(data_len, rpc->cap);
    if (rpc->len) {
        memcpy(rpc->buf, msg + rpc->data_off, rpc->len);
    }
    return true;
}

/*
 * Hand a response to the request waiting for it. A response nobody waits
 * for, such as the late answer to a request that gave up, is dropped. A
 * nonzero status fails the request, whatever follows it.
 */
static int snb_handle_rpc_rsp(SnbChannel *ch, const SnbPayloadHdr *hdr,
                              const uint8_t *msg, uint32_t len)
{
    uint32_t id = le32_to_cpu(hdr->id);
    uint32_t type = le32_to_cpu(hdr->msg_type);
    SnbRpc *rpc;

    qemu_mutex_lock(&ch->rsp_lock);
    rpc = ch->rpc;
    if (!rpc || rpc->id != id || rpc->rsp_type != type) {
        qemu_mutex_unlock(&ch->rsp_lock);
        trace_snb_rpc_drop(id, type);
        return 0;
    }
    if (!ldl_le_p(msg)) {
        rpc->ok = snb_rpc_take(rpc, msg, len);
        if (!rpc->ok) {
            warn_report("snb: response %u (type 0x%x) does not match its "
                        "request", id, type);
        }
    }
    rpc->done = true;
    ch->rpc = NULL;
    qemu_cond_broadcast(&ch->rsp_cond);
    qemu_mutex_unlock(&ch->rsp_lock);
    return 0;
}

/* The answer to a notification, which nothing waits for */
static int snb_handle_ignore(SnbChannel *ch, const SnbPayloadHdr *hdr,
                             const uint8_t *msg, uint32_t len)
{
    return 0;
}

/* Requests from the peer */

static const SnbEndpoint *snb_endpoint(SnbChannel *ch,
                                       const SnbPayloadHdr *hdr,
                                       const char *what)
{
    uint32_t id = le32_to_cpu(hdr->endpoint_id);
    const SnbEndpoint *ep = snb_server_find_endpoint(ch->server, id);

    if (!ep) {
        qemu_log_mask(LOG_GUEST_ERROR, "snb: %s for unknown endpoint %u\n",
                      what, id);
    }
    return ep;
}

static int snb_respond_status(SnbChannel *ch, const SnbPayloadHdr *hdr,
                              bool ok)
{
    SnbMsgStatus *rsp = snb_channel_msg(ch);

    rsp->status = cpu_to_le32(ok ? 0 : -1);
    return snb_channel_respond(ch, hdr, SNB_RSP(le32_to_cpu(hdr->msg_type)),
                               sizeof(*rsp));
}

static int snb_handle_nop(SnbChannel *ch, const SnbPayloadHdr *hdr,
                          const uint8_t *msg, uint32_t len)
{
    return snb_respond_status(ch, hdr, true);
}

static int snb_handle_i3c_ibi(SnbChannel *ch, const SnbPayloadHdr *hdr,
                              const uint8_t *msg, uint32_t len)
{
    const SnbMsgI3cIbi *m = (const void *)msg;
    const SnbEndpoint *ep = snb_endpoint(ch, hdr, "I3C IBI");
    bool ok = false;

    if (ep && ep->ops->i3c_ibi) {
        ok = ep->ops->i3c_ibi(ep->opaque, m->da, m->mdb, m->has_mdb);
    }
    return snb_respond_status(ch, hdr, ok);
}

/* Not answered */
static int snb_handle_gpio_set(SnbChannel *ch, const SnbPayloadHdr *hdr,
                               const uint8_t *msg, uint32_t len)
{
    const SnbMsgGpioSet *m = (const void *)msg;
    const SnbEndpoint *ep = snb_endpoint(ch, hdr, "GPIO_SET");
    char name[SNB_GPIO_NAME_MAX + 1];

    memcpy(name, m->name, SNB_GPIO_NAME_MAX);
    name[SNB_GPIO_NAME_MAX] = '\0';
    if (ep && ep->ops->gpio_set) {
        ep->ops->gpio_set(ep->opaque, name, m->level);
    }
    return 0;
}

/* Answer an eSPI cycle with the completion built in tx_buf */
static int snb_respond_completion(SnbChannel *ch, const SnbPayloadHdr *hdr,
                                  bool ok, uint8_t cyc, uint8_t tag,
                                  uint32_t espi_status, uint32_t len)
{
    SnbMsgEspiCompletion *rsp = snb_channel_msg(ch);

    if (!ok) {
        len = 0;
    }
    rsp->status = cpu_to_le32(ok ? 0 : -1);
    rsp->cyc = cyc;
    rsp->tag = tag;
    rsp->espi_status = cpu_to_le32(espi_status);
    rsp->len = cpu_to_le32(len);
    return snb_channel_respond(ch, hdr, SNB_RSP(le32_to_cpu(hdr->msg_type)),
                               sizeof(*rsp) + len);
}

static uint8_t *snb_completion_data(SnbChannel *ch)
{
    return ch->tx_buf + SNB_MSG_OFFSET + sizeof(SnbMsgEspiCompletion);
}

static int snb_handle_espi_flash_cycle(SnbChannel *ch,
                                       const SnbPayloadHdr *hdr,
                                       const uint8_t *msg, uint32_t len)
{
    const SnbMsgEspiFlashCycle *m = (const void *)msg;
    const SnbEndpoint *ep = snb_endpoint(ch, hdr, "eSPI flash cycle");
    uint32_t rsp_len = SNB_ESPI_MAX_PLD;
    uint32_t espi_status = 0;
    uint8_t rsp_cyc = 0;
    bool ok = false;

    if (ep && ep->ops->espi_flash_cycle) {
        ok = ep->ops->espi_flash_cycle(ep->opaque, m->cyc, m->tag,
                                       le32_to_cpu(m->addr),
                                       le32_to_cpu(m->len), m->data,
                                       &rsp_cyc, snb_completion_data(ch),
                                       &rsp_len, &espi_status);
    }
    return snb_respond_completion(ch, hdr, ok, rsp_cyc, m->tag, espi_status,
                                  rsp_len);
}

static int snb_handle_espi_flash_completion(SnbChannel *ch,
                                            const SnbPayloadHdr *hdr,
                                            const uint8_t *msg, uint32_t len)
{
    const SnbEndpoint *ep = snb_endpoint(ch, hdr, "eSPI flash completion");
    uint32_t rsp_len = SNB_ESPI_MAX_PLD;
    uint32_t espi_status = 0;
    uint8_t rsp_cyc = 0, rsp_tag = 0;
    bool ok = false;

    if (ep && ep->ops->espi_flash_get_completion) {
        ok = ep->ops->espi_flash_get_completion(ep->opaque, &rsp_cyc,
                                                &rsp_tag,
                                                snb_completion_data(ch),
                                                &rsp_len, &espi_status);
    }
    return snb_respond_completion(ch, hdr, ok, rsp_cyc, rsp_tag, espi_status,
                                  rsp_len);
}

static int snb_handle_espi_pc_cycle(SnbChannel *ch, const SnbPayloadHdr *hdr,
                                    const uint8_t *msg, uint32_t len)
{
    const SnbMsgEspiPcCycle *m = (const void *)msg;
    const SnbEndpoint *ep = snb_endpoint(ch, hdr, "eSPI peripheral cycle");
    uint32_t rsp_len = SNB_ESPI_MAX_PLD;
    uint32_t espi_status = 0;
    uint8_t rsp_cyc = 0;
    bool ok = false;

    if (ep && ep->ops->espi_pc_cycle) {
        ok = ep->ops->espi_pc_cycle(ep->opaque, m->cyc, m->tag,
                                    le64_to_cpu(m->addr),
                                    le32_to_cpu(m->len), m->data,
                                    m->msg_code, &rsp_cyc,
                                    snb_completion_data(ch), &rsp_len,
                                    &espi_status);
    }
    return snb_respond_completion(ch, hdr, ok, rsp_cyc, m->tag, espi_status,
                                  rsp_len);
}

/* Not answered */
static int snb_handle_espi_vw_set(SnbChannel *ch, const SnbPayloadHdr *hdr,
                                  const uint8_t *msg, uint32_t len)
{
    const SnbMsgEspiVw *m = (const void *)msg;
    const SnbEndpoint *ep = snb_endpoint(ch, hdr, "eSPI VW_SET");

    if (ep && ep->ops->espi_vw_set) {
        ep->ops->espi_vw_set(ep->opaque, m->index, le32_to_cpu(m->value));
    }
    return 0;
}

/* Not answered */
static int snb_handle_espi_reset(SnbChannel *ch, const SnbPayloadHdr *hdr,
                                 const uint8_t *msg, uint32_t len)
{
    const SnbMsgEspiReset *m = (const void *)msg;
    const SnbEndpoint *ep = snb_endpoint(ch, hdr, "eSPI reset");

    if (ep && ep->ops->espi_reset) {
        ep->ops->espi_reset(ep->opaque, m->asserted);
    }
    return 0;
}

static int snb_handle_espi_get_status(SnbChannel *ch,
                                      const SnbPayloadHdr *hdr,
                                      const uint8_t *msg, uint32_t len)
{
    const SnbEndpoint *ep = snb_endpoint(ch, hdr, "eSPI GET_STATUS");
    SnbMsgEspiStatusRsp *rsp = snb_channel_msg(ch);
    uint32_t espi_status = 0;
    bool ok = false;

    if (ep && ep->ops->espi_get_status) {
        ok = ep->ops->espi_get_status(ep->opaque, &espi_status);
    }
    rsp->status = cpu_to_le32(ok ? 0 : -1);
    rsp->espi_status = cpu_to_le32(espi_status);
    return snb_channel_respond(ch, hdr, SNB_RSP(SNB_MSG_ESPI_GET_STATUS),
                               sizeof(*rsp));
}

/* GET_CONFIG and SET_CONFIG both answer with the dword after the access */
static int snb_handle_espi_config(SnbChannel *ch, const SnbPayloadHdr *hdr,
                                  const uint8_t *msg, uint32_t len)
{
    const SnbMsgEspiConfig *m = (const void *)msg;
    bool is_set = le32_to_cpu(hdr->msg_type) == SNB_MSG_ESPI_SET_CONFIG;
    const SnbEndpoint *ep = snb_endpoint(ch, hdr, "eSPI configuration");
    SnbMsgEspiConfigRsp *rsp = snb_channel_msg(ch);
    uint16_t addr = le16_to_cpu(m->addr);
    uint32_t value = 0, espi_status = 0;
    bool ok = false;

    if (ep && ep->ops->espi_get_config) {
        ok = true;
        if (is_set) {
            ok = ep->ops->espi_set_config &&
                 ep->ops->espi_set_config(ep->opaque, addr,
                                          le32_to_cpu(m->value));
        }
        ok = ep->ops->espi_get_config(ep->opaque, addr, &value) && ok;
        if (ep->ops->espi_get_status) {
            ep->ops->espi_get_status(ep->opaque, &espi_status);
        }
    }
    rsp->status = cpu_to_le32(ok ? 0 : -1);
    rsp->value = cpu_to_le32(value);
    rsp->espi_status = cpu_to_le32(espi_status);
    return snb_channel_respond(ch, hdr, SNB_RSP(le32_to_cpu(hdr->msg_type)),
                               sizeof(*rsp));
}

static const SnbMsgDesc snb_msgs[] = {
    /* From the peer */
    { SNB_MSG_NOP, true, false, SNB_SIZE_FIXED, sizeof(SnbMsgNop),
      snb_handle_nop },
    { SNB_MSG_I3C_IBI, true, true, SNB_SIZE_FIXED, sizeof(SnbMsgI3cIbi),
      snb_handle_i3c_ibi },
    { SNB_MSG_GPIO_SET, true, true, SNB_SIZE_FIXED, sizeof(SnbMsgGpioSet),
      snb_handle_gpio_set },
    { SNB_MSG_ESPI_FLASH_CYCLE, true, true, SNB_SIZE_FLASH, 0,
      snb_handle_espi_flash_cycle },
    { SNB_MSG_ESPI_FLASH_COMPLETION, true, true, SNB_SIZE_FIXED,
      sizeof(SnbMsgReserved), snb_handle_espi_flash_completion },
    { SNB_MSG_ESPI_VW_SET, true, true, SNB_SIZE_FIXED, sizeof(SnbMsgEspiVw),
      snb_handle_espi_vw_set },
    { SNB_MSG_ESPI_RESET, true, true, SNB_SIZE_FIXED,
      sizeof(SnbMsgEspiReset), snb_handle_espi_reset },
    { SNB_MSG_ESPI_GET_STATUS, true, true, SNB_SIZE_FIXED,
      sizeof(SnbMsgReserved), snb_handle_espi_get_status },
    { SNB_MSG_ESPI_GET_CONFIG, true, true, SNB_SIZE_FIXED,
      sizeof(SnbMsgEspiConfig), snb_handle_espi_config },
    { SNB_MSG_ESPI_SET_CONFIG, true, true, SNB_SIZE_FIXED,
      sizeof(SnbMsgEspiConfig), snb_handle_espi_config },
    { SNB_MSG_ESPI_PC_CYCLE, true, true, SNB_SIZE_PC, 0,
      snb_handle_espi_pc_cycle },

    /* Answers to QEMU's requests */
    { SNB_RSP(SNB_MSG_BUS_REGISTER), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_I2C_WRITE), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_I2C_READ), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_I3C_WRITE), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_I3C_READ), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_I3C_CCC), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_I3C_ENUM_I2C), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_I3C_ENUM_I3C), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_I3C_DA_ASSIGNED), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_I3C_I2C_WRITE), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_I3C_I2C_READ), false, false, SNB_SIZE_RSP, 0,
      snb_handle_rpc_rsp },
    { SNB_RSP(SNB_MSG_GPIO_SET), false, false, SNB_SIZE_FIXED,
      sizeof(SnbMsgStatus), snb_handle_ignore },
    { SNB_RSP(SNB_MSG_ESPI_VW_NOTIFY), false, false, SNB_SIZE_FIXED,
      sizeof(SnbMsgStatus), snb_handle_ignore },
    { SNB_RSP(SNB_MSG_ESPI_ALERT), false, false, SNB_SIZE_FIXED,
      sizeof(SnbMsgStatus), snb_handle_ignore },
    { SNB_RSP(SNB_MSG_ESPI_PC_NOTIFY), false, false, SNB_SIZE_FIXED,
      sizeof(SnbMsgStatus), snb_handle_ignore },
};

typedef struct SnbJobArgs {
    SnbChannel *ch;
    const SnbMsgDesc *desc;
    const SnbPayloadHdr *hdr;
    const uint8_t *msg;
    uint32_t len;
    int ret;
} SnbJobArgs;

static void snb_channel_job(void *opaque)
{
    SnbJobArgs *a = opaque;

    a->ret = a->desc->handle(a->ch, a->hdr, a->msg, a->len);
}

static int snb_channel_dispatch(SnbChannel *ch, const uint8_t *payload,
                                uint32_t size)
{
    const SnbPayloadHdr *hdr = (const SnbPayloadHdr *)payload;
    const uint8_t *msg = payload + SNB_PAYLOAD_HDR_SIZE;
    uint32_t len = size - SNB_PAYLOAD_HDR_SIZE;
    uint32_t type = le32_to_cpu(hdr->msg_type);
    const SnbMsgDesc *d = NULL;
    SnbJobArgs args;
    int i;

    trace_snb_rx(ch->name, le32_to_cpu(hdr->id), type,
                 le32_to_cpu(hdr->flags), size);

    for (i = 0; i < ARRAY_SIZE(snb_msgs); i++) {
        if (snb_msgs[i].type == type && snb_msgs[i].request != ch->is_master) {
            d = &snb_msgs[i];
            break;
        }
    }
    if (!d) {
        qemu_log_mask(LOG_UNIMP, "snb: %s channel: ignoring message type "
                      "0x%x\n", ch->name, type);
        return 0;
    }
    if (!snb_msg_size_ok(d, msg, len)) {
        error_report("snb: %s channel: message type 0x%x has a wrong size "
                     "(%u bytes)", ch->name, type, len);
        return -EPROTO;
    }
    if (!d->bql) {
        return d->handle(ch, hdr, msg, len);
    }

    args = (SnbJobArgs) {
        .ch = ch, .desc = d, .hdr = hdr, .msg = msg, .len = len,
    };
    if (!snb_server_run_locked(ch->server, snb_channel_job, &args)) {
        return -ETIMEDOUT;
    }
    return args.ret;
}

static int snb_channel_receive(SnbChannel *ch)
{
    uint32_t size;
    int ret;

    ret = snb_channel_find_head(ch);
    if (ret) {
        return ret;
    }
    ret = snb_channel_fill(ch, sizeof(uint32_t));
    if (ret) {
        return ret;
    }
    size = ldl_le_p(ch->rx_buf + ch->rx_pos);
    if (size < SNB_MIN_PAYLOAD_SIZE || size > SNB_MAX_PAYLOAD_SIZE) {
        error_report("snb: %s channel: bad payload size %u", ch->name, size);
        return -EPROTO;
    }
    ch->rx_pos += sizeof(uint32_t);
    ret = snb_channel_fill(ch, size);
    if (ret) {
        return ret;
    }
    ret = snb_channel_dispatch(ch, ch->rx_buf + ch->rx_pos, size);
    ch->rx_pos += size;
    return ret;
}

/* No response can come any more: fail the request waiting for one */
static void snb_channel_fail_rpc(SnbChannel *ch)
{
    qemu_mutex_lock(&ch->rsp_lock);
    ch->rsp_live = false;
    if (ch->rpc) {
        ch->rpc->done = true;
        ch->rpc = NULL;
    }
    qemu_cond_broadcast(&ch->rsp_cond);
    qemu_mutex_unlock(&ch->rsp_lock);
}

static void *snb_channel_thread(void *opaque)
{
    SnbChannel *ch = opaque;
    int ret;

    do {
        ret = snb_channel_receive(ch);
    } while (!ret);

    snb_channel_fail_rpc(ch);
    if (ret == -ECANCELED) {
        /* Whoever stopped the channel tears it down */
        return NULL;
    }
    /*
     * The peer closed the link, or the stream no longer parses. The peer
     * ends the whole session when either socket goes, so start a new one.
     */
    trace_snb_link_lost(ch->name, ret);
    snb_channel_close(ch);
    snb_server_request_reconnect(ch->server, ch);
    return NULL;
}

/* Requests on the master channel */

/*
 * Wait for the response to @rpc. A thread holding the BQL runs the peer's
 * requests meanwhile, since the peer may need one answered first; once the
 * wait exceeds stall_hold_ms the guest clock is held, so that a slow peer
 * does not look like a device timeout to the guest.
 */
static bool snb_channel_wait(SnbChannel *ch, SnbRpc *rpc)
{
    const int64_t start = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
    const int64_t end = start + SNB_RPC_TIMEOUT_MS;
    const int64_t hold_ms = ch->server->stall_hold_ms;
    int64_t held_at = 0;
    bool done;

    qemu_mutex_lock(&ch->rsp_lock);
    while (!rpc->done) {
        int64_t now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
        int64_t left = end - now;

        if (left <= 0 || snb_rpc_expired(rpc)) {
            break;
        }
        if (hold_ms && !held_at && now - start >= hold_ms) {
            cpu_hold_ticks();
            held_at = now;
            trace_snb_rpc_hold(rpc->id, rpc->rsp_type, now - start);
        }
        if (bql_locked()) {
            qemu_cond_timedwait(&ch->rsp_cond, &ch->rsp_lock, 1);
            qemu_mutex_unlock(&ch->rsp_lock);
            snb_server_run_all_jobs();
            qemu_mutex_lock(&ch->rsp_lock);
            continue;
        }
        if (rpc->deadline_ms) {
            left = MIN(left, SNB_RPC_DEADLINE_POLL_MS);
        }
        if (hold_ms && !held_at) {
            left = MIN(left, start + hold_ms - now);
        }
        qemu_cond_timedwait(&ch->rsp_cond, &ch->rsp_lock, left);
    }
    done = rpc->done;
    if (ch->rpc == rpc) {
        ch->rpc = NULL;
    }
    qemu_mutex_unlock(&ch->rsp_lock);

    if (held_at) {
        cpu_release_ticks();
        trace_snb_rpc_release(rpc->id, done,
                              qemu_clock_get_ms(QEMU_CLOCK_REALTIME) -
                              held_at);
    }
    return done;
}

bool snb_channel_call(SnbChannel *ch, SnbRpc *rpc)
{
    const SnbPayloadHdr *hdr = (const SnbPayloadHdr *)(ch->tx_buf +
                                                       SNB_PKT_HEADER_SIZE);
    size_t len = SNB_PKT_HEADER_SIZE + ldl_le_p(ch->tx_buf +
                                                SNB_HEAD_TAG_SIZE);
    bool live;

    if (snb_rpc_expired(rpc)) {
        return false;
    }
    rpc->id = le32_to_cpu(hdr->id);

    qemu_mutex_lock(&ch->rsp_lock);
    assert(!ch->rpc);
    live = ch->rsp_live;
    if (live) {
        ch->rpc = rpc;
    }
    qemu_mutex_unlock(&ch->rsp_lock);
    if (!live) {
        warn_report_once("snb: the peer is not connected; dropping traffic");
        return false;
    }

    if (snb_channel_write(ch, ch->tx_buf, len) == 0 &&
        !snb_channel_wait(ch, rpc)) {
        if (snb_rpc_expired(rpc)) {
            trace_snb_rpc_deadline(rpc->id, rpc->rsp_type);
        } else {
            warn_report("snb: no answer to request %u in %d ms", rpc->id,
                        SNB_RPC_TIMEOUT_MS);
        }
    }

    qemu_mutex_lock(&ch->rsp_lock);
    if (ch->rpc == rpc) {
        ch->rpc = NULL;
    }
    qemu_mutex_unlock(&ch->rsp_lock);
    return rpc->ok;
}

/* Wake a waiter so that it runs a job the slave channel just queued */
void snb_channel_kick(SnbChannel *ch)
{
    qemu_mutex_lock(&ch->rsp_lock);
    qemu_cond_broadcast(&ch->rsp_cond);
    qemu_mutex_unlock(&ch->rsp_lock);
}

/* Connection */

void snb_channel_init(SnbChannel *ch, SnbServer *s, bool is_master,
                      SocketAddress *addr)
{
    ch->server = s;
    ch->is_master = is_master;
    ch->name = is_master ? "master" : "slave";
    ch->addr = QAPI_CLONE(SocketAddress, addr);
    ch->fd = -1;
    ch->rx_buf = g_malloc(SNB_RX_BUF_SIZE);
    ch->tx_buf = g_malloc0(SNB_TX_BUF_SIZE);
    qemu_mutex_init(&ch->write_lock);
    qemu_mutex_init(&ch->rsp_lock);
    qemu_cond_init(&ch->rsp_cond);
    event_notifier_init(&ch->stop, false);
}

void snb_channel_cleanup(SnbChannel *ch)
{
    snb_channel_stop(ch);
    event_notifier_cleanup(&ch->stop);
    qemu_cond_destroy(&ch->rsp_cond);
    qemu_mutex_destroy(&ch->rsp_lock);
    qemu_mutex_destroy(&ch->write_lock);
    g_free(ch->tx_buf);
    g_free(ch->rx_buf);
    qapi_free_SocketAddress(ch->addr);
}

/* Connect, retrying until it works or *@cancel is set */
bool snb_channel_connect(SnbChannel *ch, bool *cancel)
{
    g_autofree char *where = snb_addr_str(ch->addr);
    unsigned retries = 0;

    for (;;) {
        int fd, i;

        if (qatomic_read(cancel)) {
            return false;
        }
        fd = socket_connect(ch->addr, NULL);
        if (fd >= 0) {
            if (ch->addr->type == SOCKET_ADDRESS_TYPE_INET) {
                socket_set_nodelay(fd);
            }
            qatomic_set(&ch->fd, fd);
            break;
        }
        if (!retries) {
            warn_report("snb: waiting for the peer on %s (%s channel)",
                        where, ch->name);
        }
        trace_snb_connect_wait(ch->name, where, ++retries);
        /* Wait SNB_CONNECT_RETRY_MS, in tenths so that a cancel is seen */
        for (i = 0; i < 10 && !qatomic_read(cancel); i++) {
            g_usleep(SNB_CONNECT_RETRY_MS * 100);
        }
    }
    trace_snb_connected(ch->name, where);
    return true;
}

/*
 * Exchange HELLO; the receive thread is not running yet. *@fatal tells a
 * version the peer cannot talk to from any other failure.
 */
bool snb_channel_hello(SnbChannel *ch, bool *fatal, Error **errp)
{
    uint8_t buf[SNB_MSG_OFFSET + sizeof(SnbMsgHello)];
    SnbMsgHello *hello = (SnbMsgHello *)(buf + SNB_MSG_OFFSET);
    const SnbPayloadHdr *hdr = (SnbPayloadHdr *)(buf + SNB_PKT_HEADER_SIZE);
    size_t got = 0;
    unsigned major, minor;

    *fatal = false;
    snb_put_header(buf, SNB_MSG_HELLO, 1, 0, 0, sizeof(*hello));
    hello->version_major = cpu_to_le16(SNB_VERSION_MAJOR);
    hello->version_minor = cpu_to_le16(SNB_VERSION_MINOR);
    hello->caps = 0;
    if (qemu_write_full(ch->fd, buf, sizeof(buf)) != sizeof(buf)) {
        error_setg_errno(errp, errno, "sending HELLO failed");
        return false;
    }

    while (got < sizeof(buf)) {
        ssize_t n = read(ch->fd, buf + got, sizeof(buf) - got);

        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            error_setg(errp, "no HELLO from the peer");
            return false;
        }
        got += n;
    }
    if (memcmp(buf, snb_head_tag, SNB_HEAD_TAG_SIZE) ||
        ldl_le_p(buf + SNB_HEAD_TAG_SIZE) !=
            SNB_PAYLOAD_HDR_SIZE + sizeof(*hello) ||
        le32_to_cpu(hdr->msg_type) != SNB_MSG_HELLO) {
        error_setg(errp, "the peer did not answer with HELLO");
        return false;
    }

    major = le16_to_cpu(hello->version_major);
    minor = le16_to_cpu(hello->version_minor);
    trace_snb_hello(ch->name, major, minor);
    if (major != SNB_VERSION_MAJOR || minor < SNB_VERSION_MINOR_MIN) {
        *fatal = true;
        error_setg(errp, "the peer speaks version %u.%u; version %u.%u or "
                   "a later minor version is needed", major, minor,
                   SNB_VERSION_MAJOR, SNB_VERSION_MINOR_MIN);
        return false;
    }
    return true;
}

static void snb_channel_join(SnbChannel *ch)
{
    if (ch->thread_started) {
        qemu_thread_join(&ch->thread);
        ch->thread_started = false;
    }
}

/* Start receiving for session @session */
void snb_channel_start(SnbChannel *ch, uint32_t session)
{
    snb_channel_join(ch);
    /* A stop meant for the last session must not end this one */
    event_notifier_test_and_clear(&ch->stop);
    ch->rx_len = ch->rx_pos = 0;
    ch->session = session;
    qemu_mutex_lock(&ch->rsp_lock);
    ch->rsp_live = true;
    qemu_mutex_unlock(&ch->rsp_lock);
    qatomic_set(&ch->ready, true);
    qemu_thread_create(&ch->thread, "snb-recv", snb_channel_thread, ch,
                       QEMU_THREAD_JOINABLE);
    ch->thread_started = true;
}

/*
 * Close the socket. The receive thread and the session thread can both get
 * here for the same socket; only the one that takes it closes it.
 */
static void snb_channel_close(SnbChannel *ch)
{
    int fd;

    qatomic_set(&ch->ready, false);
    fd = qatomic_xchg(&ch->fd, -1);
    if (fd < 0) {
        return;
    }
    /*
     * A writer blocked on a peer that stopped reading fails once the socket
     * is shut down; closing under write_lock then keeps every writer off a
     * closed or reused descriptor.
     */
    shutdown(fd, SHUT_RDWR);
    qemu_mutex_lock(&ch->write_lock);
    close(fd);
    qemu_mutex_unlock(&ch->write_lock);
    trace_snb_disconnected(ch->name);
}

/* Stop the receive thread, if one runs, and close the socket */
void snb_channel_stop(SnbChannel *ch)
{
    if (qatomic_read(&ch->ready)) {
        event_notifier_set(&ch->stop);
        snb_channel_join(ch);
    }
    snb_channel_close(ch);
    snb_channel_join(ch);
}

/* Fail a blocking read or write on the socket; the owner still closes it */
void snb_channel_shutdown(SnbChannel *ch)
{
    int fd = qatomic_read(&ch->fd);

    if (fd >= 0) {
        shutdown(fd, SHUT_RDWR);
    }
}
