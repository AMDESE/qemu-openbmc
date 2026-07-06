/*
 * Simulator network bridge (SNB): transport internals
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SNB_SNB_INTERNAL_H
#define HW_SNB_SNB_INTERNAL_H

#include "qemu/event_notifier.h"
#include "qemu/queue.h"
#include "qemu/thread.h"
#include "qapi/qapi-types-sockets.h"
#include "hw/snb/snb.h"

#define SNB_MAX_ENDPOINTS           32
#define SNB_STALL_HOLD_MS_DEFAULT   100

typedef struct SnbRpc SnbRpc;

/*
 * One socket. The master channel carries QEMU's requests and the peer's
 * responses, the slave channel the peer's requests and QEMU's responses.
 * Each has a receive thread while a session is up.
 */
typedef struct SnbChannel {
    SnbServer *server;
    bool is_master;
    const char *name;
    SocketAddress *addr;

    int fd;
    bool ready;
    uint32_t session;
    EventNotifier stop;
    QemuThread thread;
    bool thread_started;

    /* Received bytes; rx_pos is the next one to parse */
    uint8_t *rx_buf;
    size_t rx_len;
    size_t rx_pos;

    /*
     * Packet under construction: a request on the master channel, owned by
     * whoever holds the server's request slot; a response on the slave
     * channel, owned by the receive thread.
     */
    uint8_t *tx_buf;
    QemuMutex write_lock;

    /* The request awaiting its response, and whether one can still come */
    QemuMutex rsp_lock;
    QemuCond rsp_cond;
    SnbRpc *rpc;
    bool rsp_live;
} SnbChannel;

typedef struct SnbEndpoint {
    bool in_use;
    uint8_t id;
    const SnbPeerOps *ops;
    void *opaque;
} SnbEndpoint;

typedef struct SnbConnectNotifier {
    bool in_use;
    void (*cb)(void *opaque);
    void *opaque;
} SnbConnectNotifier;

typedef enum SnbJobState {
    SNB_JOB_IDLE,
    SNB_JOB_PENDING,
    SNB_JOB_DONE,
} SnbJobState;

struct SnbServer {
    SnbChannel master;
    SnbChannel slave;
    uint32_t stall_hold_ms;

    SnbEndpoint endpoints[SNB_MAX_ENDPOINTS];
    SnbConnectNotifier notifiers[SNB_MAX_ENDPOINTS];
    QemuMutex table_lock;
    QEMUBH *connect_bh;

    /* One request to the peer at a time, since they share master.tx_buf */
    QemuMutex request_lock;
    QemuCond request_cond;
    bool request_busy;

    /*
     * A request from the peer that needs the BQL, handed from the slave
     * receive thread to the job thread or to a thread waiting on the peer
     * with the BQL held.
     */
    QemuMutex job_lock;
    QemuCond job_cond;
    SnbJobState job_state;
    void (*job_fn)(void *opaque);
    void *job_opaque;
    QemuThread job_thread;

    /* The session thread connects, handshakes, and reconnects */
    QemuThread session_thread;
    bool session_started;
    QemuMutex session_lock;
    QemuCond session_cond;
    uint32_t session;
    bool reconnect;
    bool tearing_down;
    bool stopping;
    bool connected;

    QLIST_ENTRY(SnbServer) link;
};

SnbServer *snb_server_new(SocketAddress *master, SocketAddress *slave,
                          uint32_t stall_hold_ms);
void snb_server_free(SnbServer *s);
void snb_server_start(SnbServer *s);

const SnbEndpoint *snb_server_find_endpoint(SnbServer *s, uint32_t id);
bool snb_server_run_locked(SnbServer *s, void (*fn)(void *opaque),
                           void *opaque);
void snb_server_run_all_jobs(void);
void snb_server_request_reconnect(SnbServer *s, SnbChannel *ch);
bool snb_server_request_begin(SnbServer *s, int64_t deadline_ms);
void snb_server_request_end(SnbServer *s);

void snb_channel_init(SnbChannel *ch, SnbServer *s, bool is_master,
                      SocketAddress *addr);
void snb_channel_cleanup(SnbChannel *ch);
bool snb_channel_connect(SnbChannel *ch, bool *cancel);
bool snb_channel_hello(SnbChannel *ch, bool *fatal, Error **errp);
void snb_channel_start(SnbChannel *ch, uint32_t session);
void snb_channel_stop(SnbChannel *ch);
void snb_channel_shutdown(SnbChannel *ch);
void snb_channel_kick(SnbChannel *ch);

/* Requests and notifications on the master channel */
void snb_channel_begin(SnbChannel *ch, uint32_t type, uint8_t endpoint_id,
                       uint32_t msg_len);
void *snb_channel_msg(SnbChannel *ch);
bool snb_channel_call(SnbChannel *ch, SnbRpc *rpc);
bool snb_channel_notify(SnbChannel *ch, uint32_t type, uint8_t endpoint_id,
                        const void *msg, uint32_t msg_len,
                        const void *data, uint32_t data_len);

/*
 * How a request's response is checked and stored: the status, then @cap
 * bytes for @buf from @data_off on. With @var_len the data is preceded by
 * its length and may be shorter; @buf takes at most @cap bytes of it.
 */
struct SnbRpc {
    uint32_t rsp_type;
    uint32_t data_off;
    bool var_len;
    uint8_t *buf;
    uint32_t cap;
    int64_t deadline_ms;

    uint32_t id;
    uint32_t len;
    bool ok;
    bool done;
};

#endif /* HW_SNB_SNB_INTERNAL_H */
