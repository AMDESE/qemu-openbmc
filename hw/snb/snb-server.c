/*
 * Simulator network bridge (SNB): the connection to the peer
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/atomic.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "snb-internal.h"
#include "trace.h"

/* How long the slave channel waits for a job to run before giving up */
#define SNB_JOB_TIMEOUT_MS          5000
#define SNB_RETRY_PAUSE_MS          1000

/* Every server, for waiters that run the jobs of all of them */
static QLIST_HEAD(, SnbServer) snb_servers =
    QLIST_HEAD_INITIALIZER(snb_servers);

/* Jobs: requests from the peer that need the BQL */

/* Run the pending job of @s; the caller holds the BQL */
static void snb_server_run_job(SnbServer *s)
{
    qemu_mutex_lock(&s->job_lock);
    if (s->job_state == SNB_JOB_PENDING) {
        s->job_fn(s->job_opaque);
        s->job_state = SNB_JOB_DONE;
        qemu_cond_broadcast(&s->job_cond);
    }
    qemu_mutex_unlock(&s->job_lock);
}

/*
 * A thread that holds the BQL while it waits on a peer runs the pending
 * jobs of every server: the job threads cannot take the BQL meanwhile,
 * and another peer may be waiting on one of them.
 */
void snb_server_run_all_jobs(void)
{
    SnbServer *s;

    assert(bql_locked());
    QLIST_FOREACH(s, &snb_servers, link) {
        snb_server_run_job(s);
    }
}

static void *snb_server_job_thread(void *opaque)
{
    SnbServer *s = opaque;

    for (;;) {
        qemu_mutex_lock(&s->job_lock);
        while (s->job_state != SNB_JOB_PENDING &&
               !qatomic_read(&s->stopping)) {
            qemu_cond_wait(&s->job_cond, &s->job_lock);
        }
        qemu_mutex_unlock(&s->job_lock);
        if (qatomic_read(&s->stopping)) {
            return NULL;
        }
        bql_lock();
        snb_server_run_job(s);
        bql_unlock();
    }
}

/*
 * Run @fn with the BQL held, on the job thread or on a thread that holds
 * the BQL while it waits on a peer, and wait for it. The slave channel never
 * takes the BQL itself: its holder may be waiting for the peer, and the
 * peer for this very answer. Returns false if @fn did not run in time.
 */
bool snb_server_run_locked(SnbServer *s, void (*fn)(void *opaque),
                           void *opaque)
{
    int64_t end = qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + SNB_JOB_TIMEOUT_MS;
    bool ran;

    qemu_mutex_lock(&s->job_lock);
    assert(s->job_state == SNB_JOB_IDLE);
    s->job_fn = fn;
    s->job_opaque = opaque;
    s->job_state = SNB_JOB_PENDING;
    qemu_cond_broadcast(&s->job_cond);
    qemu_mutex_unlock(&s->job_lock);
    snb_channel_kick(&s->master);

    qemu_mutex_lock(&s->job_lock);
    while (s->job_state == SNB_JOB_PENDING &&
           !qatomic_read(&s->stopping)) {
        int64_t left = end - qemu_clock_get_ms(QEMU_CLOCK_REALTIME);

        if (left <= 0) {
            break;
        }
        qemu_cond_timedwait(&s->job_cond, &s->job_lock, MIN(left, 100));
    }
    ran = s->job_state == SNB_JOB_DONE;
    s->job_state = SNB_JOB_IDLE;
    qemu_mutex_unlock(&s->job_lock);
    if (!ran && !qatomic_read(&s->stopping)) {
        error_report("snb: a request from the peer was not carried out in "
                     "%d ms", SNB_JOB_TIMEOUT_MS);
    }
    return ran;
}

/* Endpoints and connect notifiers */

bool snb_server_register_endpoint(SnbServer *s, uint8_t endpoint_id,
                                  const SnbPeerOps *ops, void *opaque,
                                  Error **errp)
{
    SnbEndpoint *free_ep = NULL;
    int i;

    qemu_mutex_lock(&s->table_lock);
    for (i = 0; i < SNB_MAX_ENDPOINTS; i++) {
        SnbEndpoint *ep = &s->endpoints[i];

        if (ep->in_use && ep->id == endpoint_id) {
            qemu_mutex_unlock(&s->table_lock);
            error_setg(errp, "SNB endpoint %u is already in use",
                       endpoint_id);
            return false;
        }
        if (!ep->in_use && !free_ep) {
            free_ep = ep;
        }
    }
    if (!free_ep) {
        qemu_mutex_unlock(&s->table_lock);
        error_setg(errp, "no more than %d SNB endpoints",
                   SNB_MAX_ENDPOINTS);
        return false;
    }
    free_ep->id = endpoint_id;
    free_ep->ops = ops;
    free_ep->opaque = opaque;
    free_ep->in_use = true;
    qemu_mutex_unlock(&s->table_lock);
    return true;
}

void snb_server_unregister_endpoint(SnbServer *s, uint8_t endpoint_id)
{
    int i;

    qemu_mutex_lock(&s->table_lock);
    for (i = 0; i < SNB_MAX_ENDPOINTS; i++) {
        if (s->endpoints[i].in_use && s->endpoints[i].id == endpoint_id) {
            s->endpoints[i].in_use = false;
            break;
        }
    }
    qemu_mutex_unlock(&s->table_lock);
}

/*
 * Endpoints change, and requests for them run, only with the BQL held, so
 * the entry stays valid for as long as the caller holds it.
 */
const SnbEndpoint *snb_server_find_endpoint(SnbServer *s, uint32_t id)
{
    const SnbEndpoint *found = NULL;
    int i;

    qemu_mutex_lock(&s->table_lock);
    for (i = 0; i < SNB_MAX_ENDPOINTS; i++) {
        if (s->endpoints[i].in_use && s->endpoints[i].id == id) {
            found = &s->endpoints[i];
            break;
        }
    }
    qemu_mutex_unlock(&s->table_lock);
    return found;
}

bool snb_server_add_connect_notifier(SnbServer *s, void (*cb)(void *opaque),
                                     void *opaque, Error **errp)
{
    int i;

    qemu_mutex_lock(&s->table_lock);
    for (i = 0; i < SNB_MAX_ENDPOINTS; i++) {
        if (!s->notifiers[i].in_use) {
            break;
        }
    }
    if (i == SNB_MAX_ENDPOINTS) {
        qemu_mutex_unlock(&s->table_lock);
        error_setg(errp, "no more than %d SNB connect notifiers",
                   SNB_MAX_ENDPOINTS);
        return false;
    }
    s->notifiers[i] = (SnbConnectNotifier) {
        .in_use = true, .cb = cb, .opaque = opaque,
    };
    qemu_mutex_unlock(&s->table_lock);

    /* A device added after the transport connected */
    if (snb_server_connected(s)) {
        cb(opaque);
    }
    return true;
}

void snb_server_remove_connect_notifier(SnbServer *s,
                                        void (*cb)(void *opaque),
                                        void *opaque)
{
    int i;

    qemu_mutex_lock(&s->table_lock);
    for (i = 0; i < SNB_MAX_ENDPOINTS; i++) {
        SnbConnectNotifier *n = &s->notifiers[i];

        if (n->in_use && n->cb == cb && n->opaque == opaque) {
            n->in_use = false;
            break;
        }
    }
    qemu_mutex_unlock(&s->table_lock);
}

static void snb_server_connect_bh(void *opaque)
{
    SnbServer *s = opaque;
    SnbConnectNotifier fire[SNB_MAX_ENDPOINTS];
    int i, n = 0;

    qemu_mutex_lock(&s->table_lock);
    for (i = 0; i < SNB_MAX_ENDPOINTS; i++) {
        if (s->notifiers[i].in_use) {
            fire[n++] = s->notifiers[i];
        }
    }
    qemu_mutex_unlock(&s->table_lock);

    for (i = 0; i < n; i++) {
        fire[i].cb(fire[i].opaque);
    }
}

bool snb_server_connected(SnbServer *s)
{
    return s && qatomic_read(&s->connected);
}

/* Sessions */

typedef enum SnbSessionResult {
    SNB_SESSION_UP,
    SNB_SESSION_CANCELED,
    SNB_SESSION_REFUSED,
    SNB_SESSION_RETRY,
} SnbSessionResult;

static SnbSessionResult snb_server_hello(SnbServer *s, SnbChannel *ch)
{
    Error *local_err = NULL;
    bool fatal;

    if (snb_channel_hello(ch, &fatal, &local_err)) {
        return SNB_SESSION_UP;
    }
    if (qatomic_read(&s->stopping)) {
        error_free(local_err);
        return SNB_SESSION_CANCELED;
    }
    error_reportf_err(local_err, "snb: %s channel: ", ch->name);
    return fatal ? SNB_SESSION_REFUSED : SNB_SESSION_RETRY;
}

/*
 * Connect and handshake the slave channel, then the master channel, which
 * is the order the peer accepts them in.
 */
static SnbSessionResult snb_server_run_session(SnbServer *s)
{
    SnbSessionResult ret;
    uint32_t session;

    /*
     * A new session number first, so that a teardown request still on its
     * way from the last session is recognised as stale.
     */
    qemu_mutex_lock(&s->session_lock);
    session = ++s->session;
    s->tearing_down = false;
    qemu_mutex_unlock(&s->session_lock);
    snb_channel_stop(&s->slave);
    snb_channel_stop(&s->master);

    if (!snb_channel_connect(&s->slave, &s->stopping)) {
        return SNB_SESSION_CANCELED;
    }
    ret = snb_server_hello(s, &s->slave);
    if (ret != SNB_SESSION_UP) {
        snb_channel_stop(&s->slave);
        return ret;
    }
    snb_channel_start(&s->slave, session);

    if (!snb_channel_connect(&s->master, &s->stopping)) {
        snb_channel_stop(&s->slave);
        return SNB_SESSION_CANCELED;
    }
    ret = snb_server_hello(s, &s->master);
    if (ret != SNB_SESSION_UP) {
        snb_channel_stop(&s->master);
        snb_channel_stop(&s->slave);
        return ret;
    }
    snb_channel_start(&s->master, session);

    info_report("snb: connected to the peer");
    qatomic_set(&s->connected, true);
    qemu_bh_schedule(s->connect_bh);
    return SNB_SESSION_UP;
}

/*
 * The peer ends the whole session when either socket goes, then accepts the
 * slave channel again first. Stop the other channel too and have the session
 * thread start over. Both receive threads can get here for one session;
 * only the first request counts.
 */
void snb_server_request_reconnect(SnbServer *s, SnbChannel *ch)
{
    SnbChannel *other = ch == &s->master ? &s->slave : &s->master;
    bool stale;

    qemu_mutex_lock(&s->session_lock);
    stale = ch->session != s->session || s->tearing_down ||
            qatomic_read(&s->stopping);
    s->tearing_down = true;
    qemu_mutex_unlock(&s->session_lock);
    if (stale) {
        return;
    }

    snb_channel_stop(other);
    qatomic_set(&s->connected, false);

    qemu_mutex_lock(&s->session_lock);
    s->reconnect = true;
    qemu_cond_signal(&s->session_cond);
    qemu_mutex_unlock(&s->session_lock);
}

static void snb_server_wait(SnbServer *s, bool for_reconnect)
{
    qemu_mutex_lock(&s->session_lock);
    if (for_reconnect) {
        while (!s->reconnect && !qatomic_read(&s->stopping)) {
            qemu_cond_wait(&s->session_cond, &s->session_lock);
        }
        s->reconnect = false;
    } else if (!qatomic_read(&s->stopping)) {
        qemu_cond_timedwait(&s->session_cond, &s->session_lock,
                            SNB_RETRY_PAUSE_MS);
    }
    qemu_mutex_unlock(&s->session_lock);
}

static void *snb_server_session_thread(void *opaque)
{
    SnbServer *s = opaque;

    while (!qatomic_read(&s->stopping)) {
        switch (snb_server_run_session(s)) {
        case SNB_SESSION_CANCELED:
            return NULL;
        case SNB_SESSION_REFUSED:
            error_report("snb: giving up on the peer");
            return NULL;
        case SNB_SESSION_RETRY:
            snb_server_wait(s, false);
            break;
        case SNB_SESSION_UP:
            snb_server_wait(s, true);
            qatomic_set(&s->connected, false);
            break;
        }
    }
    return NULL;
}

SnbServer *snb_server_new(SocketAddress *master, SocketAddress *slave,
                          uint32_t stall_hold_ms)
{
    SnbServer *s = g_new0(SnbServer, 1);

    s->stall_hold_ms = stall_hold_ms;
    snb_channel_init(&s->master, s, true, master);
    snb_channel_init(&s->slave, s, false, slave);
    qemu_mutex_init(&s->table_lock);
    qemu_mutex_init(&s->request_lock);
    qemu_cond_init(&s->request_cond);
    qemu_mutex_init(&s->job_lock);
    qemu_cond_init(&s->job_cond);
    qemu_mutex_init(&s->session_lock);
    qemu_cond_init(&s->session_cond);
    s->connect_bh = qemu_bh_new(snb_server_connect_bh, s);
    qemu_thread_create(&s->job_thread, "snb-job", snb_server_job_thread, s,
                       QEMU_THREAD_JOINABLE);

    assert(bql_locked());
    QLIST_INSERT_HEAD(&snb_servers, s, link);
    return s;
}

/* Start connecting; until then bridges register endpoints */
void snb_server_start(SnbServer *s)
{
    qemu_thread_create(&s->session_thread, "snb-session",
                       snb_server_session_thread, s, QEMU_THREAD_JOINABLE);
    s->session_started = true;
}

void snb_server_free(SnbServer *s)
{
    assert(bql_locked());
    QLIST_REMOVE(s, link);
    qatomic_set(&s->stopping, true);

    if (s->session_started) {
        qemu_mutex_lock(&s->session_lock);
        qemu_cond_signal(&s->session_cond);
        qemu_mutex_unlock(&s->session_lock);
        /* A HELLO the peer never answers would block the join */
        snb_channel_shutdown(&s->slave);
        snb_channel_shutdown(&s->master);
        qemu_thread_join(&s->session_thread);
    }
    snb_channel_stop(&s->slave);
    snb_channel_stop(&s->master);

    /* The job thread may be waiting for the BQL */
    qemu_mutex_lock(&s->job_lock);
    qemu_cond_broadcast(&s->job_cond);
    qemu_mutex_unlock(&s->job_lock);
    bql_unlock();
    qemu_thread_join(&s->job_thread);
    bql_lock();

    qemu_bh_delete(s->connect_bh);
    snb_channel_cleanup(&s->master);
    snb_channel_cleanup(&s->slave);
    qemu_cond_destroy(&s->session_cond);
    qemu_mutex_destroy(&s->session_lock);
    qemu_cond_destroy(&s->job_cond);
    qemu_mutex_destroy(&s->job_lock);
    qemu_cond_destroy(&s->request_cond);
    qemu_mutex_destroy(&s->request_lock);
    qemu_mutex_destroy(&s->table_lock);
    g_free(s);
}

/* Requests to the peer */

/*
 * Take the master channel, or give up at @deadline_ms. The request ahead
 * may be waiting on the peer, and the peer on a job: a waiter holding the
 * BQL runs the jobs meanwhile.
 */
bool snb_server_request_begin(SnbServer *s, int64_t deadline_ms)
{
    const bool bql = bql_locked();

    qemu_mutex_lock(&s->request_lock);
    while (s->request_busy) {
        if (deadline_ms &&
            qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) >= deadline_ms) {
            qemu_mutex_unlock(&s->request_lock);
            return false;
        }
        qemu_cond_timedwait(&s->request_cond, &s->request_lock,
                            bql ? 1 : 10);
        if (bql) {
            qemu_mutex_unlock(&s->request_lock);
            snb_server_run_all_jobs();
            qemu_mutex_lock(&s->request_lock);
        }
    }
    s->request_busy = true;
    qemu_mutex_unlock(&s->request_lock);
    return true;
}

void snb_server_request_end(SnbServer *s)
{
    qemu_mutex_lock(&s->request_lock);
    s->request_busy = false;
    qemu_cond_signal(&s->request_cond);
    qemu_mutex_unlock(&s->request_lock);
}

/*
 * Send the request built on the master channel and wait for its response;
 * the response's status, then @cap bytes for @buf, or with @var_len a length
 * and up to that many bytes.
 */
static bool snb_server_call(SnbServer *s, uint32_t type, uint32_t data_off,
                            bool var_len, void *buf, uint32_t cap,
                            uint32_t *len, int64_t deadline_ms)
{
    SnbRpc rpc = {
        .rsp_type = SNB_RSP(type),
        .data_off = data_off,
        .var_len = var_len,
        .buf = buf,
        .cap = cap,
        .deadline_ms = deadline_ms,
    };
    bool ok = snb_channel_call(&s->master, &rpc);

    if (ok && len) {
        *len = rpc.len;
    }
    return ok;
}

static bool snb_server_status_call(SnbServer *s, uint32_t type,
                                   int64_t deadline_ms)
{
    return snb_server_call(s, type, sizeof(SnbMsgStatus), false, NULL, 0,
                           NULL, deadline_ms);
}

/* Start building a request; false if the master channel was not taken */
static bool snb_server_begin(SnbServer *s, uint32_t type, uint8_t endpoint_id,
                             uint32_t msg_len, int64_t deadline_ms)
{
    if (msg_len > SNB_MAX_DATA_SIZE) {
        error_report("snb: a %u-byte request is too large", msg_len);
        return false;
    }
    if (!snb_server_request_begin(s, deadline_ms)) {
        return false;
    }
    snb_channel_begin(&s->master, type, endpoint_id, msg_len);
    return true;
}

bool snb_server_bus_register(SnbServer *s, const char *name,
                             uint32_t bus_class, uint8_t endpoint_id)
{
    SnbMsgBusRegister *m;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_BUS_REGISTER, endpoint_id, sizeof(*m),
                          0)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    memset(m, 0, sizeof(*m));
    strncpy(m->name, name, sizeof(m->name) - 1);
    m->bus_class = cpu_to_le32(bus_class);
    m->endpoint_id = cpu_to_le32(endpoint_id);
    ok = snb_server_status_call(s, SNB_MSG_BUS_REGISTER, 0);
    snb_server_request_end(s);
    trace_snb_bus_register(name, bus_class, endpoint_id, ok);
    return ok;
}

bool snb_server_i2c_write(SnbServer *s, uint8_t endpoint_id, uint8_t addr,
                          bool stop, const uint8_t *data, uint32_t len)
{
    SnbMsgI2cWrite *m;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_I2C_WRITE, endpoint_id,
                          sizeof(*m) + len, 0)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    m->addr = addr;
    m->stop = stop;
    m->len = cpu_to_le32(len);
    memcpy(m->data, data, len);
    ok = snb_server_status_call(s, SNB_MSG_I2C_WRITE, 0);
    snb_server_request_end(s);
    return ok;
}

bool snb_server_i2c_read(SnbServer *s, uint8_t endpoint_id, uint8_t addr,
                         bool stop, uint8_t *data, uint32_t len)
{
    SnbMsgI2cRead *m;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_I2C_READ, endpoint_id, sizeof(*m), 0)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    m->addr = addr;
    m->stop = stop;
    m->len = cpu_to_le32(len);
    ok = snb_server_call(s, SNB_MSG_I2C_READ, sizeof(SnbMsgStatus), false,
                         data, len, NULL, 0);
    snb_server_request_end(s);
    if (!ok) {
        memset(data, 0, len);
    }
    return ok;
}

bool snb_server_i3c_enum_i3c(SnbServer *s, uint8_t endpoint_id,
                             SnbI3cDevice *devs, unsigned *count,
                             int64_t deadline_ms)
{
    SnbMsgI3cEnumI3cRsp rsp;
    SnbMsgReserved *m;
    unsigned i;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_I3C_ENUM_I3C, endpoint_id, sizeof(*m),
                          deadline_ms)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    m->reserved = 0;
    ok = snb_server_call(s, SNB_MSG_I3C_ENUM_I3C, 0, false, &rsp,
                         sizeof(rsp), NULL, deadline_ms);
    snb_server_request_end(s);
    if (!ok) {
        return false;
    }
    *count = MIN(rsp.count, SNB_I3C_MAX_DEVS);
    for (i = 0; i < *count; i++) {
        devs[i] = (SnbI3cDevice) {
            .static_addr = rsp.devs[i].static_addr,
            .pid = le64_to_cpu(rsp.devs[i].pid),
            .bcr = rsp.devs[i].bcr,
            .dcr = rsp.devs[i].dcr,
        };
    }
    return true;
}

bool snb_server_i3c_enum_i2c(SnbServer *s, uint8_t endpoint_id,
                             uint8_t *addrs, unsigned *count,
                             int64_t deadline_ms)
{
    SnbMsgI3cEnumI2cRsp rsp;
    SnbMsgReserved *m;
    unsigned i;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_I3C_ENUM_I2C, endpoint_id, sizeof(*m),
                          deadline_ms)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    m->reserved = 0;
    ok = snb_server_call(s, SNB_MSG_I3C_ENUM_I2C, 0, false, &rsp,
                         sizeof(rsp), NULL, deadline_ms);
    snb_server_request_end(s);
    if (!ok) {
        return false;
    }
    *count = MIN(rsp.count, SNB_I3C_MAX_DEVS);
    for (i = 0; i < *count; i++) {
        addrs[i] = rsp.devs[i].addr;
    }
    return true;
}

bool snb_server_i3c_write(SnbServer *s, uint8_t endpoint_id, uint8_t da,
                          const uint8_t *data, uint32_t len,
                          int64_t deadline_ms)
{
    SnbMsgI3cWrite *m;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_I3C_WRITE, endpoint_id,
                          sizeof(*m) + len, deadline_ms)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    m->da = da;
    m->len = cpu_to_le32(len);
    memcpy(m->data, data, len);
    ok = snb_server_status_call(s, SNB_MSG_I3C_WRITE, deadline_ms);
    snb_server_request_end(s);
    return ok;
}

bool snb_server_i3c_read(SnbServer *s, uint8_t endpoint_id, uint8_t da,
                         uint8_t *data, uint32_t len, uint32_t *actual,
                         int64_t deadline_ms)
{
    SnbMsgI3cRead *m;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_I3C_READ, endpoint_id, sizeof(*m),
                          deadline_ms)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    m->da = da;
    m->len = cpu_to_le32(len);
    ok = snb_server_call(s, SNB_MSG_I3C_READ, sizeof(SnbMsgI3cDataRsp), true,
                         data, len, actual, deadline_ms);
    snb_server_request_end(s);
    if (!ok) {
        memset(data, 0, len);
    }
    return ok;
}

bool snb_server_i3c_ccc(SnbServer *s, uint8_t endpoint_id, uint8_t ccc,
                        bool rnw, uint8_t da, uint8_t *data, uint32_t len,
                        int64_t deadline_ms)
{
    uint32_t tx_len = rnw ? 0 : len;
    SnbMsgI3cCcc *m;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_I3C_CCC, endpoint_id,
                          sizeof(*m) + tx_len, deadline_ms)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    m->ccc = ccc;
    m->rnw = rnw;
    m->da = da;
    m->len = cpu_to_le32(len);
    if (tx_len) {
        memcpy(m->data, data, tx_len);
    }
    ok = snb_server_call(s, SNB_MSG_I3C_CCC, sizeof(SnbMsgI3cDataRsp), true,
                         rnw ? data : NULL, rnw ? len : 0, NULL,
                         deadline_ms);
    snb_server_request_end(s);
    return ok;
}

bool snb_server_i3c_da_assigned(SnbServer *s, uint8_t endpoint_id,
                                uint64_t pid, uint8_t da,
                                int64_t deadline_ms)
{
    SnbMsgI3cDaAssigned *m;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_I3C_DA_ASSIGNED, endpoint_id,
                          sizeof(*m), deadline_ms)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    m->pid = cpu_to_le64(pid);
    m->da = da;
    ok = snb_server_status_call(s, SNB_MSG_I3C_DA_ASSIGNED, deadline_ms);
    snb_server_request_end(s);
    return ok;
}

bool snb_server_i3c_i2c_write(SnbServer *s, uint8_t endpoint_id,
                              uint8_t addr, const uint8_t *data,
                              uint32_t len, int64_t deadline_ms)
{
    SnbMsgI3cI2cWrite *m;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_I3C_I2C_WRITE, endpoint_id,
                          sizeof(*m) + len, deadline_ms)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    m->addr = addr;
    m->len = cpu_to_le32(len);
    memcpy(m->data, data, len);
    ok = snb_server_status_call(s, SNB_MSG_I3C_I2C_WRITE, deadline_ms);
    snb_server_request_end(s);
    return ok;
}

bool snb_server_i3c_i2c_read(SnbServer *s, uint8_t endpoint_id, uint8_t addr,
                             uint8_t *data, uint32_t len,
                             int64_t deadline_ms)
{
    SnbMsgI3cI2cRead *m;
    bool ok;

    if (!snb_server_begin(s, SNB_MSG_I3C_I2C_READ, endpoint_id, sizeof(*m),
                          deadline_ms)) {
        return false;
    }
    m = snb_channel_msg(&s->master);
    m->addr = addr;
    m->len = cpu_to_le32(len);
    ok = snb_server_call(s, SNB_MSG_I3C_I2C_READ, sizeof(SnbMsgStatus), false,
                         data, len, NULL, deadline_ms);
    snb_server_request_end(s);
    if (!ok) {
        memset(data, 0, len);
    }
    return ok;
}

/* Notifications to the peer */

bool snb_server_gpio_notify(SnbServer *s, uint8_t endpoint_id,
                            const char *name, uint8_t level)
{
    SnbMsgGpioSet m = { .level = level };

    strncpy(m.name, name, sizeof(m.name) - 1);
    return snb_channel_notify(&s->master, SNB_MSG_GPIO_SET, endpoint_id,
                              &m, sizeof(m), NULL, 0);
}

bool snb_server_espi_vw_notify(SnbServer *s, uint8_t endpoint_id,
                               uint8_t index, uint8_t data)
{
    SnbMsgEspiVw m = { .index = index, .value = cpu_to_le32(data) };

    return snb_channel_notify(&s->master, SNB_MSG_ESPI_VW_NOTIFY,
                              endpoint_id, &m, sizeof(m), NULL, 0);
}

bool snb_server_espi_alert(SnbServer *s, uint8_t endpoint_id,
                           uint32_t espi_status)
{
    SnbMsgEspiAlert m = { .espi_status = cpu_to_le32(espi_status) };

    return snb_channel_notify(&s->master, SNB_MSG_ESPI_ALERT, endpoint_id,
                              &m, sizeof(m), NULL, 0);
}

bool snb_server_espi_pc_notify(SnbServer *s, uint8_t endpoint_id,
                               uint8_t cyc, uint8_t tag, uint64_t addr,
                               uint32_t len, const uint8_t *data,
                               uint8_t msg_code)
{
    uint32_t data_len = snb_espi_pc_has_data(cyc) ? len : 0;
    SnbMsgEspiPcCycle m = {
        .cyc = cyc,
        .tag = tag,
        .addr = cpu_to_le64(addr),
        .len = cpu_to_le32(snb_espi_pc_has_len(cyc) ? len : 0),
        .msg_code = msg_code,
    };

    if (data_len > SNB_ESPI_MAX_PLD) {
        error_report("snb: a %u-byte peripheral cycle is too large", len);
        return false;
    }
    return snb_channel_notify(&s->master, SNB_MSG_ESPI_PC_NOTIFY, endpoint_id,
                              &m, sizeof(m), data, data_len);
}
