/*
 * Simulator network bridge (SNB): I3C targets the peer holds
 *
 * The I3C core assigns and resets dynamic addresses and answers the
 * identity CCCs from the target's PID, BCR and DCR; the rest goes to the
 * peer, addressed by the target's current dynamic address. Each time the
 * core changes that address the peer is told, so that it can map the
 * addresses it sees to its targets.
 *
 * A transfer to the peer blocks; when the bus allows it the BQL is
 * released meanwhile, and the bus's transfer deadline bounds the wait.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "snb-i3c.h"
#include "trace.h"

static I3CBus *snb_i3c_target_bus(I3CTarget *t)
{
    return I3C_BUS(qdev_get_parent_bus(DEVICE(t)));
}

void snb_i3c_target_attach(SnbI3cTarget *st, SnbServer *server,
                           uint8_t endpoint_id,
                           void (*entdaa_hook)(void *opaque, I3CBus *bus),
                           void *opaque)
{
    st->server = server;
    st->endpoint_id = endpoint_id;
    st->entdaa_hook = entdaa_hook;
    st->entdaa_opaque = opaque;
    /* The peer knows the target as unaddressed from the enumeration */
    st->reported_da = I3C_TARGET(st)->address;
}

static bool snb_i3c_target_da_assigned(SnbI3cTarget *st, uint8_t da,
                                       int64_t deadline_ms)
{
    I3CTarget *t = I3C_TARGET(st);
    I3CBus *bus = snb_i3c_target_bus(t);
    bool unlocked = i3c_bus_release_bql(bus);
    bool ok = snb_server_i3c_da_assigned(st->server, st->endpoint_id, t->pid,
                                         da, deadline_ms);

    if (unlocked) {
        bql_lock();
    }
    trace_snb_i3c_da_assigned(t->pid, da, ok);
    return ok;
}

/* Tell the peer if the core changed the target's dynamic address */
static void snb_i3c_target_report_da(SnbI3cTarget *st)
{
    I3CTarget *t = I3C_TARGET(st);
    I3CBus *bus = snb_i3c_target_bus(t);

    if (t->address == st->reported_da) {
        return;
    }
    if (snb_i3c_target_da_assigned(st, t->address, bus->xfer_deadline_ms)) {
        st->reported_da = t->address;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: the peer did not take address "
                      "0x%02x for PID 0x%012" PRIx64 "\n",
                      TYPE_SNB_I3C_TARGET, t->address, t->pid);
    }
}

bool snb_i3c_target_resync(SnbI3cTarget *st, int64_t deadline_ms)
{
    I3CTarget *t = I3C_TARGET(st);

    st->reported_da = 0;
    if (!t->address) {
        return true;
    }
    if (!snb_i3c_target_da_assigned(st, t->address, deadline_ms)) {
        return false;
    }
    st->reported_da = t->address;
    return true;
}

static int snb_i3c_target_send(I3CTarget *t, const uint8_t *data,
                               uint32_t num_to_send, uint32_t *num_sent)
{
    SnbI3cTarget *st = SNB_I3C_TARGET(t);
    I3CBus *bus = snb_i3c_target_bus(t);
    bool unlocked = i3c_bus_release_bql(bus);
    bool ok = snb_server_i3c_write(st->server, st->endpoint_id, t->address,
                                   data, num_to_send, bus->xfer_deadline_ms);

    if (unlocked) {
        bql_lock();
    }
    trace_snb_i3c_write(t->address, num_to_send, ok);
    if (!ok) {
        return -1;
    }
    *num_sent = num_to_send;
    return 0;
}

/* A read can end before @num_to_read; return what the target produced */
static uint32_t snb_i3c_target_recv(I3CTarget *t, uint8_t *data,
                                    uint32_t num_to_read)
{
    SnbI3cTarget *st = SNB_I3C_TARGET(t);
    I3CBus *bus = snb_i3c_target_bus(t);
    uint32_t actual = 0;
    bool unlocked = i3c_bus_release_bql(bus);
    bool ok = snb_server_i3c_read(st->server, st->endpoint_id, t->address,
                                  data, num_to_read, &actual,
                                  bus->xfer_deadline_ms);

    if (unlocked) {
        bql_lock();
    }
    trace_snb_i3c_read(t->address, num_to_read, actual, ok);
    return ok ? MIN(actual, num_to_read) : 0;
}

static int snb_i3c_target_event(I3CTarget *t, enum I3CEvent event)
{
    /* Address assignment and reset take effect by the end of a transfer */
    if (event == I3C_STOP) {
        snb_i3c_target_report_da(SNB_I3C_TARGET(t));
    }
    return 0;
}

/*
 * The CCCs the core carries out from the target's own state, and the bus
 * capability queries this target answers itself. None goes to the peer.
 */
static bool snb_i3c_target_ccc_local(uint8_t ccc)
{
    switch (ccc) {
    case I3C_CCC_ENTDAA:
    case I3C_CCC_RSTDAA:
    case I3C_CCC_SETAASA:
    case I3C_CCCD_SETDASA:
    case I3C_CCC_ENTTM:
    case I3C_CCCD_GETPID:
    case I3C_CCCD_GETBCR:
    case I3C_CCCD_GETDCR:
    case I3C_CCCD_GETMWL:
    case I3C_CCCD_GETMRL:
    case I3C_CCCD_GETMXDS:
    case I3C_CCCD_GETCAPS:
        return true;
    default:
        return false;
    }
}

static bool snb_i3c_target_ccc(I3CTarget *t, bool rnw, uint8_t da,
                               uint8_t *data, uint32_t len)
{
    SnbI3cTarget *st = SNB_I3C_TARGET(t);
    I3CBus *bus = snb_i3c_target_bus(t);
    bool unlocked = i3c_bus_release_bql(bus);
    bool ok = snb_server_i3c_ccc(st->server, st->endpoint_id, t->curr_ccc,
                                 rnw, da, data, len, bus->xfer_deadline_ms);

    if (unlocked) {
        bql_lock();
    }
    trace_snb_i3c_ccc(t->curr_ccc, rnw, da, len, ok);
    return ok;
}

static int snb_i3c_target_handle_ccc_write(I3CTarget *t, const uint8_t *data,
                                           uint32_t num_to_send,
                                           uint32_t *num_sent)
{
    SnbI3cTarget *st = SNB_I3C_TARGET(t);
    uint8_t da;

    /* Both are big-endian; SETMRL may add the IBI payload length */
    if (t->curr_ccc == I3C_CCCD_SETMWL && num_to_send >= 2) {
        st->mwl = lduw_be_p(data);
    } else if (t->curr_ccc == I3C_CCCD_SETMRL && num_to_send >= 2) {
        st->mrl = lduw_be_p(data);
        if (num_to_send >= 3) {
            st->max_ibi_len = data[2];
        }
    }

    if (st->entdaa_hook && t->in_ccc && t->curr_ccc == I3C_CCC_ENTDAA &&
        t->ccc_byte_offset == 0) {
        st->entdaa_hook(st->entdaa_opaque, snb_i3c_target_bus(t));
    }

    *num_sent = num_to_send;
    if (!t->in_ccc || !num_to_send ||
        snb_i3c_target_ccc_local(t->curr_ccc)) {
        return 0;
    }
    /*
     * The core offers the broadcast part of a direct CCC to every target;
     * one without an address has nothing to pass on until it is addressed.
     */
    if (CCC_IS_DIRECT(t->curr_ccc) && !t->address) {
        return 0;
    }

    if (t->curr_ccc == I3C_CCCD_SETNEWDA) {
        /*
         * The core has already moved the target to its new address. The
         * peer finds the target by the address it last heard of, and takes
         * the new one from the payload. The broadcast part carries only the
         * CCC itself.
         */
        if (num_to_send == 1 && data[0] == I3C_CCCD_SETNEWDA) {
            return 0;
        }
        da = st->reported_da;
    } else if (CCC_IS_DIRECT(t->curr_ccc)) {
        da = t->address;
    } else {
        da = I3C_BROADCAST;
    }
    return snb_i3c_target_ccc(t, false, da, (uint8_t *)data, num_to_send) ?
           0 : -1;
}

static int snb_i3c_target_handle_ccc_read(I3CTarget *t, uint8_t *data,
                                          uint32_t num_to_read,
                                          uint32_t *num_read)
{
    SnbI3cTarget *st = SNB_I3C_TARGET(t);

    if (*num_read) {
        return 0;
    }

    switch (t->curr_ccc) {
    case I3C_CCCD_GETMWL:
        memset(data, 0, num_to_read);
        if (num_to_read >= 2) {
            stw_be_p(data, st->mwl);
        }
        *num_read = num_to_read;
        return 0;
    case I3C_CCCD_GETMRL:
        memset(data, 0, num_to_read);
        if (num_to_read >= 2) {
            stw_be_p(data, st->mrl);
        }
        if (st->mrl && num_to_read >= 3) {
            data[2] = st->max_ibi_len;
        }
        *num_read = num_to_read;
        return 0;
    case I3C_CCCD_GETMXDS:
    case I3C_CCCD_GETCAPS:
        /* No limits and no optional capabilities */
        memset(data, 0, num_to_read);
        *num_read = num_to_read;
        return 0;
    default:
        break;
    }

    if (snb_i3c_target_ccc_local(t->curr_ccc)) {
        return 0;
    }
    if (!snb_i3c_target_ccc(t, true, t->address, data, num_to_read)) {
        return -1;
    }
    *num_read = num_to_read;
    return 0;
}

static void snb_i3c_target_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I3CTargetClass *tc = I3C_TARGET_CLASS(oc);

    dc->desc = "Simulator network bridge I3C target";
    /* Created by snb-i3c-bus-bridge from the peer's enumeration */
    dc->user_creatable = false;
    tc->send = snb_i3c_target_send;
    tc->recv = snb_i3c_target_recv;
    tc->event = snb_i3c_target_event;
    tc->handle_ccc_write = snb_i3c_target_handle_ccc_write;
    tc->handle_ccc_read = snb_i3c_target_handle_ccc_read;
}

/* Legacy I2C targets on the I3C bus */

void snb_i3c_i2c_target_attach(SnbI3cI2cTarget *st, SnbServer *server,
                               uint8_t endpoint_id, I3CBus *bus)
{
    st->server = server;
    st->endpoint_id = endpoint_id;
    st->bus = bus;
}

static int snb_i3c_i2c_target_event(I2CSlave *i2c, enum i2c_event event)
{
    SnbI3cI2cTarget *st = SNB_I3C_I2C_TARGET(i2c);
    bool unlocked, ok;

    switch (event) {
    case I2C_START_SEND:
    case I2C_START_SEND_ASYNC:
        st->len = 0;
        break;
    case I2C_FINISH:
        if (!st->len) {
            break;
        }
        unlocked = i3c_bus_release_bql(st->bus);
        ok = snb_server_i3c_i2c_write(st->server, st->endpoint_id,
                                      i2c->address, st->buf, st->len,
                                      st->bus->xfer_deadline_ms);
        if (unlocked) {
            bql_lock();
        }
        trace_snb_i3c_i2c_write(i2c->address, st->len, ok);
        if (!ok) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: write of %u bytes to 0x%02x "
                          "failed\n", TYPE_SNB_I3C_I2C_TARGET, st->len,
                          i2c->address);
        }
        st->len = 0;
        break;
    default:
        break;
    }
    return 0;
}

static int snb_i3c_i2c_target_send(I2CSlave *i2c, uint8_t data)
{
    SnbI3cI2cTarget *st = SNB_I3C_I2C_TARGET(i2c);

    if (st->len == SNB_I3C_I2C_BUF_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to 0x%02x longer than %d "
                      "bytes\n", TYPE_SNB_I3C_I2C_TARGET, i2c->address,
                      SNB_I3C_I2C_BUF_SIZE);
        return -1;
    }
    st->buf[st->len++] = data;
    return 0;
}

/* One byte at a time, since the controller may stop after any of them */
static uint8_t snb_i3c_i2c_target_recv(I2CSlave *i2c)
{
    SnbI3cI2cTarget *st = SNB_I3C_I2C_TARGET(i2c);
    uint8_t byte;
    bool unlocked = i3c_bus_release_bql(st->bus);
    bool ok = snb_server_i3c_i2c_read(st->server, st->endpoint_id,
                                      i2c->address, &byte, 1,
                                      st->bus->xfer_deadline_ms);

    if (unlocked) {
        bql_lock();
    }
    trace_snb_i3c_i2c_read(i2c->address, ok);
    return ok ? byte : 0xff;
}

static void snb_i3c_i2c_target_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->desc = "Simulator network bridge I2C target on an I3C bus";
    dc->user_creatable = false;
    sc->event = snb_i3c_i2c_target_event;
    sc->send = snb_i3c_i2c_target_send;
    sc->recv = snb_i3c_i2c_target_recv;
}

static const TypeInfo snb_i3c_target_types[] = {
    {
        .name = TYPE_SNB_I3C_TARGET,
        .parent = TYPE_I3C_TARGET,
        .instance_size = sizeof(SnbI3cTarget),
        .class_init = snb_i3c_target_class_init,
    },
    {
        .name = TYPE_SNB_I3C_I2C_TARGET,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(SnbI3cI2cTarget),
        .class_init = snb_i3c_i2c_target_class_init,
    },
};

DEFINE_TYPES(snb_i3c_target_types)
