/*
 * Simulator network bridge (SNB): I2C bus bridge
 *
 * Sits on a QEMU I2C bus and carries the transfers to the targets the peer
 * holds. It answers every unicast address, or only those listed in
 * "addresses" (colon separated, as in addresses=0x3c:0x4c). A write is sent
 * when the transfer ends; a read fetches "read-size" bytes when it starts,
 * after sending any bytes written before a repeated start, and the
 * controller takes as many of them as it wants.
 *
 * The peer drives the "alert" GPIO output with the signal it names ALERT_L,
 * 1 while asserted.
 *
 *   -device snb-i2c-bus-bridge,bus=aspeed.i2c.bus.1,transport=snb0,
 *           addresses=0x3c:0x4c
 *
 * Copyright (C) 2025-2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitmap.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "hw/snb/snb.h"
#include "trace.h"

#define TYPE_SNB_I2C_BRIDGE "snb-i2c-bus-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(SnbI2cBridgeState, SNB_I2C_BRIDGE)

#define SNB_I2C_BUF_SIZE    4096
#define SNB_I2C_NR_ADDRS    128

typedef enum SnbI2cDir {
    SNB_I2C_IDLE,
    SNB_I2C_WRITING,
    SNB_I2C_READING,
} SnbI2cDir;

struct SnbI2cBridgeState {
    I2CSlave parent_obj;

    SnbTransportState *transport;
    SnbServer *server;
    uint8_t endpoint_id;
    char *bus_name;
    uint32_t read_size;
    char *addresses;

    /* Addresses to answer; empty answers all of them */
    DECLARE_BITMAP(addr_map, SNB_I2C_NR_ADDRS);
    qemu_irq alert;

    uint8_t addr;
    SnbI2cDir dir;
    uint8_t wbuf[SNB_I2C_BUF_SIZE];
    uint32_t wlen;
    uint8_t rbuf[SNB_I2C_BUF_SIZE];
    uint32_t rlen;
    uint32_t rpos;
};

static bool snb_i2c_bridge_match(I2CSlave *candidate, uint8_t address,
                                 bool broadcast, I2CNodeList *current_devs)
{
    SnbI2cBridgeState *s = SNB_I2C_BRIDGE(candidate);
    I2CNode *node;

    if (broadcast) {
        return false;
    }
    if (!bitmap_empty(s->addr_map, SNB_I2C_NR_ADDRS) &&
        !test_bit(address, s->addr_map)) {
        return false;
    }

    s->addr = address;
    node = g_new(I2CNode, 1);
    node->elt = candidate;
    QLIST_INSERT_HEAD(current_devs, node, next);
    return true;
}

/* Send what was written; @stop is false before a repeated start */
static void snb_i2c_bridge_flush(SnbI2cBridgeState *s, bool stop)
{
    bool ok;

    if (!s->wlen) {
        return;
    }
    ok = snb_server_i2c_write(s->server, s->endpoint_id, s->addr, stop,
                              s->wbuf, s->wlen);
    trace_snb_i2c_write(s->addr, s->wlen, stop, ok);
    if (!ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of %u bytes to 0x%02x "
                      "failed\n", TYPE_SNB_I2C_BRIDGE, s->wlen, s->addr);
    }
    s->wlen = 0;
}

static int snb_i2c_bridge_event(I2CSlave *i2c, enum i2c_event event)
{
    SnbI2cBridgeState *s = SNB_I2C_BRIDGE(i2c);
    bool ok;

    switch (event) {
    case I2C_START_SEND_ASYNC:
    case I2C_START_SEND:
        s->dir = SNB_I2C_WRITING;
        s->wlen = 0;
        break;
    case I2C_START_RECV:
        if (s->dir == SNB_I2C_WRITING) {
            snb_i2c_bridge_flush(s, false);
        }
        s->dir = SNB_I2C_READING;
        s->rlen = s->rpos = 0;
        ok = snb_server_i2c_read(s->server, s->endpoint_id, s->addr, true,
                                 s->rbuf, s->read_size);
        trace_snb_i2c_read(s->addr, s->read_size, ok);
        if (ok) {
            s->rlen = s->read_size;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: read from 0x%02x failed\n",
                          TYPE_SNB_I2C_BRIDGE, s->addr);
        }
        break;
    case I2C_FINISH:
        if (s->dir == SNB_I2C_WRITING) {
            snb_i2c_bridge_flush(s, true);
        }
        s->dir = SNB_I2C_IDLE;
        break;
    case I2C_NACK:
        break;
    default:
        return -1;
    }
    return 0;
}

static int snb_i2c_bridge_send(I2CSlave *i2c, uint8_t data)
{
    SnbI2cBridgeState *s = SNB_I2C_BRIDGE(i2c);

    if (s->wlen == SNB_I2C_BUF_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to 0x%02x longer than %d "
                      "bytes\n", TYPE_SNB_I2C_BRIDGE, s->addr,
                      SNB_I2C_BUF_SIZE);
        return -1;
    }
    s->wbuf[s->wlen++] = data;
    return 0;
}

/* A read the peer did not answer, or one past read-size, gets 0xff */
static uint8_t snb_i2c_bridge_recv(I2CSlave *i2c)
{
    SnbI2cBridgeState *s = SNB_I2C_BRIDGE(i2c);

    return s->rpos < s->rlen ? s->rbuf[s->rpos++] : 0xff;
}

static bool snb_i2c_bridge_gpio_set(void *opaque, const char *name,
                                    uint8_t level)
{
    SnbI2cBridgeState *s = opaque;

    trace_snb_i2c_gpio_set(name, level);
    if (strcmp(name, "ALERT_L")) {
        qemu_log_mask(LOG_UNIMP, "%s: no signal named '%s'\n",
                      TYPE_SNB_I2C_BRIDGE, name);
        return false;
    }
    qemu_set_irq(s->alert, !!level);
    return true;
}

static const SnbPeerOps snb_i2c_bridge_ops = {
    .gpio_set = snb_i2c_bridge_gpio_set,
};

/* The peer forgets the bus binding when a session ends */
static void snb_i2c_bridge_connected(void *opaque)
{
    SnbI2cBridgeState *s = opaque;

    if (s->bus_name && s->bus_name[0] &&
        !snb_server_bus_register(s->server, s->bus_name, SNB_BUS_I2C,
                                 s->endpoint_id)) {
        warn_report("%s: the peer did not take bus '%s'",
                    TYPE_SNB_I2C_BRIDGE, s->bus_name);
    }
}

static bool snb_i2c_bridge_parse_addresses(SnbI2cBridgeState *s,
                                           Error **errp)
{
    g_auto(GStrv) list = NULL;
    int i;

    if (!s->addresses) {
        return true;
    }
    list = g_strsplit(s->addresses, ":", -1);
    for (i = 0; list[i]; i++) {
        uint64_t addr;

        if (qemu_strtou64(list[i], NULL, 0, &addr) || addr < 1 ||
            addr >= SNB_I2C_NR_ADDRS) {
            error_setg(errp, "'%s' is not a 7-bit I2C address", list[i]);
            return false;
        }
        set_bit(addr, s->addr_map);
    }
    return true;
}

static void snb_i2c_bridge_realize(DeviceState *dev, Error **errp)
{
    SnbI2cBridgeState *s = SNB_I2C_BRIDGE(dev);

    if (!s->transport) {
        error_setg(errp, "'transport' is not set");
        return;
    }
    if (!s->read_size || s->read_size > SNB_I2C_BUF_SIZE) {
        error_setg(errp, "'read-size' must be 1 to %d", SNB_I2C_BUF_SIZE);
        return;
    }
    if (!snb_i2c_bridge_parse_addresses(s, errp)) {
        return;
    }

    s->server = snb_transport_get_server(s->transport);
    if (!snb_server_register_endpoint(s->server, s->endpoint_id,
                                      &snb_i2c_bridge_ops, s, errp)) {
        return;
    }
    if (!snb_server_add_connect_notifier(s->server, snb_i2c_bridge_connected,
                                         s, errp)) {
        snb_server_unregister_endpoint(s->server, s->endpoint_id);
        return;
    }
}

static void snb_i2c_bridge_unrealize(DeviceState *dev)
{
    SnbI2cBridgeState *s = SNB_I2C_BRIDGE(dev);

    snb_server_remove_connect_notifier(s->server, snb_i2c_bridge_connected,
                                       s);
    snb_server_unregister_endpoint(s->server, s->endpoint_id);
}

static void snb_i2c_bridge_init(Object *obj)
{
    SnbI2cBridgeState *s = SNB_I2C_BRIDGE(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->alert, "alert", 1);
}

static const Property snb_i2c_bridge_props[] = {
    DEFINE_PROP_LINK("transport", SnbI2cBridgeState, transport,
                     TYPE_SNB_TRANSPORT, SnbTransportState *),
    DEFINE_PROP_UINT8("endpoint-id", SnbI2cBridgeState, endpoint_id,
                      SNB_ENDPOINT_I2C),
    DEFINE_PROP_STRING("bus-name", SnbI2cBridgeState, bus_name),
    DEFINE_PROP_UINT32("read-size", SnbI2cBridgeState, read_size, 256),
    DEFINE_PROP_STRING("addresses", SnbI2cBridgeState, addresses),
};

static void snb_i2c_bridge_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(oc);

    dc->desc = "Simulator network bridge I2C bus bridge";
    dc->realize = snb_i2c_bridge_realize;
    dc->unrealize = snb_i2c_bridge_unrealize;
    device_class_set_props(dc, snb_i2c_bridge_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    sc->match_and_add = snb_i2c_bridge_match;
    sc->event = snb_i2c_bridge_event;
    sc->send = snb_i2c_bridge_send;
    sc->recv = snb_i2c_bridge_recv;
}

static const TypeInfo snb_i2c_bridge_types[] = {
    {
        .name = TYPE_SNB_I2C_BRIDGE,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(SnbI2cBridgeState),
        .instance_init = snb_i2c_bridge_init,
        .class_init = snb_i2c_bridge_class_init,
    },
};

DEFINE_TYPES(snb_i2c_bridge_types)
