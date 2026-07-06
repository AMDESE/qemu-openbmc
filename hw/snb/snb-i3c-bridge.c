/*
 * Simulator network bridge (SNB): I3C bus bridge
 *
 * Puts the I3C and legacy I2C targets the peer holds on the bus of an I3C
 * controller. The first time the transport connects, the bridge asks the
 * peer for its targets and creates one device for each; the peer's
 * in-band interrupts are raised through the controller.
 *
 *   -device snb-i3c-bus-bridge,ctrl=/machine/soc/i3c-hci/ctrl[4],
 *           transport=snb0
 *
 * Copyright (C) 2025-2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "hw/core/qdev-properties.h"
#include "hw/i3c/i3c-controller.h"
#include "snb-i3c.h"
#include "trace.h"

#define TYPE_SNB_I3C_BRIDGE "snb-i3c-bus-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(SnbI3cBridgeState, SNB_I3C_BRIDGE)

/*
 * A replay of the bus to a new session waits this long for the peer at a
 * time, with the BQL held, and is retried this often until it is taken.
 */
#define SNB_I3C_RESYNC_DEADLINE_MS  1000
#define SNB_I3C_RESYNC_RETRY_MS     1000

struct SnbI3cBridgeState {
    DeviceState parent_obj;

    Object *ctrl;
    SnbTransportState *transport;
    SnbServer *server;
    uint8_t endpoint_id;
    char *bus_name;

    bool enumerated;
    SnbI3cTarget *targets[SNB_I3C_MAX_DEVS];
    unsigned n_targets;
    QEMUTimer *resync_timer;
};

static bool snb_i3c_bridge_ibi(void *opaque, uint8_t da, uint8_t mdb,
                               bool has_mdb)
{
    SnbI3cBridgeState *s = opaque;

    trace_snb_i3c_ibi(da, mdb, has_mdb);
    return i3c_controller_inject_ibi(s->ctrl, da, mdb, has_mdb);
}

static const SnbPeerOps snb_i3c_bridge_ops = {
    .i3c_ibi = snb_i3c_bridge_ibi,
};

/*
 * A target's PID can change after enumeration, for example once it learns
 * where it sits. Ask again as each target is about to present its PID in
 * ENTDAA, and update the targets still without an address.
 */
static void snb_i3c_bridge_entdaa(void *opaque, I3CBus *bus)
{
    SnbI3cBridgeState *s = opaque;
    SnbI3cDevice devs[SNB_I3C_MAX_DEVS];
    int64_t deadline_ms = bus->xfer_deadline_ms;
    unsigned count, i;
    bool unlocked, ok;

    if (!snb_server_connected(s->server)) {
        return;
    }
    if (!deadline_ms) {
        deadline_ms = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                      SNB_I3C_RESYNC_DEADLINE_MS;
    }

    unlocked = i3c_bus_release_bql(bus);
    ok = snb_server_i3c_enum_i3c(s->server, s->endpoint_id, devs, &count,
                                 deadline_ms);
    if (unlocked) {
        bql_lock();
    }
    if (!ok || count != s->n_targets) {
        return;
    }

    for (i = 0; i < count; i++) {
        I3CTarget *t = I3C_TARGET(s->targets[i]);

        if (t->address || t->static_address != devs[i].static_addr ||
            t->bcr != devs[i].bcr || t->dcr != devs[i].dcr ||
            t->pid == devs[i].pid) {
            continue;
        }
        trace_snb_i3c_pid_update(devs[i].static_addr, t->pid, devs[i].pid);
        t->pid = devs[i].pid;
    }
}

static void snb_i3c_bridge_enumerate(SnbI3cBridgeState *s, Error **errp)
{
    I3CBus *bus = i3c_controller_get_bus(s->ctrl);
    SnbI3cDevice devs[SNB_I3C_MAX_DEVS];
    uint8_t addrs[SNB_I3C_MAX_DEVS];
    unsigned count, i;

    if (snb_server_i3c_enum_i3c(s->server, s->endpoint_id, devs, &count, 0)) {
        trace_snb_i3c_enum_i3c(count);
        for (i = 0; i < count; i++) {
            I3CTarget *t = i3c_target_new(TYPE_SNB_I3C_TARGET,
                                          devs[i].static_addr, devs[i].dcr,
                                          devs[i].bcr, devs[i].pid);

            snb_i3c_target_attach(SNB_I3C_TARGET(t), s->server,
                                  s->endpoint_id, snb_i3c_bridge_entdaa, s);
            if (!i3c_target_realize_and_unref(t, bus, errp)) {
                return;
            }
            s->targets[s->n_targets++] = SNB_I3C_TARGET(t);
        }
    }

    if (snb_server_i3c_enum_i2c(s->server, s->endpoint_id, addrs, &count,
                                0)) {
        trace_snb_i3c_enum_i2c(count);
        for (i = 0; i < count; i++) {
            I2CSlave *i2c = legacy_i2c_device_create_simple(
                bus, TYPE_SNB_I3C_I2C_TARGET, addrs[i]);

            snb_i3c_i2c_target_attach(SNB_I3C_I2C_TARGET(i2c), s->server,
                                      s->endpoint_id, bus);
        }
    }
    s->enumerated = true;
}

/*
 * A new session of the peer knows nothing of the bus: it learns each PID
 * from the enumeration and only then takes the dynamic addresses. Replay
 * both, so that the addresses the guest assigned still reach the targets.
 */
static bool snb_i3c_bridge_resync(SnbI3cBridgeState *s)
{
    int64_t deadline_ms = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                          SNB_I3C_RESYNC_DEADLINE_MS;
    SnbI3cDevice devs[SNB_I3C_MAX_DEVS];
    unsigned count, i;
    bool ok = true;

    if (!snb_server_i3c_enum_i3c(s->server, s->endpoint_id, devs, &count,
                                 deadline_ms)) {
        return false;
    }
    if (count != s->n_targets) {
        warn_report("%s: the peer now has %u I3C targets on endpoint %u, "
                    "not %u; restart QEMU to use them", TYPE_SNB_I3C_BRIDGE,
                    count, s->endpoint_id, s->n_targets);
    }
    for (i = 0; i < s->n_targets; i++) {
        ok &= snb_i3c_target_resync(s->targets[i], deadline_ms);
    }
    return ok;
}

/* Until the peer's simulation runs, it cannot take the addresses */
static void snb_i3c_bridge_resync_timer(void *opaque)
{
    SnbI3cBridgeState *s = opaque;

    if (snb_server_connected(s->server) && !snb_i3c_bridge_resync(s)) {
        timer_mod(s->resync_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                                   SNB_I3C_RESYNC_RETRY_MS);
    }
}

static void snb_i3c_bridge_connected(void *opaque)
{
    SnbI3cBridgeState *s = opaque;
    Error *local_err = NULL;

    /* The peer forgets the bus binding when a session ends */
    if (s->bus_name && s->bus_name[0] &&
        !snb_server_bus_register(s->server, s->bus_name, SNB_BUS_I3C,
                                 s->endpoint_id)) {
        warn_report("%s: the peer did not take bus '%s'",
                    TYPE_SNB_I3C_BRIDGE, s->bus_name);
    }

    if (s->enumerated) {
        snb_i3c_bridge_resync_timer(s);
        return;
    }
    snb_i3c_bridge_enumerate(s, &local_err);
    if (local_err) {
        error_reportf_err(local_err, "%s: ", TYPE_SNB_I3C_BRIDGE);
    }
}

typedef struct SnbI3cBridgeFind {
    SnbI3cBridgeState *self;
    SnbI3cBridgeState *other;
} SnbI3cBridgeFind;

static int snb_i3c_bridge_find_other(Object *obj, void *opaque)
{
    SnbI3cBridgeFind *f = opaque;
    SnbI3cBridgeState *b = (SnbI3cBridgeState *)
        object_dynamic_cast(obj, TYPE_SNB_I3C_BRIDGE);

    if (b && b != f->self && DEVICE(b)->realized && b->ctrl == f->self->ctrl) {
        f->other = b;
        return 1;
    }
    return 0;
}

static void snb_i3c_bridge_realize(DeviceState *dev, Error **errp)
{
    SnbI3cBridgeState *s = SNB_I3C_BRIDGE(dev);
    SnbI3cBridgeFind find = { .self = s };

    if (!s->ctrl) {
        error_setg(errp, "'ctrl' is not set");
        return;
    }
    if (!s->transport) {
        error_setg(errp, "'transport' is not set");
        return;
    }
    object_child_foreach_recursive(object_get_root(),
                                   snb_i3c_bridge_find_other, &find);
    if (find.other) {
        error_setg(errp, "'%s' already serves this controller",
                   object_get_canonical_path(OBJECT(find.other)));
        return;
    }

    s->server = snb_transport_get_server(s->transport);
    if (!snb_server_register_endpoint(s->server, s->endpoint_id,
                                      &snb_i3c_bridge_ops, s, errp)) {
        return;
    }
    s->resync_timer = timer_new_ms(QEMU_CLOCK_REALTIME,
                                   snb_i3c_bridge_resync_timer, s);
    if (!snb_server_add_connect_notifier(s->server, snb_i3c_bridge_connected,
                                         s, errp)) {
        timer_free(s->resync_timer);
        snb_server_unregister_endpoint(s->server, s->endpoint_id);
        return;
    }
}

static void snb_i3c_bridge_unrealize(DeviceState *dev)
{
    SnbI3cBridgeState *s = SNB_I3C_BRIDGE(dev);

    snb_server_remove_connect_notifier(s->server, snb_i3c_bridge_connected,
                                       s);
    snb_server_unregister_endpoint(s->server, s->endpoint_id);
    timer_free(s->resync_timer);
}

static const Property snb_i3c_bridge_props[] = {
    DEFINE_PROP_LINK("ctrl", SnbI3cBridgeState, ctrl, TYPE_I3C_CONTROLLER,
                     Object *),
    DEFINE_PROP_LINK("transport", SnbI3cBridgeState, transport,
                     TYPE_SNB_TRANSPORT, SnbTransportState *),
    DEFINE_PROP_UINT8("endpoint-id", SnbI3cBridgeState, endpoint_id,
                      SNB_ENDPOINT_I3C),
    DEFINE_PROP_STRING("bus-name", SnbI3cBridgeState, bus_name),
};

static void snb_i3c_bridge_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->desc = "Simulator network bridge I3C bus bridge";
    dc->realize = snb_i3c_bridge_realize;
    dc->unrealize = snb_i3c_bridge_unrealize;
    device_class_set_props(dc, snb_i3c_bridge_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo snb_i3c_bridge_types[] = {
    {
        .name = TYPE_SNB_I3C_BRIDGE,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(SnbI3cBridgeState),
        .class_init = snb_i3c_bridge_class_init,
    },
};

DEFINE_TYPES(snb_i3c_bridge_types)
