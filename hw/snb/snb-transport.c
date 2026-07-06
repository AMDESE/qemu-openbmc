/*
 * Simulator network bridge (SNB): the transport device
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/qapi-types-sockets.h"
#include "hw/core/qdev-properties.h"
#include "migration/blocker.h"
#include "system/system.h"
#include "snb-internal.h"

SnbServer *snb_transport_get_server(SnbTransportState *t)
{
    return t ? t->server : NULL;
}

/* Every device is realized by now, so every bridge has its endpoint */
static void snb_transport_machine_done(Notifier *notifier, void *data)
{
    SnbTransportState *s = container_of(notifier, SnbTransportState,
                                        machine_done);

    snb_server_start(s->server);
}

static SocketAddress *snb_transport_addr(SnbTransportState *s,
                                         const char *path, uint16_t port)
{
    SocketAddress *addr = g_new0(SocketAddress, 1);

    if (path) {
        addr->type = SOCKET_ADDRESS_TYPE_UNIX;
        addr->u.q_unix.path = g_strdup(path);
    } else {
        addr->type = SOCKET_ADDRESS_TYPE_INET;
        addr->u.inet.host = g_strdup(s->ip);
        addr->u.inet.port = g_strdup_printf("%u", port);
    }
    return addr;
}

static void snb_transport_realize(DeviceState *dev, Error **errp)
{
    SnbTransportState *s = SNB_TRANSPORT(dev);
    g_autoptr(SocketAddress) master = NULL;
    g_autoptr(SocketAddress) slave = NULL;
    bool unix_master = s->unix_master_path && s->unix_master_path[0];
    bool unix_slave = s->unix_slave_path && s->unix_slave_path[0];

    if (unix_master != unix_slave) {
        error_setg(errp, "set both 'unix-master-path' and "
                   "'unix-slave-path', or neither");
        return;
    }
    if (!unix_master && (!s->ip || !s->ip[0])) {
        error_setg(errp, "set 'ip', or 'unix-master-path' and "
                   "'unix-slave-path'");
        return;
    }

    error_setg(&s->migration_blocker,
               "%s: the state of the external simulator cannot migrate",
               TYPE_SNB_TRANSPORT);
    if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
        return;
    }

    master = snb_transport_addr(s, unix_master ? s->unix_master_path : NULL,
                                s->master_port);
    slave = snb_transport_addr(s, unix_slave ? s->unix_slave_path : NULL,
                               s->slave_port);
    s->server = snb_server_new(master, slave, s->stall_hold_ms);

    s->machine_done.notify = snb_transport_machine_done;
    qemu_add_machine_init_done_notifier(&s->machine_done);
}

static void snb_transport_unrealize(DeviceState *dev)
{
    SnbTransportState *s = SNB_TRANSPORT(dev);

    qemu_remove_machine_init_done_notifier(&s->machine_done);
    snb_server_free(s->server);
    s->server = NULL;
    migrate_del_blocker(&s->migration_blocker);
}

static const Property snb_transport_props[] = {
    DEFINE_PROP_STRING("ip", SnbTransportState, ip),
    DEFINE_PROP_UINT16("master-port", SnbTransportState, master_port, 5300),
    DEFINE_PROP_UINT16("slave-port", SnbTransportState, slave_port, 5301),
    DEFINE_PROP_STRING("unix-master-path", SnbTransportState,
                       unix_master_path),
    DEFINE_PROP_STRING("unix-slave-path", SnbTransportState, unix_slave_path),
    DEFINE_PROP_UINT32("stall-hold-ms", SnbTransportState, stall_hold_ms,
                       SNB_STALL_HOLD_MS_DEFAULT),
};

static void snb_transport_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->desc = "Simulator network bridge transport";
    dc->realize = snb_transport_realize;
    dc->unrealize = snb_transport_unrealize;
    device_class_set_props(dc, snb_transport_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo snb_transport_types[] = {
    {
        .name = TYPE_SNB_TRANSPORT,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(SnbTransportState),
        .class_init = snb_transport_class_init,
    },
};

DEFINE_TYPES(snb_transport_types)
