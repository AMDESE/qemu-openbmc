/*
 * Simulator network bridge (SNB): GPIO bridge
 *
 * Carries named signals between the peer and an LTPI GPIO controller. Each
 * pair of the controller has an input line, at an even offset, and an
 * output line, at an odd offset; "line-names" names them, separated by
 * semicolons, and leaves a line unnamed with an empty name. A signal the
 * peer sets drives the input of its pair; a change on a named output goes
 * to the peer.
 *
 * The level on the wire is logical, 1 asserted. A signal is active low if
 * "active-low" lists it, or, with "polarity-from-suffix", if its name ends
 * in _L. Named inputs start deasserted.
 *
 *   -device snb-gpio-bridge,transport=snb0,
 *           ltpi-gpio-path=/machine/soc/ltpi-gpio[0],
 *           line-names=";;;SIGNAL_A;SIGNAL_B_L"
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/resettable.h"
#include "hw/gpio/aspeed_ltpi_gpio.h"
#include "hw/snb/snb.h"
#include "system/reset.h"
#include "trace.h"

#define TYPE_SNB_GPIO_BRIDGE "snb-gpio-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(SnbGpioBridgeState, SNB_GPIO_BRIDGE)

struct SnbGpioBridgeState {
    DeviceState parent_obj;

    SnbTransportState *transport;
    SnbServer *server;
    uint8_t endpoint_id;
    char *bus_name;
    char *ltpi_gpio_path;
    char *line_names;
    char *active_low_list;
    bool polarity_from_suffix;

    AspeedLtpiGpioState *ltpi;
    /* Name and polarity of each line, by offset; NULL if unnamed */
    char **names;
    bool *active_low;
    uint32_t num_names;
    /* Input name to pair + 1 */
    GHashTable *in_pairs;
    qemu_irq out_sinks[ASPEED_LTPI_GPIO_NR_PAIRS];
};

static bool snb_gpio_bridge_suffix_low(const char *name)
{
    return g_str_has_suffix(name, "_L");
}

static void snb_gpio_bridge_free_names(SnbGpioBridgeState *s)
{
    g_clear_pointer(&s->in_pairs, g_hash_table_unref);
    g_clear_pointer(&s->names, g_strfreev);
    g_clear_pointer(&s->active_low, g_free);
    s->num_names = 0;
}

/* A name used again for the same direction keeps its first line */
static void snb_gpio_bridge_parse_names(SnbGpioBridgeState *s)
{
    g_autoptr(GHashTable) low = g_hash_table_new(g_str_hash, g_str_equal);
    g_autoptr(GHashTable) outs = g_hash_table_new(g_str_hash, g_str_equal);
    g_auto(GStrv) low_list = NULL;
    uint32_t i;

    s->in_pairs = g_hash_table_new(g_str_hash, g_str_equal);
    if (!s->line_names || !s->line_names[0]) {
        return;
    }
    if (s->active_low_list) {
        low_list = g_strsplit(s->active_low_list, ";", -1);
        for (i = 0; low_list[i]; i++) {
            if (low_list[i][0]) {
                g_hash_table_add(low, low_list[i]);
            }
        }
    }

    s->names = g_strsplit(s->line_names, ";", -1);
    s->num_names = MIN(g_strv_length(s->names),
                       ASPEED_LTPI_GPIO_NR_PAIRS * 2);
    s->active_low = g_new0(bool, s->num_names);
    for (i = 0; i < s->num_names; i++) {
        char *name = s->names[i];
        GHashTable *dir = (i & 1) ? outs : s->in_pairs;

        if (!name[0] || g_hash_table_contains(dir, name)) {
            if (name[0]) {
                warn_report("%s: '%s' is named again at line %u; the first "
                            "one is used", TYPE_SNB_GPIO_BRIDGE, name, i);
            }
            g_free(name);
            s->names[i] = g_strdup("");
            continue;
        }
        s->active_low[i] = g_hash_table_contains(low, name) ||
                           (s->polarity_from_suffix &&
                            snb_gpio_bridge_suffix_low(name));
        g_hash_table_insert(dir, name, GUINT_TO_POINTER(i / 2 + 1));
    }
}

static const char *snb_gpio_bridge_name(SnbGpioBridgeState *s,
                                        uint32_t offset)
{
    if (offset >= s->num_names || !s->names[offset][0]) {
        return NULL;
    }
    return s->names[offset];
}

static bool snb_gpio_bridge_gpio_set(void *opaque, const char *name,
                                     uint8_t level)
{
    SnbGpioBridgeState *s = opaque;
    gpointer val = g_hash_table_lookup(s->in_pairs, name);
    uint32_t pair;

    if (!val) {
        qemu_log_mask(LOG_UNIMP, "%s: no input named '%s'\n",
                      TYPE_SNB_GPIO_BRIDGE, name);
        return false;
    }
    pair = GPOINTER_TO_UINT(val) - 1;
    trace_snb_gpio_set(name, level, pair);
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(s->ltpi), "ltpi-gpio-in",
                                        pair),
                 s->active_low[pair * 2] ? !level : !!level);
    return true;
}

static const SnbPeerOps snb_gpio_bridge_ops = {
    .gpio_set = snb_gpio_bridge_gpio_set,
};

static void snb_gpio_bridge_notify(SnbGpioBridgeState *s, uint32_t pair,
                                   int level)
{
    uint32_t offset = pair * 2 + 1;
    const char *name = snb_gpio_bridge_name(s, offset);
    uint8_t logical;

    if (!name || !snb_server_connected(s->server)) {
        return;
    }
    logical = s->active_low[offset] ? !level : !!level;
    trace_snb_gpio_notify(name, logical, pair);
    snb_server_gpio_notify(s->server, s->endpoint_id, name, logical);
}

static void snb_gpio_bridge_out_changed(void *opaque, int pair, int level)
{
    snb_gpio_bridge_notify(opaque, pair, level);
}

/*
 * Send the level of every named output. Changes alone are not enough: an
 * output that keeps its reset level never changes, and the peer would
 * never learn it. This runs after every device has its reset values, and
 * again each time the transport connects.
 */
static void snb_gpio_bridge_snapshot(SnbGpioBridgeState *s)
{
    uint32_t pair;

    for (pair = 0; pair < ASPEED_LTPI_GPIO_NR_PAIRS; pair++) {
        snb_gpio_bridge_notify(s, pair,
                               aspeed_ltpi_gpio_out_level(s->ltpi, pair));
    }
}

static void snb_gpio_bridge_reset_exit(Object *obj, ResetType type)
{
    snb_gpio_bridge_snapshot(SNB_GPIO_BRIDGE(obj));
}

static void snb_gpio_bridge_connected(void *opaque)
{
    SnbGpioBridgeState *s = opaque;

    /* The peer forgets the bus binding when a session ends */
    if (s->bus_name && s->bus_name[0] &&
        !snb_server_bus_register(s->server, s->bus_name, SNB_BUS_GPIO,
                                 s->endpoint_id)) {
        warn_report("%s: the peer did not take bus '%s'",
                    TYPE_SNB_GPIO_BRIDGE, s->bus_name);
    }
    snb_gpio_bridge_snapshot(s);
}

typedef struct SnbGpioBridgeClash {
    SnbGpioBridgeState *self;
    SnbGpioBridgeState *other;
    uint32_t offset;
} SnbGpioBridgeClash;

/*
 * Several bridges may serve one controller, each naming its own lines.
 * Find another one naming a line this one names.
 */
static int snb_gpio_bridge_find_clash(Object *obj, void *opaque)
{
    SnbGpioBridgeClash *c = opaque;
    SnbGpioBridgeState *b = (SnbGpioBridgeState *)
        object_dynamic_cast(obj, TYPE_SNB_GPIO_BRIDGE);
    uint32_t i;

    if (!b || b == c->self || !DEVICE(b)->realized ||
        b->ltpi != c->self->ltpi) {
        return 0;
    }
    for (i = 0; i < MIN(b->num_names, c->self->num_names); i++) {
        if (snb_gpio_bridge_name(b, i) && snb_gpio_bridge_name(c->self, i)) {
            c->other = b;
            c->offset = i;
            return 1;
        }
    }
    return 0;
}

static void snb_gpio_bridge_realize(DeviceState *dev, Error **errp)
{
    SnbGpioBridgeState *s = SNB_GPIO_BRIDGE(dev);
    SnbGpioBridgeClash clash = { .self = s };
    uint32_t i;

    if (!s->transport) {
        error_setg(errp, "'transport' is not set");
        return;
    }
    if (!s->ltpi_gpio_path) {
        error_setg(errp, "'ltpi-gpio-path' is not set");
        return;
    }
    s->ltpi = (AspeedLtpiGpioState *)
        object_resolve_path_type(s->ltpi_gpio_path, TYPE_ASPEED_LTPI_GPIO,
                                 NULL);
    if (!s->ltpi) {
        error_setg(errp, "no LTPI GPIO controller at '%s'",
                   s->ltpi_gpio_path);
        return;
    }

    snb_gpio_bridge_parse_names(s);
    object_child_foreach_recursive(object_get_root(),
                                   snb_gpio_bridge_find_clash, &clash);
    if (clash.other) {
        error_setg(errp, "line %u ('%s') is already served by '%s'",
                   clash.offset, s->names[clash.offset],
                   object_get_canonical_path(OBJECT(clash.other)));
        snb_gpio_bridge_free_names(s);
        return;
    }

    s->server = snb_transport_get_server(s->transport);
    if (!snb_server_register_endpoint(s->server, s->endpoint_id,
                                      &snb_gpio_bridge_ops, s, errp)) {
        snb_gpio_bridge_free_names(s);
        return;
    }
    if (!snb_server_add_connect_notifier(s->server, snb_gpio_bridge_connected,
                                         s, errp)) {
        snb_server_unregister_endpoint(s->server, s->endpoint_id);
        snb_gpio_bridge_free_names(s);
        return;
    }

    for (i = 0; i < ASPEED_LTPI_GPIO_NR_PAIRS; i++) {
        if (snb_gpio_bridge_name(s, i * 2 + 1)) {
            s->out_sinks[i] = qemu_allocate_irq(snb_gpio_bridge_out_changed,
                                                s, i);
            qdev_connect_gpio_out_named(DEVICE(s->ltpi), "ltpi-gpio-out", i,
                                        s->out_sinks[i]);
        }
        if (snb_gpio_bridge_name(s, i * 2)) {
            qemu_set_irq(qdev_get_gpio_in_named(DEVICE(s->ltpi),
                                                "ltpi-gpio-in", i),
                         s->active_low[i * 2]);
        }
    }

    /* A device on no bus takes part in reset only when registered */
    qemu_register_resettable(OBJECT(dev));
}

static void snb_gpio_bridge_unrealize(DeviceState *dev)
{
    SnbGpioBridgeState *s = SNB_GPIO_BRIDGE(dev);
    uint32_t i;

    qemu_unregister_resettable(OBJECT(dev));
    snb_server_remove_connect_notifier(s->server, snb_gpio_bridge_connected,
                                       s);
    snb_server_unregister_endpoint(s->server, s->endpoint_id);
    for (i = 0; i < ASPEED_LTPI_GPIO_NR_PAIRS; i++) {
        if (s->out_sinks[i]) {
            qdev_connect_gpio_out_named(DEVICE(s->ltpi), "ltpi-gpio-out", i,
                                        NULL);
            qemu_free_irq(s->out_sinks[i]);
            s->out_sinks[i] = NULL;
        }
    }
    snb_gpio_bridge_free_names(s);
}

static const Property snb_gpio_bridge_props[] = {
    DEFINE_PROP_LINK("transport", SnbGpioBridgeState, transport,
                     TYPE_SNB_TRANSPORT, SnbTransportState *),
    DEFINE_PROP_UINT8("endpoint-id", SnbGpioBridgeState, endpoint_id,
                      SNB_ENDPOINT_GPIO),
    DEFINE_PROP_STRING("bus-name", SnbGpioBridgeState, bus_name),
    DEFINE_PROP_STRING("ltpi-gpio-path", SnbGpioBridgeState, ltpi_gpio_path),
    DEFINE_PROP_STRING("line-names", SnbGpioBridgeState, line_names),
    DEFINE_PROP_STRING("active-low", SnbGpioBridgeState, active_low_list),
    DEFINE_PROP_BOOL("polarity-from-suffix", SnbGpioBridgeState,
                     polarity_from_suffix, true),
};

static void snb_gpio_bridge_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Simulator network bridge GPIO bridge";
    dc->realize = snb_gpio_bridge_realize;
    dc->unrealize = snb_gpio_bridge_unrealize;
    device_class_set_props(dc, snb_gpio_bridge_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    rc->phases.exit = snb_gpio_bridge_reset_exit;
}

static const TypeInfo snb_gpio_bridge_types[] = {
    {
        .name = TYPE_SNB_GPIO_BRIDGE,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(SnbGpioBridgeState),
        .class_init = snb_gpio_bridge_class_init,
    },
};

DEFINE_TYPES(snb_gpio_bridge_types)
