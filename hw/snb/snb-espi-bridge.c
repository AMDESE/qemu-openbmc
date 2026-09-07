/*
 * Simulator network bridge (SNB): eSPI bridge
 *
 * Connects an eSPI controller the peer holds to an AST2700 eSPI target.
 * The peer's flash, peripheral, configuration, status, virtual wire and
 * reset requests go to the target; the virtual wires it drives, its alert
 * and the peripheral cycles it originates go to the peer. Every completion
 * carries the target's status word.
 *
 *   -device snb-espi-bridge,transport=snb0,espi-path=/machine/soc/espi[0]
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/aspeed_espi.h"
#include "hw/snb/snb.h"
#include "trace.h"

#define TYPE_SNB_ESPI_BRIDGE "snb-espi-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(SnbEspiBridgeState, SNB_ESPI_BRIDGE)

/*
 * Alert is a level the target holds until the controller reads its status,
 * but a notification to the peer can go unanswered: offer it again until
 * the target no longer owes one, backing off from 2 s to 32 s. Every
 * completion carries the status word anyway, so this only hurries the
 * controller along.
 */
#define SNB_ESPI_ALERT_RETRY_NS     (2 * NANOSECONDS_PER_SECOND)
#define SNB_ESPI_ALERT_RETRY_MAX_NS (32 * NANOSECONDS_PER_SECOND)

struct SnbEspiBridgeState {
    DeviceState parent_obj;

    SnbTransportState *transport;
    SnbServer *server;
    uint8_t endpoint_id;
    char *bus_name;
    char *espi_path;

    AspeedESPIState *espi;
    qemu_irq reset;
    QEMUTimer *alert_timer;
    int64_t alert_retry_ns;
};

static bool snb_espi_bridge_flash_cycle(void *opaque, uint8_t cyc,
                                        uint8_t tag, uint32_t addr,
                                        uint32_t len, const uint8_t *data,
                                        uint8_t *rsp_cyc, uint8_t *rsp_data,
                                        uint32_t *rsp_len,
                                        uint32_t *espi_status)
{
    SnbEspiBridgeState *s = opaque;
    bool ok = aspeed_espi_flash_cycle(s->espi, cyc, tag, len, addr, data,
                                      rsp_cyc, rsp_data, rsp_len);

    /* The status rides along; reading it here acknowledges nothing */
    *espi_status = aspeed_espi_peek_status(s->espi);
    /*
     * A read the target completed is answered in this response, so its
     * completion must not still show as available. A read the target left
     * to software comes back with no data and stays pending.
     */
    if (ok && cyc == SNB_ESPI_FLASH_READ && *rsp_len) {
        aspeed_espi_flash_read_done(s->espi);
    }
    trace_snb_espi_flash_cycle(cyc, tag, addr, len, *rsp_cyc, *rsp_len, ok);
    return ok;
}

static bool snb_espi_bridge_flash_get_completion(void *opaque,
                                                 uint8_t *rsp_cyc,
                                                 uint8_t *rsp_tag,
                                                 uint8_t *rsp_data,
                                                 uint32_t *rsp_len,
                                                 uint32_t *espi_status)
{
    SnbEspiBridgeState *s = opaque;
    bool ok = aspeed_espi_flash_get_completion(s->espi, rsp_cyc, rsp_tag,
                                               rsp_data, rsp_len);

    *espi_status = aspeed_espi_peek_status(s->espi);
    trace_snb_espi_flash_completion(ok ? *rsp_cyc : 0, ok ? *rsp_len : 0, ok);
    return ok;
}

static bool snb_espi_bridge_pc_cycle(void *opaque, uint8_t cyc, uint8_t tag,
                                     uint64_t addr, uint32_t len,
                                     const uint8_t *data, uint8_t msg_code,
                                     uint8_t *rsp_cyc, uint8_t *rsp_data,
                                     uint32_t *rsp_len,
                                     uint32_t *espi_status)
{
    SnbEspiBridgeState *s = opaque;
    bool ok = aspeed_espi_perif_cycle(s->espi, cyc, tag, addr, len, data,
                                      msg_code, rsp_cyc, rsp_data, rsp_len);

    *espi_status = aspeed_espi_peek_status(s->espi);
    trace_snb_espi_pc_cycle(cyc, tag, addr, len, *rsp_cyc, *rsp_len, ok);
    return ok;
}

/*
 * The system event wires the controller drives, by virtual wire index, and
 * the bits of the target's System Event register that hold wires 0 to 2.
 * Bits 6:4 of the data byte say which of the wires are valid.
 */
static const struct {
    uint8_t index;
    uint8_t evt_bit[3];
} snb_espi_bridge_sysevt[] = {
    { 2, { 0, 1, 2 } },     /* SLP_S3#, SLP_S4#, SLP_S5# */
    { 3, { 4, 5, 6 } },     /* SUS_STAT#, PLTRST#, OOB_RST_WARN */
    { 7, { 8, 9, 10 } },    /* HOST_RST_WARN, SMIOUT#, NMIOUT# */
};

static bool snb_espi_bridge_vw_set(void *opaque, uint8_t index, uint8_t data)
{
    SnbEspiBridgeState *s = opaque;
    uint32_t evt;
    int i, w;

    trace_snb_espi_vw_set(index, data);
    for (i = 0; i < ARRAY_SIZE(snb_espi_bridge_sysevt); i++) {
        if (snb_espi_bridge_sysevt[i].index != index) {
            continue;
        }
        evt = aspeed_espi_vw_get_event(s->espi, false);
        for (w = 0; w < 3; w++) {
            if (data & BIT(w + 4)) {
                evt = deposit32(evt, snb_espi_bridge_sysevt[i].evt_bit[w], 1,
                                extract32(data, w, 1));
            }
        }
        aspeed_espi_vw_set_event(s->espi, false, evt);
        return true;
    }
    qemu_log_mask(LOG_UNIMP, "%s: virtual wire index %u is not a system "
                  "event the controller drives\n", TYPE_SNB_ESPI_BRIDGE,
                  index);
    return false;
}

static bool snb_espi_bridge_reset(void *opaque, bool asserted)
{
    SnbEspiBridgeState *s = opaque;

    trace_snb_espi_reset(asserted);
    qemu_set_irq(s->reset, asserted);
    return true;
}

static bool snb_espi_bridge_get_status(void *opaque, uint32_t *espi_status)
{
    SnbEspiBridgeState *s = opaque;

    *espi_status = aspeed_espi_get_status(s->espi);
    return true;
}

static bool snb_espi_bridge_get_config(void *opaque, uint16_t addr,
                                       uint32_t *value)
{
    SnbEspiBridgeState *s = opaque;

    *value = aspeed_espi_get_config(s->espi, addr);
    trace_snb_espi_get_config(addr, *value);
    return true;
}

static bool snb_espi_bridge_set_config(void *opaque, uint16_t addr,
                                       uint32_t value)
{
    SnbEspiBridgeState *s = opaque;

    trace_snb_espi_set_config(addr, value);
    return aspeed_espi_set_config(s->espi, addr, value);
}

static const SnbPeerOps snb_espi_bridge_ops = {
    .espi_flash_cycle = snb_espi_bridge_flash_cycle,
    .espi_flash_get_completion = snb_espi_bridge_flash_get_completion,
    .espi_vw_set = snb_espi_bridge_vw_set,
    .espi_reset = snb_espi_bridge_reset,
    .espi_get_status = snb_espi_bridge_get_status,
    .espi_get_config = snb_espi_bridge_get_config,
    .espi_set_config = snb_espi_bridge_set_config,
    .espi_pc_cycle = snb_espi_bridge_pc_cycle,
};

static void snb_espi_bridge_vw_notify(void *opaque, uint8_t index,
                                      uint8_t data)
{
    SnbEspiBridgeState *s = opaque;

    if (snb_server_connected(s->server)) {
        trace_snb_espi_vw_notify(index, data);
        snb_server_espi_vw_notify(s->server, s->endpoint_id, index, data);
    }
}

/*
 * Offer the alert again while the target still owes one. This is upkeep of
 * the link, not anything the target does, so it runs on host time.
 */
static void snb_espi_bridge_alert_retry(void *opaque)
{
    SnbEspiBridgeState *s = opaque;
    uint32_t status;

    if (!snb_server_connected(s->server) ||
        !aspeed_espi_alert_pending(s->espi, &status)) {
        return;
    }
    trace_snb_espi_alert(status);
    if (!snb_server_espi_alert(s->server, s->endpoint_id, status)) {
        return;
    }
    timer_mod(s->alert_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + s->alert_retry_ns);
    s->alert_retry_ns = MIN(s->alert_retry_ns * 2,
                            SNB_ESPI_ALERT_RETRY_MAX_NS);
}

static void snb_espi_bridge_alert_notify(void *opaque, uint32_t espi_status)
{
    SnbEspiBridgeState *s = opaque;

    /* A new status, so start again from the shortest interval */
    s->alert_retry_ns = SNB_ESPI_ALERT_RETRY_NS;
    snb_espi_bridge_alert_retry(s);
}

static void snb_espi_bridge_pc_notify(void *opaque, uint8_t cyc, uint8_t tag,
                                      uint64_t addr, uint32_t len,
                                      const uint8_t *data, uint8_t msg_code)
{
    SnbEspiBridgeState *s = opaque;

    if (snb_server_connected(s->server)) {
        trace_snb_espi_pc_notify(cyc, tag, addr, len);
        snb_server_espi_pc_notify(s->server, s->endpoint_id, cyc, tag, addr,
                                  len, data, msg_code);
    }
}

static const AspeedESPINotifier snb_espi_bridge_notifier = {
    .vw_notify = snb_espi_bridge_vw_notify,
    .alert_notify = snb_espi_bridge_alert_notify,
    .pc_notify = snb_espi_bridge_pc_notify,
};

/*
 * The peer forgets the bus binding when a session ends, and an alert that
 * was due while the link was down has to be offered again.
 */
static void snb_espi_bridge_connected(void *opaque)
{
    SnbEspiBridgeState *s = opaque;

    if (s->bus_name && s->bus_name[0] &&
        !snb_server_bus_register(s->server, s->bus_name, SNB_BUS_ESPI,
                                 s->endpoint_id)) {
        warn_report("%s: the peer did not take bus '%s'",
                    TYPE_SNB_ESPI_BRIDGE, s->bus_name);
    }
    s->alert_retry_ns = SNB_ESPI_ALERT_RETRY_NS;
    snb_espi_bridge_alert_retry(s);
}

static void snb_espi_bridge_realize(DeviceState *dev, Error **errp)
{
    SnbEspiBridgeState *s = SNB_ESPI_BRIDGE(dev);

    if (!s->transport) {
        error_setg(errp, "'transport' is not set");
        return;
    }
    if (!s->espi_path) {
        error_setg(errp, "'espi-path' is not set");
        return;
    }
    s->espi = (AspeedESPIState *)
        object_resolve_path_type(s->espi_path, TYPE_ASPEED_ESPI, NULL);
    if (!s->espi) {
        error_setg(errp, "no eSPI controller at '%s'", s->espi_path);
        return;
    }
    s->reset = qdev_get_gpio_in_named(DEVICE(s->espi),
                                      ASPEED_ESPI_RESET_GPIO, 0);

    s->server = snb_transport_get_server(s->transport);
    if (!snb_server_register_endpoint(s->server, s->endpoint_id,
                                      &snb_espi_bridge_ops, s, errp)) {
        return;
    }
    s->alert_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                  snb_espi_bridge_alert_retry, s);
    if (!snb_server_add_connect_notifier(s->server,
                                         snb_espi_bridge_connected, s,
                                         errp)) {
        timer_free(s->alert_timer);
        snb_server_unregister_endpoint(s->server, s->endpoint_id);
        return;
    }
    aspeed_espi_set_notifier(s->espi, &snb_espi_bridge_notifier, s);
}

static void snb_espi_bridge_unrealize(DeviceState *dev)
{
    SnbEspiBridgeState *s = SNB_ESPI_BRIDGE(dev);

    aspeed_espi_set_notifier(s->espi, NULL, NULL);
    snb_server_remove_connect_notifier(s->server, snb_espi_bridge_connected,
                                       s);
    snb_server_unregister_endpoint(s->server, s->endpoint_id);
    timer_free(s->alert_timer);
}

static const Property snb_espi_bridge_props[] = {
    DEFINE_PROP_LINK("transport", SnbEspiBridgeState, transport,
                     TYPE_SNB_TRANSPORT, SnbTransportState *),
    DEFINE_PROP_UINT8("endpoint-id", SnbEspiBridgeState, endpoint_id,
                      SNB_ENDPOINT_ESPI),
    DEFINE_PROP_STRING("bus-name", SnbEspiBridgeState, bus_name),
    DEFINE_PROP_STRING("espi-path", SnbEspiBridgeState, espi_path),
};

static void snb_espi_bridge_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->desc = "Simulator network bridge eSPI bridge";
    dc->realize = snb_espi_bridge_realize;
    dc->unrealize = snb_espi_bridge_unrealize;
    device_class_set_props(dc, snb_espi_bridge_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo snb_espi_bridge_types[] = {
    {
        .name = TYPE_SNB_ESPI_BRIDGE,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(SnbEspiBridgeState),
        .class_init = snb_espi_bridge_class_init,
    },
};

DEFINE_TYPES(snb_espi_bridge_types)
