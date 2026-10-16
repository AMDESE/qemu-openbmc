/*
 * Mock host for the ASPEED eSPI controller
 *
 * Takes the eSPI controller (host) side of an AST2700 eSPI target, so that a
 * test can issue configuration, flash and virtual wire cycles and check what
 * the target answers. Every operation is a QOM property, set or read with
 * qom-set and qom-get:
 *
 *   config          get: GET_CONFIGURATION, set: SET_CONFIGURATION, both at
 *                   cfg-addr; config-ok tells whether the target took a set
 *   flash-read      set: read flash-len bytes at flash-addr with flash-tag
 *   flash-write     set: write flash-data at flash-addr with flash-tag
 *   flash-erase     set: erase at flash-addr, the value is the size code
 *   flash-poll      get: collect a completion software queued; true if one
 *                   was waiting
 *   flash-accepted  get: whether the target took the last cycle
 *   flash-rsp-cyc   get: completion type of the last cycle or poll
 *   flash-rsp-tag   get: tag of the last polled completion
 *   flash-data      get: data of the last completion, set: data to write;
 *                   a hex string, two digits per byte
 *   flash-done      set: finish a read the target answered at once
 *   vw-evt0/vw-evt1 get/set: the system event wires the host drives
 *   status          get: read the eSPI status word, as a GET_STATUS would
 *   alert           get: whether the target is signalling alert
 *   vw-count, vw-index, vw-data, alert-count
 *                   get: wires and alerts the target sent on its own
 *   perif-cycle     set: issue a peripheral cycle of this type with
 *                   perif-tag, perif-addr, perif-len, perif-msg and, for
 *                   cycles that carry data, flash-data as the payload;
 *                   perif-accepted and flash-rsp-cyc tell what the target
 *                   answered, flash-data holds the data it returned
 *   pc-count, pc-cyc, pc-tag, pc-addr, pc-len, pc-msg, pc-data
 *                   get: the last peripheral cycle the target sent on its
 *                   own, and how many it sent
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/aspeed_espi.h"
#include "qom/object.h"

#define TYPE_ASPEED_ESPI_MOCK_HOST "aspeed-espi-mock-host"
OBJECT_DECLARE_SIMPLE_TYPE(AspeedESPIMockHostState, ASPEED_ESPI_MOCK_HOST)

/* Flash channel request cycle types */
#define FLASH_READ      0x00
#define FLASH_WRITE     0x01
#define FLASH_ERASE     0x02

struct AspeedESPIMockHostState {
    DeviceState parent_obj;

    AspeedESPIState *espi;

    uint32_t cfg_addr;
    bool cfg_ok;

    uint32_t flash_addr;
    uint32_t flash_len;
    uint8_t flash_tag;
    bool flash_accepted;
    uint8_t rsp_cyc;
    uint8_t rsp_tag;
    uint8_t data[ASPEED_ESPI_MAX_PLD];
    uint32_t data_len;

    uint32_t vw_count;
    uint8_t vw_index;
    uint8_t vw_data;
    uint32_t alert_count;

    uint64_t perif_addr;
    uint32_t perif_len;
    uint8_t perif_tag;
    uint8_t perif_msg;
    bool perif_accepted;

    uint32_t pc_count;
    uint8_t pc_cyc;
    uint8_t pc_tag;
    uint64_t pc_addr;
    uint32_t pc_len;
    uint8_t pc_msg;
    uint8_t pc_data[ASPEED_ESPI_MAX_PLD];
    uint32_t pc_data_len;
};

static void mock_host_vw_notify(void *opaque, uint8_t index, uint8_t data)
{
    AspeedESPIMockHostState *s = opaque;

    s->vw_count++;
    s->vw_index = index;
    s->vw_data = data;
}

static void mock_host_alert_notify(void *opaque, uint32_t espi_status)
{
    AspeedESPIMockHostState *s = opaque;

    s->alert_count++;
}

static void mock_host_pc_notify(void *opaque, uint8_t cyc, uint8_t tag,
                                uint64_t addr, uint32_t len,
                                const uint8_t *data, uint8_t msg_code)
{
    AspeedESPIMockHostState *s = opaque;

    s->pc_count++;
    s->pc_cyc = cyc;
    s->pc_tag = tag;
    s->pc_addr = addr;
    s->pc_len = len;
    s->pc_msg = msg_code;
    s->pc_data_len = 0;
    if (data) {
        s->pc_data_len = MIN(len, sizeof(s->pc_data));
        memcpy(s->pc_data, data, s->pc_data_len);
    }
}

static const AspeedESPINotifier mock_host_notifier = {
    .vw_notify = mock_host_vw_notify,
    .alert_notify = mock_host_alert_notify,
    .pc_notify = mock_host_pc_notify,
};

static void mock_host_get_config(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    uint32_t val = aspeed_espi_get_config(s->espi, s->cfg_addr);

    visit_type_uint32(v, name, &val, errp);
}

static void mock_host_set_config(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    uint32_t val;

    if (visit_type_uint32(v, name, &val, errp)) {
        s->cfg_ok = aspeed_espi_set_config(s->espi, s->cfg_addr, val);
    }
}

/* Issue a flash cycle and keep what the target answered */
static void mock_host_flash_cycle(AspeedESPIMockHostState *s, uint8_t cyc,
                                  uint32_t len)
{
    uint8_t out[ASPEED_ESPI_MAX_PLD];
    uint32_t out_len = sizeof(out);

    s->flash_accepted = aspeed_espi_flash_cycle(s->espi, cyc, s->flash_tag,
                                                len, s->flash_addr, s->data,
                                                &s->rsp_cyc, out, &out_len);
    s->rsp_tag = s->flash_tag;
    if (cyc == FLASH_READ) {
        memcpy(s->data, out, out_len);
        s->data_len = out_len;
    }
}

static void mock_host_set_flash_read(Object *obj, Visitor *v,
                                     const char *name, void *opaque,
                                     Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    uint32_t len;

    if (visit_type_uint32(v, name, &len, errp)) {
        s->data_len = 0;
        mock_host_flash_cycle(s, FLASH_READ, len);
    }
}

static void mock_host_set_flash_write(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    bool go;

    if (visit_type_bool(v, name, &go, errp) && go) {
        mock_host_flash_cycle(s, FLASH_WRITE, s->data_len);
    }
}

static void mock_host_set_flash_erase(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    uint32_t code;

    if (visit_type_uint32(v, name, &code, errp)) {
        mock_host_flash_cycle(s, FLASH_ERASE, code);
    }
}

static void mock_host_get_flash_poll(Object *obj, Visitor *v,
                                     const char *name, void *opaque,
                                     Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    uint32_t len = sizeof(s->data);
    bool got = aspeed_espi_flash_get_completion(s->espi, &s->rsp_cyc,
                                                &s->rsp_tag, s->data, &len);

    if (got) {
        s->data_len = len;
    }
    visit_type_bool(v, name, &got, errp);
}

static void mock_host_set_flash_done(Object *obj, Visitor *v,
                                     const char *name, void *opaque,
                                     Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    bool go;

    if (visit_type_bool(v, name, &go, errp) && go) {
        aspeed_espi_flash_read_done(s->espi);
    }
}

static void mock_host_visit_hex(Visitor *v, const char *name,
                                const uint8_t *buf, uint32_t len,
                                Error **errp)
{
    g_autofree char *hex = g_malloc0(len * 2 + 1);
    uint32_t i;

    for (i = 0; i < len; i++) {
        snprintf(hex + i * 2, 3, "%02x", buf[i]);
    }
    visit_type_str(v, name, &hex, errp);
}

static void mock_host_get_flash_data(Object *obj, Visitor *v,
                                     const char *name, void *opaque,
                                     Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);

    mock_host_visit_hex(v, name, s->data, s->data_len, errp);
}

static void mock_host_set_flash_data(Object *obj, Visitor *v,
                                     const char *name, void *opaque,
                                     Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    g_autofree char *hex = NULL;
    size_t n, i;

    if (!visit_type_str(v, name, &hex, errp)) {
        return;
    }
    n = strlen(hex);
    if (n % 2 || n / 2 > sizeof(s->data)) {
        error_setg(errp, "%s: need an even number of hex digits, at most %zu",
                   name, sizeof(s->data) * 2);
        return;
    }
    for (i = 0; i < n / 2; i++) {
        unsigned int byte;

        if (sscanf(hex + i * 2, "%2x", &byte) != 1) {
            error_setg(errp, "%s: '%s' is not hex", name, hex);
            return;
        }
        s->data[i] = byte;
    }
    s->data_len = n / 2;
}

/* Issue a peripheral cycle and keep what the target answered */
static void mock_host_set_perif_cycle(Object *obj, Visitor *v,
                                      const char *name, void *opaque,
                                      Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    uint8_t out[ASPEED_ESPI_MAX_PLD];
    uint32_t out_len = sizeof(out);
    uint8_t cyc;

    if (!visit_type_uint8(v, name, &cyc, errp)) {
        return;
    }
    s->rsp_cyc = 0;
    s->perif_accepted = aspeed_espi_perif_cycle(s->espi, cyc, s->perif_tag,
                                                s->perif_addr, s->perif_len,
                                                s->data, s->perif_msg,
                                                &s->rsp_cyc, out, &out_len);
    s->rsp_tag = s->perif_tag;
    s->data_len = MIN(out_len, sizeof(s->data));
    memcpy(s->data, out, s->data_len);
}

static void mock_host_get_pc_data(Object *obj, Visitor *v, const char *name,
                                  void *opaque, Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);

    mock_host_visit_hex(v, name, s->pc_data, s->pc_data_len, errp);
}

static void mock_host_get_vw(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    uint32_t val = aspeed_espi_vw_get_event(s->espi, opaque != NULL);

    visit_type_uint32(v, name, &val, errp);
}

static void mock_host_set_vw(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    uint32_t val;

    if (visit_type_uint32(v, name, &val, errp)) {
        aspeed_espi_vw_set_event(s->espi, opaque != NULL, val);
    }
}

static void mock_host_get_status(Object *obj, Visitor *v, const char *name,
                                 void *opaque, Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    uint32_t val = aspeed_espi_get_status(s->espi);

    visit_type_uint32(v, name, &val, errp);
}

static void mock_host_get_alert(Object *obj, Visitor *v, const char *name,
                                void *opaque, Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);
    uint32_t status;
    bool alert = aspeed_espi_alert_pending(s->espi, &status);

    visit_type_bool(v, name, &alert, errp);
}

static bool mock_host_get_config_ok(Object *obj, Error **errp)
{
    return ASPEED_ESPI_MOCK_HOST(obj)->cfg_ok;
}

static bool mock_host_get_flash_accepted(Object *obj, Error **errp)
{
    return ASPEED_ESPI_MOCK_HOST(obj)->flash_accepted;
}

static bool mock_host_get_perif_accepted(Object *obj, Error **errp)
{
    return ASPEED_ESPI_MOCK_HOST(obj)->perif_accepted;
}

static void mock_host_realize(DeviceState *dev, Error **errp)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(dev);

    if (!s->espi) {
        error_setg(errp, "%s: the 'espi' link must be set",
                   TYPE_ASPEED_ESPI_MOCK_HOST);
        return;
    }
    aspeed_espi_set_notifier(s->espi, &mock_host_notifier, s);
}

static void mock_host_unrealize(DeviceState *dev)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(dev);

    aspeed_espi_set_notifier(s->espi, NULL, NULL);
}

static void mock_host_init(Object *obj)
{
    AspeedESPIMockHostState *s = ASPEED_ESPI_MOCK_HOST(obj);

    object_property_add(obj, "config", "uint32", mock_host_get_config,
                        mock_host_set_config, NULL, NULL);
    object_property_add(obj, "flash-read", "uint32", NULL,
                        mock_host_set_flash_read, NULL, NULL);
    object_property_add(obj, "flash-write", "bool", NULL,
                        mock_host_set_flash_write, NULL, NULL);
    object_property_add(obj, "flash-erase", "uint32", NULL,
                        mock_host_set_flash_erase, NULL, NULL);
    object_property_add(obj, "flash-poll", "bool", mock_host_get_flash_poll,
                        NULL, NULL, NULL);
    object_property_add(obj, "flash-done", "bool", NULL,
                        mock_host_set_flash_done, NULL, NULL);
    object_property_add(obj, "flash-data", "str", mock_host_get_flash_data,
                        mock_host_set_flash_data, NULL, NULL);
    object_property_add(obj, "vw-evt0", "uint32", mock_host_get_vw,
                        mock_host_set_vw, NULL, NULL);
    object_property_add(obj, "vw-evt1", "uint32", mock_host_get_vw,
                        mock_host_set_vw, NULL, (void *)1);
    object_property_add(obj, "status", "uint32", mock_host_get_status, NULL,
                        NULL, NULL);
    object_property_add(obj, "alert", "bool", mock_host_get_alert, NULL,
                        NULL, NULL);

    object_property_add_uint32_ptr(obj, "cfg-addr", &s->cfg_addr,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_bool(obj, "config-ok", mock_host_get_config_ok, NULL);
    object_property_add_uint32_ptr(obj, "flash-addr", &s->flash_addr,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint8_ptr(obj, "flash-tag", &s->flash_tag,
                                  OBJ_PROP_FLAG_READWRITE);
    object_property_add_bool(obj, "flash-accepted",
                             mock_host_get_flash_accepted, NULL);
    object_property_add_uint8_ptr(obj, "flash-rsp-cyc", &s->rsp_cyc,
                                  OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "flash-rsp-tag", &s->rsp_tag,
                                  OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "vw-count", &s->vw_count,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "vw-index", &s->vw_index,
                                  OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "vw-data", &s->vw_data,
                                  OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "alert-count", &s->alert_count,
                                   OBJ_PROP_FLAG_READ);

    object_property_add(obj, "perif-cycle", "uint8", NULL,
                        mock_host_set_perif_cycle, NULL, NULL);
    object_property_add_uint64_ptr(obj, "perif-addr", &s->perif_addr,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "perif-len", &s->perif_len,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint8_ptr(obj, "perif-tag", &s->perif_tag,
                                  OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint8_ptr(obj, "perif-msg", &s->perif_msg,
                                  OBJ_PROP_FLAG_READWRITE);
    object_property_add_bool(obj, "perif-accepted",
                             mock_host_get_perif_accepted, NULL);

    object_property_add_uint32_ptr(obj, "pc-count", &s->pc_count,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "pc-cyc", &s->pc_cyc,
                                  OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "pc-tag", &s->pc_tag,
                                  OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "pc-addr", &s->pc_addr,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "pc-len", &s->pc_len,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint8_ptr(obj, "pc-msg", &s->pc_msg,
                                  OBJ_PROP_FLAG_READ);
    object_property_add(obj, "pc-data", "str", mock_host_get_pc_data, NULL,
                        NULL, NULL);
}

static const Property mock_host_props[] = {
    DEFINE_PROP_LINK("espi", AspeedESPIMockHostState, espi, TYPE_ASPEED_ESPI,
                     AspeedESPIState *),
};

static void mock_host_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Mock host for the ASPEED eSPI controller";
    dc->realize = mock_host_realize;
    dc->unrealize = mock_host_unrealize;
    dc->user_creatable = true;
    device_class_set_props(dc, mock_host_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo mock_host_types[] = {
    {
        .name = TYPE_ASPEED_ESPI_MOCK_HOST,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(AspeedESPIMockHostState),
        .instance_init = mock_host_init,
        .class_init = mock_host_class_init,
    },
};

DEFINE_TYPES(mock_host_types)
