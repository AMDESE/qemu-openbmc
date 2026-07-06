/*
 * Simulator network bridge (SNB)
 *
 * An snb-transport device holds one connection to an external simulator,
 * the peer: two stream sockets, one for the requests QEMU sends and the
 * answers to them, one for the requests the peer sends and the answers.
 * Bridge devices connect QEMU models to it; each registers an endpoint,
 * and the transport delivers the peer's requests for that endpoint to
 * its SnbPeerOps.
 *
 * The transport connects once machine creation is done and keeps
 * reconnecting, so QEMU runs whether or not the peer is up. Until it is
 * connected, requests to the peer fail and notifications are dropped.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SNB_SNB_H
#define HW_SNB_SNB_H

#include "hw/core/qdev.h"
#include "qom/object.h"
#include "hw/snb/snb-proto.h"

#define TYPE_SNB_TRANSPORT "snb-transport"
OBJECT_DECLARE_SIMPLE_TYPE(SnbTransportState, SNB_TRANSPORT)

typedef struct SnbServer SnbServer;

struct SnbTransportState {
    DeviceState parent_obj;

    SnbServer *server;
    Notifier machine_done;
    Error *migration_blocker;

    char *ip;
    uint16_t master_port;
    uint16_t slave_port;
    char *unix_master_path;
    char *unix_slave_path;
    uint32_t stall_hold_ms;
};

SnbServer *snb_transport_get_server(SnbTransportState *t);

/*
 * Requests the peer sends to an endpoint. Each runs with the BQL held and
 * returns false if it could not be carried out. None may send a request to
 * the peer: the peer may be waiting on its answer.
 *
 * @rsp_len is in/out for the eSPI cycles: the size of @rsp_data in, the
 * completion length out.
 */
typedef struct SnbPeerOps {
    bool (*i3c_ibi)(void *opaque, uint8_t da, uint8_t mdb, bool has_mdb);
    bool (*gpio_set)(void *opaque, const char *name, uint8_t level);
    bool (*espi_flash_cycle)(void *opaque, uint8_t cyc, uint8_t tag,
                             uint32_t addr, uint32_t len,
                             const uint8_t *data, uint8_t *rsp_cyc,
                             uint8_t *rsp_data, uint32_t *rsp_len,
                             uint32_t *espi_status);
    bool (*espi_flash_get_completion)(void *opaque, uint8_t *rsp_cyc,
                                      uint8_t *rsp_tag, uint8_t *rsp_data,
                                      uint32_t *rsp_len,
                                      uint32_t *espi_status);
    bool (*espi_vw_set)(void *opaque, uint8_t index, uint8_t data);
    bool (*espi_reset)(void *opaque, bool asserted);
    bool (*espi_get_status)(void *opaque, uint32_t *espi_status);
    bool (*espi_get_config)(void *opaque, uint16_t addr, uint32_t *value);
    bool (*espi_set_config)(void *opaque, uint16_t addr, uint32_t value);
    bool (*espi_pc_cycle)(void *opaque, uint8_t cyc, uint8_t tag,
                          uint64_t addr, uint32_t len, const uint8_t *data,
                          uint8_t msg_code, uint8_t *rsp_cyc,
                          uint8_t *rsp_data, uint32_t *rsp_len,
                          uint32_t *espi_status);
} SnbPeerOps;

bool snb_server_connected(SnbServer *s);

/* @endpoint_id must be unique on the transport */
bool snb_server_register_endpoint(SnbServer *s, uint8_t endpoint_id,
                                  const SnbPeerOps *ops, void *opaque,
                                  Error **errp);
void snb_server_unregister_endpoint(SnbServer *s, uint8_t endpoint_id);

/*
 * Call @cb with the BQL held each time the transport connects, and at once
 * if it is connected already.
 */
bool snb_server_add_connect_notifier(SnbServer *s, void (*cb)(void *opaque),
                                     void *opaque, Error **errp);
void snb_server_remove_connect_notifier(SnbServer *s,
                                        void (*cb)(void *opaque),
                                        void *opaque);

/*
 * Requests to the peer. Each waits for the answer and returns false if the
 * peer refused it, did not answer, or is not connected. A thread that holds
 * the BQL keeps running the peer's requests while it waits, and the guest
 * clock stands still once the wait exceeds the transport's stall-hold-ms.
 *
 * @deadline_ms is a QEMU_CLOCK_VIRTUAL time by which the answer must come,
 * or 0 for none; a request that misses it fails.
 */
bool snb_server_bus_register(SnbServer *s, const char *name,
                             uint32_t bus_class, uint8_t endpoint_id);

bool snb_server_i2c_write(SnbServer *s, uint8_t endpoint_id, uint8_t addr,
                          bool stop, const uint8_t *data, uint32_t len);
/* The peer answers with exactly @len bytes */
bool snb_server_i2c_read(SnbServer *s, uint8_t endpoint_id, uint8_t addr,
                         bool stop, uint8_t *data, uint32_t len);

typedef struct SnbI3cDevice {
    uint8_t static_addr;
    uint64_t pid;
    uint8_t bcr;
    uint8_t dcr;
} SnbI3cDevice;

/* Fill up to SNB_I3C_MAX_DEVS entries of @devs and set *@count */
bool snb_server_i3c_enum_i3c(SnbServer *s, uint8_t endpoint_id,
                             SnbI3cDevice *devs, unsigned *count,
                             int64_t deadline_ms);
/* Fill up to SNB_I3C_MAX_DEVS addresses into @addrs and set *@count */
bool snb_server_i3c_enum_i2c(SnbServer *s, uint8_t endpoint_id,
                             uint8_t *addrs, unsigned *count,
                             int64_t deadline_ms);
bool snb_server_i3c_write(SnbServer *s, uint8_t endpoint_id, uint8_t da,
                          const uint8_t *data, uint32_t len,
                          int64_t deadline_ms);
/* *@actual is the number of bytes the target produced, at most @len */
bool snb_server_i3c_read(SnbServer *s, uint8_t endpoint_id, uint8_t da,
                         uint8_t *data, uint32_t len, uint32_t *actual,
                         int64_t deadline_ms);
/* A read CCC fills @data with up to @len bytes */
bool snb_server_i3c_ccc(SnbServer *s, uint8_t endpoint_id, uint8_t ccc,
                        bool rnw, uint8_t da, uint8_t *data, uint32_t len,
                        int64_t deadline_ms);
bool snb_server_i3c_da_assigned(SnbServer *s, uint8_t endpoint_id,
                                uint64_t pid, uint8_t da,
                                int64_t deadline_ms);
bool snb_server_i3c_i2c_write(SnbServer *s, uint8_t endpoint_id,
                              uint8_t addr, const uint8_t *data,
                              uint32_t len, int64_t deadline_ms);
bool snb_server_i3c_i2c_read(SnbServer *s, uint8_t endpoint_id, uint8_t addr,
                             uint8_t *data, uint32_t len,
                             int64_t deadline_ms);

/*
 * Notifications to the peer. They do not wait, so they may be sent from a
 * device's MMIO handler, and return false only if the packet was not sent.
 */
bool snb_server_gpio_notify(SnbServer *s, uint8_t endpoint_id,
                            const char *name, uint8_t level);
bool snb_server_espi_vw_notify(SnbServer *s, uint8_t endpoint_id,
                               uint8_t index, uint8_t data);
bool snb_server_espi_alert(SnbServer *s, uint8_t endpoint_id,
                           uint32_t espi_status);
bool snb_server_espi_pc_notify(SnbServer *s, uint8_t endpoint_id,
                               uint8_t cyc, uint8_t tag, uint64_t addr,
                               uint32_t len, const uint8_t *data,
                               uint8_t msg_code);

#endif /* HW_SNB_SNB_H */
