/*
 * Simulator network bridge (SNB): I3C targets the peer holds
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SNB_SNB_I3C_H
#define HW_SNB_SNB_I3C_H

#include "hw/i2c/i2c.h"
#include "hw/i3c/i3c.h"
#include "hw/snb/snb.h"

/* An I3C target of the peer, created by snb-i3c-bus-bridge */
#define TYPE_SNB_I3C_TARGET "snb-i3c-target"
OBJECT_DECLARE_SIMPLE_TYPE(SnbI3cTarget, SNB_I3C_TARGET)

struct SnbI3cTarget {
    I3CTarget parent_obj;

    SnbServer *server;
    uint8_t endpoint_id;

    /* The dynamic address the peer was last told */
    uint8_t reported_da;

    /* Set by SETMWL and SETMRL, returned by GETMWL and GETMRL */
    uint16_t mwl;
    uint16_t mrl;
    uint8_t max_ibi_len;

    /* Run as ENTDAA starts, before the target presents its PID */
    void (*entdaa_hook)(void *opaque, I3CBus *bus);
    void *entdaa_opaque;
};

void snb_i3c_target_attach(SnbI3cTarget *t, SnbServer *server,
                           uint8_t endpoint_id,
                           void (*entdaa_hook)(void *opaque, I3CBus *bus),
                           void *opaque);
/*
 * Tell the peer the target's dynamic address again, for a new session.
 * Returns false if the peer did not take it by @deadline_ms.
 */
bool snb_i3c_target_resync(SnbI3cTarget *t, int64_t deadline_ms);

/* A legacy I2C target of the peer on an I3C bus */
#define TYPE_SNB_I3C_I2C_TARGET "snb-i3c-i2c-target"
OBJECT_DECLARE_SIMPLE_TYPE(SnbI3cI2cTarget, SNB_I3C_I2C_TARGET)

#define SNB_I3C_I2C_BUF_SIZE 256

struct SnbI3cI2cTarget {
    I2CSlave parent_obj;

    SnbServer *server;
    uint8_t endpoint_id;
    I3CBus *bus;

    uint8_t buf[SNB_I3C_I2C_BUF_SIZE];
    uint32_t len;
};

void snb_i3c_i2c_target_attach(SnbI3cI2cTarget *t, SnbServer *server,
                               uint8_t endpoint_id, I3CBus *bus);

#endif /* HW_SNB_SNB_I3C_H */
