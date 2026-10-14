/*
 * QOM interface for an I3C bus controller.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef QEMU_HW_I3C_I3C_CONTROLLER_H
#define QEMU_HW_I3C_I3C_CONTROLLER_H

#include "qom/object.h"
#include "hw/i3c/i3c.h"

#define TYPE_I3C_CONTROLLER "i3c-controller"

typedef struct I3CControllerClass I3CControllerClass;
DECLARE_CLASS_CHECKERS(I3CControllerClass, I3C_CONTROLLER, TYPE_I3C_CONTROLLER)

#define I3C_CONTROLLER(obj) \
    INTERFACE_CHECK(I3CController, (obj), TYPE_I3C_CONTROLLER)

typedef struct I3CController I3CController;

struct I3CControllerClass {
    InterfaceClass parent_class;

    /* Return the I3CBus hosted by this controller. */
    I3CBus *(*get_bus)(Object *obj);
    /* Inject an in-band interrupt on behalf of the given dynamic address. */
    bool (*inject_ibi)(Object *obj, uint8_t da, uint8_t mdb, bool has_mdb);
};

I3CBus *i3c_controller_get_bus(Object *obj);
bool i3c_controller_inject_ibi(Object *obj, uint8_t da, uint8_t mdb,
                               bool has_mdb);

#endif /* QEMU_HW_I3C_I3C_CONTROLLER_H */
