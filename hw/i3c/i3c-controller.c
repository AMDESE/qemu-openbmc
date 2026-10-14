/*
 * QOM interface for an I3C bus controller.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i3c/i3c-controller.h"
#include "qemu/module.h"

I3CBus *i3c_controller_get_bus(Object *obj)
{
    I3CControllerClass *k = I3C_CONTROLLER_GET_CLASS(obj);

    return k->get_bus(obj);
}

bool i3c_controller_inject_ibi(Object *obj, uint8_t da, uint8_t mdb,
                               bool has_mdb)
{
    I3CControllerClass *k = I3C_CONTROLLER_GET_CLASS(obj);

    return k->inject_ibi(obj, da, mdb, has_mdb);
}

static const TypeInfo i3c_controller_types[] = {
    {
        .name = TYPE_I3C_CONTROLLER,
        .parent = TYPE_INTERFACE,
        .class_size = sizeof(I3CControllerClass),
    },
};

DEFINE_TYPES(i3c_controller_types)
