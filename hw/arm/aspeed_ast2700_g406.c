/*
 * ASPEED AST27x0 EVB
 *
 * Copyright 2016 IBM Corp.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/aspeed.h"
#include "hw/arm/aspeed_soc.h"
#include "hw/sensor/tmp105.h"

/* AST2700 evb hardware value */
/* SCU HW Strap1 */
#define AST2700_G406_STRAP1 0x00000800
/* SCUIO HW Strap1 */
#define AST2700_G406_STRAP2 0x00000700

static void aspeed_machine_g406_class_init(ObjectClass *oc,
                                                    const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    AspeedMachineClass *amc = ASPEED_MACHINE_CLASS(oc);

    mc->alias = "ast2700-g406";
    mc->desc = "AMD G406 BMC (Cortex-A35)";
    amc->soc_name  = "ast2700-a1";
    amc->hw_strap1 = AST2700_G406_STRAP1;
    amc->hw_strap2 = AST2700_G406_STRAP2;
    amc->fmc_model = "w25q01jvq";
    amc->spi_model = "w25q512jv";
    amc->num_cs    = 2;
    amc->macs_mask = ASPEED_MAC0_ON | ASPEED_MAC1_ON | ASPEED_MAC2_ON;
    amc->uart_default = ASPEED_DEV_UART12;
    amc->vbootrom = true;
    mc->default_ram_size = 2 * GiB;
    aspeed_machine_class_init_cpus_defaults(mc);
}

static const TypeInfo aspeed_ast2700_g406_types[] = {
    {
        .name          = MACHINE_TYPE_NAME("g406-bmc"),
        .parent        = TYPE_ASPEED_MACHINE,
        .class_init    = aspeed_machine_g406_class_init,
        .interfaces    = aarch64_machine_interfaces,
    }
};

DEFINE_TYPES(aspeed_ast2700_g406_types)
