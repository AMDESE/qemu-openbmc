/*
 * AMD G406 BMC
 *
 * Copyright 2025 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/aspeed.h"
#include "hw/arm/aspeed_soc.h"

/* Malta hardware value, same as the AST2700 EVB for now */
/* SCU HW Strap1 */
#define MALTA_BMC_HW_STRAP1 0x00000800
/* SCUIO HW Strap1 */
#define MALTA_BMC_HW_STRAP2 0x00000700

static void aspeed_machine_g406_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    AspeedMachineClass *amc = ASPEED_MACHINE_CLASS(oc);

    mc->desc       = "AMD G406 BMC (Cortex-A35)";
    /* Silicon revision picked at configure time; see --amd-bmc-soc */
    amc->soc_name  = CONFIG_AMD_BMC_SOC_NAME;
    amc->hw_strap1 = MALTA_BMC_HW_STRAP1;
    amc->hw_strap2 = MALTA_BMC_HW_STRAP2;
    amc->fmc_model = "w25q01jvq";
    amc->spi_model = "w25q512jv";
    amc->num_cs    = 2;
    amc->macs_mask = ASPEED_MAC0_ON | ASPEED_MAC1_ON | ASPEED_MAC2_ON;
    amc->uart_default = ASPEED_DEV_UART12;
    mc->default_ram_size = 2 * GiB;
    aspeed_machine_class_init_cpus_defaults(mc);
}

static const TypeInfo aspeed_ast27x0_g406_types[] = {
    {
        .name          = MACHINE_TYPE_NAME("g406-bmc"),
        .parent        = TYPE_ASPEED_MACHINE,
        .class_init    = aspeed_machine_g406_class_init,
        .interfaces    = aarch64_machine_interfaces,
    }
};

DEFINE_TYPES(aspeed_ast27x0_g406_types)
