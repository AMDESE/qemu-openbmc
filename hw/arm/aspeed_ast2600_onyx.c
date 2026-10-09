/*
 * AMD Onyx BMC
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
#include "hw/i2c/i2c_mux_pca954x.h"
#include "hw/nvram/eeprom_at24c.h"
#include "hw/sensor/tmp105.h"

/* Hawai hardware value, same as the AST2600 EVB for now */
#define HAWAI_BMC_HW_STRAP1 0x000000C0
#define HAWAI_BMC_HW_STRAP2 0x00000003

#define TYPE_LM75 TYPE_TMP105

static void onyx_bmc_i2c_init(AspeedMachineState *bmc)
{
    AspeedSoCState *soc = bmc->soc;
    I2CSlave *i2c_switch;
    I2CBus *i2c5 = aspeed_i2c_get_bus(&soc->i2c, 5);
    I2CBus *i2c9 = aspeed_i2c_get_bus(&soc->i2c, 9);
    I2CBus *i2c10 = aspeed_i2c_get_bus(&soc->i2c, 10);
    I2CBus *i2c14 = aspeed_i2c_get_bus(&soc->i2c, 14);
    I2CBus *mux;
    int i;

    /* Bus 5 */
    i2c_slave_create_simple(i2c5, "pca9546", 0x71);
    at24c_eeprom_init(i2c5, 0x50, 32 * KiB);

    /* Bus 9 */
    i2c_switch = i2c_slave_create_simple(i2c9, "pca9548", 0x70);
    /* Missing model emc2305, using emc1413 */
    for (i = 0; i < 4; i++) {
        i2c_slave_create_simple(pca954x_i2c_get_bus(i2c_switch, i),
                                "emc1413", 0x4d);
    }
    mux = pca954x_i2c_get_bus(i2c_switch, 5);
    for (i = 0x48; i <= 0x4f; i++) {
        i2c_slave_create_simple(mux, TYPE_LM75, i);
    }
    /* Missing model tmp468, using tmp423 */
    i2c_slave_create_simple(pca954x_i2c_get_bus(i2c_switch, 7),
                            "tmp423", 0x48);

    /* Bus 10 */
    i2c_slave_create_simple(i2c10, "pca9548", 0x70);

    i2c_switch = i2c_slave_create_simple(i2c10, "pca9546", 0x71);
    mux = pca954x_i2c_get_bus(i2c_switch, 0);
    i2c_slave_create_simple(mux, "pca9548", 0x72);
    at24c_eeprom_init(mux, 0x50, 32 * KiB);
    at24c_eeprom_init(mux, 0x51, 32 * KiB);

    /* Bus 14 */
    at24c_eeprom_init(i2c14, 0x50, 32 * KiB);
}

static void aspeed_machine_onyx_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    AspeedMachineClass *amc = ASPEED_MACHINE_CLASS(oc);

    mc->desc       = "AMD Onyx BMC (Cortex-A7)";
    amc->soc_name  = "ast2600-a3";
    amc->hw_strap1 = HAWAI_BMC_HW_STRAP1;
    amc->hw_strap2 = HAWAI_BMC_HW_STRAP2;
    amc->fmc_model = "w25q01jvq";
    amc->spi_model = "w25q512jv";
    amc->num_cs    = 2;
    amc->macs_mask = ASPEED_MAC2_ON;
    amc->i2c_init  = onyx_bmc_i2c_init;
    mc->default_ram_size = 1 * GiB;
    aspeed_machine_class_init_cpus_defaults(mc);
}

static const TypeInfo aspeed_ast2600_onyx_types[] = {
    {
        .name          = MACHINE_TYPE_NAME("onyx-bmc"),
        .parent        = TYPE_ASPEED_MACHINE,
        .class_init    = aspeed_machine_onyx_class_init,
        .interfaces    = arm_machine_interfaces,
    }
};

DEFINE_TYPES(aspeed_ast2600_onyx_types)
