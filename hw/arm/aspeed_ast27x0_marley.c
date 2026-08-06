/*
 * AMD Marley BMC
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

/* Malta hardware value, same as the AST2700 EVB for now */
/* SCU HW Strap1 */
#define MALTA_BMC_HW_STRAP1 0x00000800
/* SCUIO HW Strap1 */
#define MALTA_BMC_HW_STRAP2 0x00000700

static const uint8_t marley_fruid[] = {
    0x01, 0x00, 0x01, 0x03, 0x06, 0x09, 0x00, 0xec, 0x01, 0x02, 0x11, 0xc3,
    0x41, 0x4d, 0x44, 0x00, 0xc1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96,
    0x01, 0x03, 0x00, 0x64, 0xaf, 0xe1, 0xc3, 0x41, 0x4d, 0x44, 0xc6, 0x4d,
    0x61, 0x72, 0x6c, 0x65, 0x79, 0x00, 0x00, 0x00, 0xc1, 0x00, 0x00, 0x82,
    0x01, 0x03, 0x00, 0xc3, 0x41, 0x4d, 0x44, 0xc6, 0x31, 0x50, 0x20, 0x48,
    0x50, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc1, 0x00, 0x00, 0x00, 0x5a,
    0xc1, 0x00, 0x0f, 0xea, 0x46, 0x7f, 0xa6, 0x00, 0x01, 0x02, 0x00, 0x10,
    0x01, 0x00, 0x10, 0x01, 0x02, 0x10, 0x79, 0x41, 0x5a, 0x2d, 0x50, 0x52,
    0x00, 0xc9, 0x50, 0x52, 0x41, 0x2d, 0x51, 0x5a, 0x2d, 0x53, 0x52, 0x00,
    0x00, 0xc1, 0x00, 0xab, 0xc1, 0x00, 0x0f, 0xea, 0x46, 0x7f, 0xa6, 0x00,
    0x01, 0x02, 0x00, 0x10, 0x01, 0x00, 0x10, 0x01, 0x02, 0x10, 0x79, 0x41,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
};
static const size_t marley_fruid_len = sizeof(marley_fruid);

static void marley_bmc_i2c_init(AspeedMachineState *bmc)
{
    AspeedSoCState *soc = bmc->soc;
    I2CSlave *i2c_switch;
    I2CBus *i2c10;
    int i;

    /*
     * Bus 7: the atmel 24c08 model does not behave as expected. Enable this
     * once the device model support is ready.
     */

    /* Bus 8 */
    at24c_eeprom_init_rom(aspeed_i2c_get_bus(&soc->i2c, 8), 0x50,
                          32 * KiB, marley_fruid, marley_fruid_len);

    /* Bus 10 */
    i2c10 = aspeed_i2c_get_bus(&soc->i2c, 10);
    i2c_switch = i2c_slave_create_simple(i2c10, "pca9548", 0x70);
    /* Missing model emc2305, using emc1413 */
    for (i = 0; i < 5; i++) {
        i2c_slave_create_simple(pca954x_i2c_get_bus(i2c_switch, i),
                                "emc1413", 0x4d);
    }
}

static void aspeed_machine_marley_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    AspeedMachineClass *amc = ASPEED_MACHINE_CLASS(oc);

    mc->desc       = "AMD Marley BMC (Cortex-A35)";
    amc->soc_name  = "ast2700-a1";
    amc->hw_strap1 = MALTA_BMC_HW_STRAP1;
    amc->hw_strap2 = MALTA_BMC_HW_STRAP2;
    amc->fmc_model = "w25q01jvq";
    amc->spi_model = "w25q512jv";
    amc->num_cs    = 2;
    amc->macs_mask = ASPEED_MAC0_ON | ASPEED_MAC1_ON | ASPEED_MAC2_ON;
    amc->uart_default = ASPEED_DEV_UART12;
    amc->i2c_init  = marley_bmc_i2c_init;
    mc->default_ram_size = 2 * GiB;
    aspeed_machine_class_init_cpus_defaults(mc);
}

static const TypeInfo aspeed_ast27x0_marley_types[] = {
    {
        .name          = MACHINE_TYPE_NAME("marley-bmc"),
        .parent        = TYPE_ASPEED_MACHINE,
        .class_init    = aspeed_machine_marley_class_init,
        .interfaces    = aarch64_machine_interfaces,
    }
};

DEFINE_TYPES(aspeed_ast27x0_marley_types)
