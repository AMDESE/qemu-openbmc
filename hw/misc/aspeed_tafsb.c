/*
 * ASPEED AST2700 TAFS/SAFS Bridge Controller
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The bridge, also called the eDAF bridge, sits behind the eSPI flash
 * channel and carries the flash cycles serviced in hardware onto the BMC's
 * AHB. It holds two 40-bit base addresses, one for a SPI controller's
 * register block and one for that controller's flash window, and a set of
 * command descriptors naming the SPI instructions to use.
 *
 * In SPI mode the bridge drives the SPI controller: each operation is sent
 * in user mode with the command byte, address and dummy bytes taken from a
 * descriptor. In memory mode MBASE points at plain memory, which is read and
 * written directly, and an erase writes the fill selected in the global
 * controller.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/aspeed_tafsb.h"
#include "hw/misc/aspeed_tafsc.h"
#include "hw/ssi/aspeed_smc.h"
#include "migration/vmstate.h"
#include "trace.h"

#define TO_REG(offset) ((offset) >> 2)

#define TAFSB_CFG                0x000
#define   TAFSB_CFG_CMD_STOP     BIT(1)
#define   TAFSB_CFG_CMD_EN       BIT(0)
#define   TAFSB_CFG_WMASK        MAKE_64BIT_MASK(0, 2)
#define TAFSB_WLOCK              0x00c
#define   TAFSB_WLOCK_MASK       MAKE_64BIT_MASK(0, 4)
/*
 * Bit 0 locks the two base addresses, bit 1 the command descriptors and the
 * register carrying the high bytes of both bases. Bits 2 and 3 are writable
 * and lock nothing in this block.
 */
#define   TAFSB_WLOCK_BASE       BIT(0)
#define   TAFSB_WLOCK_CSET       BIT(1)
#define TAFSB_CBASE              0x020
#define TAFSB_MBASE              0x024
#define TAFSB_CSET0              0x040   /* read */
#define TAFSB_CSET1              0x044   /* write enable */
#define TAFSB_CSET2              0x048   /* page program */
#define TAFSB_CSET3              0x04c   /* read status */
#define TAFSB_CSET4              0x050   /* sector erase */
#define TAFSB_CSET5              0x054   /* 32 KiB erase */
#define TAFSB_CSET6              0x058   /* 64 KiB erase */
#define TAFSB_MISC               0x070
#define   TAFSB_MISC_MBASE_HI    MAKE_64BIT_MASK(8, 8)
#define   TAFSB_MISC_CBASE_HI    MAKE_64BIT_MASK(0, 8)
#define   TAFSB_MISC_WMASK       MAKE_64BIT_MASK(0, 16)
#define TAFSB_INFO               0x080
#define TAFSB_DBG                0x0f0

/*
 * Command descriptor layout. A descriptor has the layout of the SPI
 * controller's chip-enable control register: the command byte, the clock
 * rate, the dummy byte count (bits 14 and 7:6), the byte expansion pattern
 * for the address and data phases, a control that drops chip select as soon
 * as the transfer ends, and the command mode. The reset descriptors select
 * user mode, in which the bridge sends the command byte, the address and the
 * dummy bytes itself.
 */
#define TAFSB_CSET_OPCODE_SHIFT  16
#define TAFSB_CSET_OPCODE_MASK   0xff
#define TAFSB_CSET_CLKRATE_SHIFT 8
#define TAFSB_CSET_CLKRATE_MASK  0xf
#define TAFSB_CSET_DUMMY_HI      BIT(14)
#define TAFSB_CSET_DUMMY_LO_SHIFT 6
#define TAFSB_CSET_DUMMY_LO_MASK 0x3
#define TAFSB_CSET_LSB_FIRST     BIT(5)
#define TAFSB_CSET_BEB_SHIFT     3
#define TAFSB_CSET_BEB_MASK      0x3
#define TAFSB_CSET_CE_STOP       BIT(2)
#define TAFSB_CSET_MODE_MASK     0x3

/* Chip-enable control registers of the SPI controller at CBASE */
#define TAFSB_SPI_CE0_CTRL       0x10  /* CE n control is four bytes further */
#define TAFSB_SPI_CE_CTRL_STRIDE 4

#define TAFSB_FLASH_PAGE_SIZE    256
#define TAFSB_FLASH_SR_WIP       BIT(0)
/* Bound the busy poll so a stuck part cannot hold the BQL forever */
#define TAFSB_FLASH_WIP_POLLS    100000

#define TAFSB_CSET_MODE_AUTO_RD  0
#define TAFSB_CSET_MODE_NORM_RD  1
#define TAFSB_CSET_MODE_NORM_WR  2
#define TAFSB_CSET_MODE_USER     3

/*
 * Reset values. The descriptors are read, write enable, page program, read
 * status, and segment, 32 KiB and 64 KiB erase; the ones that take an address
 * use the four-byte address variant.
 */
static const struct {
    hwaddr   offset;
    uint32_t value;
    const char *name;
} aspeed_tafsb_reset_values[] = {
    { TAFSB_CFG,   0x00000001, "CFG" },
    { TAFSB_WLOCK, 0x00000000, "WLOCK" },
    { TAFSB_CBASE, 0x00010000, "CBASE" },
    { TAFSB_MBASE, 0x00000000, "MBASE" },
    { TAFSB_CSET0, 0x00130703, "CSET0 RDI" },
    { 0x044,      0x0006071e, "CSET1 WRENI" },
    { 0x048,      0x00120703, "CSET2 PPI" },
    { 0x04c,      0x00050715, "CSET3 RDSRI" },
    { 0x050,      0x0021070e, "CSET4 ERASEI segment" },
    { 0x054,      0x005c070e, "CSET5 ERASEI 32K" },
    { 0x058,      0x00dc070e, "CSET6 ERASEI 64K" },
    { TAFSB_MISC,  0x00000000, "MISC" },
    { TAFSB_INFO,  0x00000000, "INFO" },
    { TAFSB_DBG,   0x00000000, "DBG" },
};

/*
 * Both base registers are 40 bits: the low 32 bits in CBASE or MBASE and the
 * top 8 in a byte of MISC. The memory base points at the flash window of a
 * SPI controller and the controller base at that controller's registers.
 */
static uint64_t aspeed_tafsb_base(AspeedTAFSBState *s,
                                  hwaddr reg, int hi_shift)
{
    uint32_t misc = s->regs[TO_REG(TAFSB_MISC)];
    uint64_t hi = (misc >> hi_shift) & 0xff;

    return (hi << 32) | s->regs[TO_REG(reg)];
}

uint64_t aspeed_tafsb_mem_base(AspeedTAFSBState *s)
{
    return aspeed_tafsb_base(s, TAFSB_MBASE, 8);
}

static bool aspeed_tafsb_memory_mode(AspeedTAFSBState *s)
{
    return !(s->regs[TO_REG(TAFSB_CFG)] & TAFSB_CFG_CMD_EN);
}

/* Does this opcode take a four-byte address? */
static bool aspeed_tafsb_opcode_is_4b(uint32_t opcode)
{
    switch (opcode) {
    case 0x0c: /* FAST_READ4B */
    case 0x12: /* PP4B */
    case 0x13: /* READ4B */
    case 0x21: /* SE4B */
    case 0x34: /* QPP4B */
    case 0x3c: /* DOR4B */
    case 0x5c: /* BE32K4B */
    case 0x6c: /* QOR4B */
    case 0xdc: /* BE64K4B */
        return true;
    default:
        return false;
    }
}

static MemTxResult aspeed_tafsb_spi_ctrl(AspeedTAFSBState *s,
                                         uint32_t val)
{
    uint64_t cbase = aspeed_tafsb_base(s, TAFSB_CBASE, 0);
    hwaddr ce_reg = TAFSB_SPI_CE0_CTRL + (s->cs * TAFSB_SPI_CE_CTRL_STRIDE);
    uint8_t buf[4];

    stl_le_p(buf, val);
    return address_space_write(&s->ahb_as, cbase + ce_reg,
                               MEMTXATTRS_UNSPECIFIED, buf, 4);
}

/*
 * Issue one descriptor's command in SPI user mode: the command byte, the
 * address if the command takes one, then for a read the descriptor's dummy
 * bytes. The chip select's control register is restored afterwards.
 */
static bool aspeed_tafsb_spi_cmd(AspeedTAFSBState *s, hwaddr cset,
                                 bool has_addr, uint32_t addr,
                                 const uint8_t *tx, uint32_t tx_len,
                                 uint8_t *rx, uint32_t rx_len)
{
    uint64_t cbase = aspeed_tafsb_base(s, TAFSB_CBASE, 0);
    uint64_t win = aspeed_tafsb_mem_base(s);
    hwaddr ce_reg = TAFSB_SPI_CE0_CTRL + (s->cs * TAFSB_SPI_CE_CTRL_STRIDE);
    uint32_t desc = s->regs[TO_REG(cset)];
    uint32_t opcode = (desc >> TAFSB_CSET_OPCODE_SHIFT) &
                      TAFSB_CSET_OPCODE_MASK;
    uint32_t user = (desc & ~TAFSB_CSET_MODE_MASK) | TAFSB_CSET_MODE_USER;
    uint32_t dummy = (desc & TAFSB_CSET_DUMMY_HI ? 4 : 0) |
                     ((desc >> TAFSB_CSET_DUMMY_LO_SHIFT) &
                      TAFSB_CSET_DUMMY_LO_MASK);
    uint8_t hdr[12] = { opcode };
    uint32_t hdr_len = 1;
    uint32_t saved;
    uint8_t buf[4];
    MemTxResult res;

    if (address_space_read(&s->ahb_as, cbase + ce_reg, MEMTXATTRS_UNSPECIFIED,
                           buf, 4) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: cannot reach the SPI controller at 0x%" PRIx64
                      "; is CBASE right?\n", __func__, cbase);
        return false;
    }
    saved = ldl_le_p(buf);

    if (has_addr) {
        if (aspeed_tafsb_opcode_is_4b(opcode)) {
            stl_be_p(&hdr[1], addr);
            hdr_len += 4;
        } else {
            hdr[1] = addr >> 16;
            hdr[2] = addr >> 8;
            hdr[3] = addr;
            hdr_len += 3;
        }
    }
    if (rx_len) {
        hdr_len += dummy;
    }

    res = aspeed_tafsb_spi_ctrl(s, user | TAFSB_CSET_CE_STOP);
    res |= aspeed_tafsb_spi_ctrl(s, user);
    res |= address_space_write(&s->ahb_as, win, MEMTXATTRS_UNSPECIFIED,
                               hdr, hdr_len);
    if (tx_len) {
        res |= address_space_write(&s->ahb_as, win, MEMTXATTRS_UNSPECIFIED,
                                   tx, tx_len);
    }
    if (rx_len) {
        res |= address_space_read(&s->ahb_as, win, MEMTXATTRS_UNSPECIFIED,
                                  rx, rx_len);
    }
    res |= aspeed_tafsb_spi_ctrl(s, user | TAFSB_CSET_CE_STOP);
    aspeed_tafsb_spi_ctrl(s, saved);

    trace_aspeed_tafsb_spi_cmd(opcode, addr, tx_len, rx_len, res);
    return res == MEMTX_OK;
}

/* WREN, then @cset, then poll RDSR until the flash is idle */
static bool aspeed_tafsb_spi_update(AspeedTAFSBState *s,
                                    hwaddr cset, uint32_t addr,
                                    const uint8_t *data, uint32_t len)
{
    uint8_t sr;
    int i;

    if (!aspeed_tafsb_spi_cmd(s, TAFSB_CSET1, false, 0, NULL, 0,
                              NULL, 0) ||
        !aspeed_tafsb_spi_cmd(s, cset, true, addr, data, len,
                              NULL, 0)) {
        return false;
    }

    for (i = 0; i < TAFSB_FLASH_WIP_POLLS; i++) {
        if (!aspeed_tafsb_spi_cmd(s, TAFSB_CSET3, false, 0, NULL, 0,
                                  &sr, 1)) {
            return false;
        }
        if (!(sr & TAFSB_FLASH_SR_WIP)) {
            return true;
        }
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: the flash is still busy after %d status reads\n",
                  __func__, TAFSB_FLASH_WIP_POLLS);
    return false;
}

/* Offset of @addr from MBASE, as the flash sees it */
static bool aspeed_tafsb_flash_offset(AspeedTAFSBState *s,
                                      uint64_t addr, uint32_t len,
                                      uint32_t *offset)
{
    uint64_t base = aspeed_tafsb_mem_base(s);
    uint64_t rel = addr - base;

    /* The last byte, not the end, must have a 32-bit flash offset */
    if (addr < base || rel > UINT32_MAX ||
        (len && len - 1 > UINT32_MAX - rel)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: AHB address 0x%" PRIx64 " is outside the flash "
                      "window at 0x%" PRIx64 "\n", __func__, addr, base);
        return false;
    }
    *offset = rel;
    return true;
}

MemTxResult aspeed_tafsb_ahb_read(AspeedTAFSBState *s,
                                  uint64_t addr,
                                  void *buf, uint32_t len)
{
    MemTxResult res;

    trace_aspeed_tafsb_ahb_access(addr, len, false);

    if (!aspeed_tafsb_memory_mode(s)) {
        uint32_t offset;

        if (!aspeed_tafsb_flash_offset(s, addr, len, &offset)) {
            return MEMTX_ERROR;
        }
        return aspeed_tafsb_spi_cmd(s, TAFSB_CSET0, true, offset, NULL, 0,
                                    buf, len) ? MEMTX_OK : MEMTX_ERROR;
    }

    res = address_space_read(&s->ahb_as, addr, MEMTXATTRS_UNSPECIFIED,
                             buf, len);

    trace_aspeed_tafsb_ahb(addr, len, false, res);

    if (res != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: read of %" PRIu32 " bytes at AHB address 0x%"
                      PRIx64 " failed (%d); nothing is mapped at it\n",
                      __func__, len, addr, res);
    }

    return res;
}

MemTxResult aspeed_tafsb_program(AspeedTAFSBState *s,
                                 uint64_t addr,
                                 const void *buf, uint32_t len)
{
    const uint8_t *data = buf;
    uint32_t offset;

    trace_aspeed_tafsb_ahb_access(addr, len, true);

    if (!aspeed_tafsb_flash_offset(s, addr, len, &offset)) {
        return MEMTX_ERROR;
    }
    if (aspeed_tafsb_memory_mode(s)) {
        return address_space_write(&s->ahb_as, addr, MEMTXATTRS_UNSPECIFIED,
                                   buf, len);
    }

    /* A page program must not cross a page boundary */
    while (len) {
        uint32_t n = MIN(len, TAFSB_FLASH_PAGE_SIZE -
                              (offset % TAFSB_FLASH_PAGE_SIZE));

        if (!aspeed_tafsb_spi_update(s, TAFSB_CSET2, offset, data, n)) {
            return MEMTX_ERROR;
        }
        offset += n;
        data += n;
        len -= n;
    }
    return MEMTX_OK;
}

/* Memory mode erase: fill the block of @size containing @offset */
static MemTxResult aspeed_tafsb_mem_erase(AspeedTAFSBState *s,
                                          uint32_t offset, uint32_t size)
{
    uint64_t start = aspeed_tafsb_mem_base(s) +
                     QEMU_ALIGN_DOWN(offset, size);
    g_autofree uint8_t *fill = NULL;

    if (!s->tafsc) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: no global controller to say what an erase "
                      "fills with\n", __func__);
        return MEMTX_ERROR;
    }
    fill = g_malloc(size);
    memset(fill, aspeed_tafsc_erase_fill(s->tafsc, s->index), size);
    return address_space_write(&s->ahb_as, start, MEMTXATTRS_UNSPECIFIED,
                               fill, size);
}

MemTxResult aspeed_tafsb_erase(AspeedTAFSBState *s, uint64_t addr,
                               uint32_t size)
{
    uint32_t offset;
    hwaddr cset;

    if (aspeed_tafsb_memory_mode(s)) {
        if (!size || !is_power_of_2(size) ||
            !aspeed_tafsb_flash_offset(s, addr, 1, &offset)) {
            return MEMTX_ERROR;
        }
        trace_aspeed_tafsb_ahb_access(addr, size, true);
        return aspeed_tafsb_mem_erase(s, offset, size);
    }

    switch (size) {
    case 4 * KiB:
        cset = TAFSB_CSET4;
        break;
    case 32 * KiB:
        cset = TAFSB_CSET5;
        break;
    case 64 * KiB:
        cset = TAFSB_CSET6;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: no erase descriptor for %" PRIu32 " bytes\n",
                      __func__, size);
        return MEMTX_ERROR;
    }

    trace_aspeed_tafsb_ahb_access(addr, size, true);

    if (!aspeed_tafsb_flash_offset(s, addr, size, &offset) ||
        !aspeed_tafsb_spi_update(s, cset, offset, NULL, 0)) {
        return MEMTX_ERROR;
    }
    return MEMTX_OK;
}

static bool aspeed_tafsb_implemented(hwaddr offset)
{
    switch (offset) {
    case TAFSB_CFG:
    case TAFSB_WLOCK:
    case TAFSB_CBASE:
    case TAFSB_MBASE:
    case TAFSB_CSET0 ... TAFSB_CSET6:
    case TAFSB_MISC:
    case TAFSB_INFO:
    case TAFSB_DBG:
        return true;
    default:
        return false;
    }
}

static void aspeed_tafsb_trace_cset(hwaddr offset, uint32_t val)
{
    static const char * const modes[] = {
        [TAFSB_CSET_MODE_AUTO_RD] = "auto-read",
        [TAFSB_CSET_MODE_NORM_RD] = "normal-read",
        [TAFSB_CSET_MODE_NORM_WR] = "normal-write",
        [TAFSB_CSET_MODE_USER]    = "user",
    };
    uint32_t mode = val & TAFSB_CSET_MODE_MASK;
    uint32_t opcode = (val >> TAFSB_CSET_OPCODE_SHIFT) & TAFSB_CSET_OPCODE_MASK;
    uint32_t clkrate = (val >> TAFSB_CSET_CLKRATE_SHIFT) &
                       TAFSB_CSET_CLKRATE_MASK;

    trace_aspeed_tafsb_cset((int)((offset - TAFSB_CSET0) >> 2),
                            opcode, clkrate, modes[mode],
                            (val >> TAFSB_CSET_BEB_SHIFT) &
                            TAFSB_CSET_BEB_MASK,
                            !!(val & TAFSB_CSET_CE_STOP));
}

static uint64_t aspeed_tafsb_read(void *opaque, hwaddr offset,
                                  unsigned size)
{
    AspeedTAFSBState *s = ASPEED_TAFSB(opaque);
    uint32_t val;

    if (!aspeed_tafsb_implemented(offset)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: read of reserved offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }

    val = s->regs[TO_REG(offset)];
    trace_aspeed_tafsb_read(offset, val);

    return val;
}

/* Is this register write-locked by a bit software has already set? */
static bool aspeed_tafsb_locked(AspeedTAFSBState *s, hwaddr offset)
{
    uint32_t wlock = s->regs[TO_REG(TAFSB_WLOCK)];

    switch (offset) {
    case TAFSB_CBASE:
    case TAFSB_MBASE:
        return wlock & TAFSB_WLOCK_BASE;
    case TAFSB_CSET0 ... TAFSB_CSET6:
    case TAFSB_MISC:
        return wlock & TAFSB_WLOCK_CSET;
    default:
        return false;
    }
}

static void aspeed_tafsb_write(void *opaque, hwaddr offset,
                               uint64_t data, unsigned size)
{
    AspeedTAFSBState *s = ASPEED_TAFSB(opaque);
    uint32_t val = (uint32_t)data;

    trace_aspeed_tafsb_write(offset, val);

    if (!aspeed_tafsb_implemented(offset)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write 0x%08x to reserved offset 0x%"
                      HWADDR_PRIx "\n", __func__, val, offset);
        return;
    }

    if (aspeed_tafsb_locked(s, offset)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write 0x%08x to offset 0x%" HWADDR_PRIx
                      " which WLOCK 0x%x has locked until reset\n",
                      __func__, val, offset, s->regs[TO_REG(TAFSB_WLOCK)]);
        return;
    }

    switch (offset) {
    case TAFSB_WLOCK:
        /*
         * Write-once-to-one: a bit can be set but never cleared, so a lock
         * survives until the block is reset.
         */
        val = s->regs[TO_REG(offset)] | (val & TAFSB_WLOCK_MASK);
        break;

    case TAFSB_CSET0 ... TAFSB_CSET6:
        aspeed_tafsb_trace_cset(offset, val);
        break;

    case TAFSB_CFG:
        /*
         * CMD_EN chooses between driving a SPI controller and addressing
         * memory directly; the descriptors and the controller base only
         * apply to the former.
         */
        val &= TAFSB_CFG_WMASK;
        if (val & TAFSB_CFG_CMD_STOP) {
            qemu_log_mask(LOG_UNIMP, "%s: CMD_STOP is not modelled\n",
                          __func__);
        }
        trace_aspeed_tafsb_cfg(!!(val & TAFSB_CFG_CMD_EN),
                               !!(val & TAFSB_CFG_CMD_STOP));
        break;

    case TAFSB_DBG:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write 0x%08x to read-only DBG register\n",
                      __func__, val);
        return;

    case TAFSB_MISC:
        val &= TAFSB_MISC_WMASK;
        break;

    default:
        break;
    }

    s->regs[TO_REG(offset)] = val;
}

static const MemoryRegionOps aspeed_tafsb_ops = {
    .read = aspeed_tafsb_read,
    .write = aspeed_tafsb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void aspeed_tafsb_reset_hold(Object *obj, ResetType type)
{
    AspeedTAFSBState *s = ASPEED_TAFSB(obj);
    int i;

    memset(s->regs, 0, sizeof(s->regs));

    for (i = 0; i < ARRAY_SIZE(aspeed_tafsb_reset_values); i++) {
        s->regs[TO_REG(aspeed_tafsb_reset_values[i].offset)] =
            aspeed_tafsb_reset_values[i].value;
    }
}

static void aspeed_tafsb_realize(DeviceState *dev, Error **errp)
{
    AspeedTAFSBState *s = ASPEED_TAFSB(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (!s->ahb_mr) {
        error_setg(errp, TYPE_ASPEED_TAFSB
                   ": 'ahb' link is not set");
        return;
    }

    if (s->index >= ASPEED_TAFSC_NR_BRIDGES) {
        error_setg(errp, TYPE_ASPEED_TAFSB
                   ": 'index' is %u, but the global controller has %d bridges",
                   s->index, ASPEED_TAFSC_NR_BRIDGES);
        return;
    }

    /* The chip select picks one of the controller's control registers */
    if (s->cs >= ASPEED_SMC_CS_MAX) {
        error_setg(errp, TYPE_ASPEED_TAFSB
                   ": 'cs' is %u, but the controller has %d chip selects",
                   s->cs, ASPEED_SMC_CS_MAX);
        return;
    }

    memory_region_init_io(&s->mmio, OBJECT(s), &aspeed_tafsb_ops, s,
                          TYPE_ASPEED_TAFSB,
                          ASPEED_TAFSB_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);

    address_space_init(&s->ahb_as, s->ahb_mr, "aspeed-tafsb-ahb");
}

static const VMStateDescription vmstate_aspeed_tafsb = {
    .name = TYPE_ASPEED_TAFSB,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AspeedTAFSBState,
                             ASPEED_TAFSB_NR_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static const Property aspeed_tafsb_properties[] = {
    DEFINE_PROP_LINK("ahb", AspeedTAFSBState, ahb_mr,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_UINT32("cs", AspeedTAFSBState, cs, 0),
    DEFINE_PROP_LINK("tafsc", AspeedTAFSBState, tafsc,
                     TYPE_ASPEED_TAFSC, AspeedTAFSCState *),
    DEFINE_PROP_UINT32("index", AspeedTAFSBState, index, 0),
};

static void aspeed_tafsb_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_props(dc, aspeed_tafsb_properties);
    dc->realize = aspeed_tafsb_realize;
    dc->vmsd = &vmstate_aspeed_tafsb;
    dc->desc = "ASPEED TAFS/SAFS Bridge Controller";
    rc->phases.hold = aspeed_tafsb_reset_hold;
}

static const TypeInfo aspeed_tafsb_types[] = {
    {
        .name          = TYPE_ASPEED_TAFSB,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AspeedTAFSBState),
        .class_init    = aspeed_tafsb_class_init,
    },
};

DEFINE_TYPES(aspeed_tafsb_types)
