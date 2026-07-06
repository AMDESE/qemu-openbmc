/*
 * Simulator network bridge (SNB): wire format
 *
 * SNB connects QEMU device models to an external simulator over a pair of
 * stream sockets. Every packet starts with a fixed 16-byte head tag and a
 * little-endian payload size, followed by a 16-byte payload header and the
 * message itself. All multi-byte fields are little-endian; all structures
 * are packed.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SNB_SNB_PROTO_H
#define HW_SNB_SNB_PROTO_H

/* Packet framing */
#define SNB_HEAD_TAG_SIZE           16
#define SNB_PKT_HEADER_SIZE         (SNB_HEAD_TAG_SIZE + 4)
#define SNB_PAYLOAD_HDR_SIZE        16
#define SNB_MAX_DATA_SIZE           (1024 * 1024)
#define SNB_MIN_PAYLOAD_SIZE        SNB_PAYLOAD_HDR_SIZE
#define SNB_MAX_PAYLOAD_SIZE        (SNB_PAYLOAD_HDR_SIZE + SNB_MAX_DATA_SIZE)

extern const uint8_t snb_head_tag[SNB_HEAD_TAG_SIZE];

/*
 * Payload header. @id numbers a request; its response echoes @id and
 * @endpoint_id and sets SNB_PKT_FLAG_RESPONSE in @flags. @endpoint_id names
 * the device a message belongs to, so that one connection carries several
 * of them; it is 0 for control messages.
 */
typedef struct QEMU_PACKED SnbPayloadHdr {
    uint32_t msg_type;
    uint32_t id;
    uint32_t flags;
    uint32_t endpoint_id;
} SnbPayloadHdr;

#define SNB_PKT_FLAG_RESPONSE       0x00000001U

/*
 * Message type: bit 31 marks a response, bits 27:24 select the bus class
 * and bits 23:0 the operation within it.
 */
#define SNB_MSG_RSP_BIT             0x80000000U
#define SNB_MSG_BUS_SHIFT           24
#define SNB_MSG_BUS_MASK            0x0F000000U
#define SNB_MSG_OP_MASK             0x00FFFFFFU

#define SNB_MSG(bus, op)            (((uint32_t)(bus) << SNB_MSG_BUS_SHIFT) | \
                                     (op))
#define SNB_RSP(type)               ((type) | SNB_MSG_RSP_BIT)
#define SNB_MSG_BUS(type)           (((type) & SNB_MSG_BUS_MASK) >> \
                                     SNB_MSG_BUS_SHIFT)

/* Bus classes; 1 to 3 are reserved */
#define SNB_BUS_CTRL                0
#define SNB_BUS_I2C                 4
#define SNB_BUS_I3C                 5
#define SNB_BUS_GPIO                6
#define SNB_BUS_ESPI                7

/* Default endpoint of each bus class */
#define SNB_ENDPOINT_I2C            2
#define SNB_ENDPOINT_I3C            4
#define SNB_ENDPOINT_GPIO           6
#define SNB_ENDPOINT_ESPI           7

/* Control messages; operation 2 is reserved */
#define SNB_MSG_NOP                 SNB_MSG(SNB_BUS_CTRL, 0)
#define SNB_MSG_HELLO               SNB_MSG(SNB_BUS_CTRL, 1)
#define SNB_MSG_BUS_REGISTER        SNB_MSG(SNB_BUS_CTRL, 3)

/*
 * I2C: QEMU drives the bus, the peer holds the targets. @addr is the 7-bit
 * target address.
 */
#define SNB_MSG_I2C_WRITE           SNB_MSG(SNB_BUS_I2C, 0)
#define SNB_MSG_I2C_READ            SNB_MSG(SNB_BUS_I2C, 1)

/*
 * I3C: QEMU holds the controller, the peer the targets. A target is named
 * by its current dynamic address; the peer learns it from DA_ASSIGNED,
 * which identifies the target by its provisioned ID. Legacy I2C targets on
 * an I3C bus are named by the static address. Operation 4 is reserved.
 */
#define SNB_MSG_I3C_WRITE           SNB_MSG(SNB_BUS_I3C, 0)
#define SNB_MSG_I3C_READ            SNB_MSG(SNB_BUS_I3C, 1)
#define SNB_MSG_I3C_CCC             SNB_MSG(SNB_BUS_I3C, 2)
#define SNB_MSG_I3C_IBI             SNB_MSG(SNB_BUS_I3C, 3)
#define SNB_MSG_I3C_ENUM_I2C        SNB_MSG(SNB_BUS_I3C, 5)
#define SNB_MSG_I3C_I2C_WRITE       SNB_MSG(SNB_BUS_I3C, 6)
#define SNB_MSG_I3C_I2C_READ        SNB_MSG(SNB_BUS_I3C, 7)
#define SNB_MSG_I3C_ENUM_I3C        SNB_MSG(SNB_BUS_I3C, 8)
#define SNB_MSG_I3C_DA_ASSIGNED     SNB_MSG(SNB_BUS_I3C, 9)

/*
 * GPIO: a named signal changed level, in either direction. Neither side
 * waits for an answer; a response, if one comes, is ignored.
 */
#define SNB_MSG_GPIO_SET            SNB_MSG(SNB_BUS_GPIO, 0)

/*
 * eSPI: the peer holds the controller, QEMU the target. VW_NOTIFY, ALERT
 * and PC_NOTIFY go from QEMU to the peer and are not answered; VW_SET and
 * RESET go the other way and are not answered either. Operation 11 is
 * reserved.
 */
#define SNB_MSG_ESPI_FLASH_CYCLE    SNB_MSG(SNB_BUS_ESPI, 0)
#define SNB_MSG_ESPI_VW_SET         SNB_MSG(SNB_BUS_ESPI, 1)
#define SNB_MSG_ESPI_VW_NOTIFY      SNB_MSG(SNB_BUS_ESPI, 2)
#define SNB_MSG_ESPI_RESET          SNB_MSG(SNB_BUS_ESPI, 3)
#define SNB_MSG_ESPI_GET_STATUS     SNB_MSG(SNB_BUS_ESPI, 4)
#define SNB_MSG_ESPI_ALERT          SNB_MSG(SNB_BUS_ESPI, 5)
#define SNB_MSG_ESPI_GET_CONFIG     SNB_MSG(SNB_BUS_ESPI, 6)
#define SNB_MSG_ESPI_SET_CONFIG     SNB_MSG(SNB_BUS_ESPI, 7)
#define SNB_MSG_ESPI_FLASH_COMPLETION SNB_MSG(SNB_BUS_ESPI, 8)
#define SNB_MSG_ESPI_PC_CYCLE       SNB_MSG(SNB_BUS_ESPI, 9)
#define SNB_MSG_ESPI_PC_NOTIFY      SNB_MSG(SNB_BUS_ESPI, 10)

/*
 * Version handshake. Each side sends HELLO once on each socket before
 * anything else. A different major version, or a minor version below
 * SNB_VERSION_MINOR_MIN, which changed a message layout, is refused.
 */
#define SNB_VERSION_MAJOR           2
#define SNB_VERSION_MINOR           8
#define SNB_VERSION_MINOR_MIN       8

typedef struct QEMU_PACKED SnbMsgHello {
    uint16_t version_major;
    uint16_t version_minor;
    uint32_t caps;
} SnbMsgHello;

/* Status-only response, the first field of every response */
typedef struct QEMU_PACKED SnbMsgStatus {
    int32_t status;
} SnbMsgStatus;

typedef struct QEMU_PACKED SnbMsgNop {
    uint32_t data;
} SnbMsgNop;

/*
 * BUS_REGISTER binds @endpoint_id to a bus the peer knows by @name. It is
 * sent after every handshake, since the peer forgets the binding when a
 * session ends.
 */
#define SNB_BUS_NAME_MAX            32

typedef struct QEMU_PACKED SnbMsgBusRegister {
    char name[SNB_BUS_NAME_MAX];
    uint32_t bus_class;
    uint32_t endpoint_id;
} SnbMsgBusRegister;

/* I2C */
typedef struct QEMU_PACKED SnbMsgI2cWrite {
    uint8_t addr;
    uint8_t stop;
    uint32_t len;
    uint8_t data[];
} SnbMsgI2cWrite;

typedef struct QEMU_PACKED SnbMsgI2cRead {
    uint8_t addr;
    uint8_t stop;
    uint32_t len;
} SnbMsgI2cRead;

/* I3C */
typedef struct QEMU_PACKED SnbMsgI3cWrite {
    uint8_t da;
    uint32_t len;
    uint8_t data[];
} SnbMsgI3cWrite;

typedef struct QEMU_PACKED SnbMsgI3cRead {
    uint8_t da;
    uint32_t len;
} SnbMsgI3cRead;

/* A read and a CCC response carry the length the target produced */
typedef struct QEMU_PACKED SnbMsgI3cDataRsp {
    int32_t status;
    uint32_t len;
    uint8_t data[];
} SnbMsgI3cDataRsp;

/* @da is 0x7e for a broadcast CCC; a read CCC carries no data */
typedef struct QEMU_PACKED SnbMsgI3cCcc {
    uint8_t ccc;
    uint8_t rnw;
    uint8_t da;
    uint32_t len;
    uint8_t data[];
} SnbMsgI3cCcc;

/* An in-band interrupt from the target at @da; da 0 is a hot-join */
typedef struct QEMU_PACKED SnbMsgI3cIbi {
    uint8_t da;
    uint8_t has_mdb;
    uint8_t mdb;
} SnbMsgI3cIbi;

#define SNB_I3C_MAX_DEVS            16

/* ENUM_I2C and ENUM_I3C requests carry one reserved byte */
typedef struct QEMU_PACKED SnbMsgReserved {
    uint8_t reserved;
} SnbMsgReserved;

typedef struct QEMU_PACKED SnbMsgI3cI2cDev {
    uint8_t addr;
    uint8_t lvr;
} SnbMsgI3cI2cDev;

typedef struct QEMU_PACKED SnbMsgI3cEnumI2cRsp {
    int32_t status;
    uint8_t count;
    SnbMsgI3cI2cDev devs[SNB_I3C_MAX_DEVS];
} SnbMsgI3cEnumI2cRsp;

/* @static_addr is 0 for a target that only takes a dynamic address */
typedef struct QEMU_PACKED SnbMsgI3cDev {
    uint8_t static_addr;
    uint64_t pid;
    uint8_t bcr;
    uint8_t dcr;
} SnbMsgI3cDev;

typedef struct QEMU_PACKED SnbMsgI3cEnumI3cRsp {
    int32_t status;
    uint8_t count;
    SnbMsgI3cDev devs[SNB_I3C_MAX_DEVS];
} SnbMsgI3cEnumI3cRsp;

/* @da is 0 when the target lost its dynamic address */
typedef struct QEMU_PACKED SnbMsgI3cDaAssigned {
    uint64_t pid;
    uint8_t da;
} SnbMsgI3cDaAssigned;

typedef struct QEMU_PACKED SnbMsgI3cI2cWrite {
    uint8_t addr;
    uint32_t len;
    uint8_t data[];
} SnbMsgI3cI2cWrite;

typedef struct QEMU_PACKED SnbMsgI3cI2cRead {
    uint8_t addr;
    uint32_t len;
} SnbMsgI3cI2cRead;

/* GPIO: @level is the logical level, 1 asserted */
#define SNB_GPIO_NAME_MAX           64

typedef struct QEMU_PACKED SnbMsgGpioSet {
    uint8_t level;
    char name[SNB_GPIO_NAME_MAX];
} SnbMsgGpioSet;

/* eSPI */
#define SNB_ESPI_MAX_PLD            4096

/* Flash cycle types; only a write carries data */
#define SNB_ESPI_FLASH_READ         0x00
#define SNB_ESPI_FLASH_WRITE        0x01

/* @addr and @len carry the erase size code for an erase */
typedef struct QEMU_PACKED SnbMsgEspiFlashCycle {
    uint8_t cyc;
    uint8_t tag;
    uint32_t addr;
    uint32_t len;
    uint8_t data[];
} SnbMsgEspiFlashCycle;

/*
 * The completion of a flash or peripheral cycle, with the eSPI status word
 * after it. A nonzero status means the target did not take the cycle.
 */
typedef struct QEMU_PACKED SnbMsgEspiCompletion {
    int32_t status;
    uint8_t cyc;
    uint8_t tag;
    uint32_t espi_status;
    uint32_t len;
    uint8_t data[];
} SnbMsgEspiCompletion;

/*
 * Peripheral channel cycle types, as eSPI encodes them. I/O cycles have no
 * cycle type; SNB names them 0x20 and 0x21.
 */
#define SNB_ESPI_PC_MEMRD32         0x00
#define SNB_ESPI_PC_MEMWR32         0x01
#define SNB_ESPI_PC_MEMRD64         0x02
#define SNB_ESPI_PC_MEMWR64         0x03
#define SNB_ESPI_PC_SUC_CMPLT       0x06
#define SNB_ESPI_PC_SUC_CMPLT_D_MIDDLE 0x09
#define SNB_ESPI_PC_SUC_CMPLT_D_FIRST 0x0b
#define SNB_ESPI_PC_UNSUC_CMPLT_LAST 0x0c
#define SNB_ESPI_PC_SUC_CMPLT_D_LAST 0x0d
#define SNB_ESPI_PC_UNSUC_CMPLT_ONLY 0x0e
#define SNB_ESPI_PC_SUC_CMPLT_D_ONLY 0x0f
#define SNB_ESPI_PC_MSG             0x10
#define SNB_ESPI_PC_MSG_D           0x11
#define SNB_ESPI_PC_IORD            0x20
#define SNB_ESPI_PC_IOWR            0x21

/*
 * A peripheral cycle, in either direction. For a message, @addr carries the
 * four message-specific bytes, byte i in bits 8 * i + 7:8 * i, and
 * @msg_code the message code.
 */
typedef struct QEMU_PACKED SnbMsgEspiPcCycle {
    uint8_t cyc;
    uint8_t tag;
    uint64_t addr;
    uint32_t len;
    uint8_t msg_code;
    uint8_t data[];
} SnbMsgEspiPcCycle;

/* True for the cycle types whose data follows the cycle */
static inline bool snb_espi_pc_has_data(uint8_t cyc)
{
    switch (cyc) {
    case SNB_ESPI_PC_MEMWR32:
    case SNB_ESPI_PC_MEMWR64:
    case SNB_ESPI_PC_MSG_D:
    case SNB_ESPI_PC_SUC_CMPLT_D_MIDDLE:
    case SNB_ESPI_PC_SUC_CMPLT_D_FIRST:
    case SNB_ESPI_PC_SUC_CMPLT_D_LAST:
    case SNB_ESPI_PC_SUC_CMPLT_D_ONLY:
    case SNB_ESPI_PC_IOWR:
        return true;
    default:
        return false;
    }
}

/*
 * False for the cycle types with no length: a read carries the length it
 * asks for, a message or completion without data carries 0.
 */
static inline bool snb_espi_pc_has_len(uint8_t cyc)
{
    switch (cyc) {
    case SNB_ESPI_PC_MSG:
    case SNB_ESPI_PC_SUC_CMPLT:
    case SNB_ESPI_PC_UNSUC_CMPLT_LAST:
    case SNB_ESPI_PC_UNSUC_CMPLT_ONLY:
        return false;
    default:
        return true;
    }
}

/* A virtual wire index and its data byte in @value bits 7:0 */
typedef struct QEMU_PACKED SnbMsgEspiVw {
    uint8_t index;
    uint32_t value;
} SnbMsgEspiVw;

typedef struct QEMU_PACKED SnbMsgEspiReset {
    uint8_t asserted;
} SnbMsgEspiReset;

typedef struct QEMU_PACKED SnbMsgEspiStatusRsp {
    int32_t status;
    uint32_t espi_status;
} SnbMsgEspiStatusRsp;

typedef struct QEMU_PACKED SnbMsgEspiAlert {
    uint32_t espi_status;
} SnbMsgEspiAlert;

/* @addr is an eSPI configuration address; GET_CONFIG ignores @value */
typedef struct QEMU_PACKED SnbMsgEspiConfig {
    uint16_t addr;
    uint32_t value;
} SnbMsgEspiConfig;

typedef struct QEMU_PACKED SnbMsgEspiConfigRsp {
    int32_t status;
    uint32_t value;
    uint32_t espi_status;
} SnbMsgEspiConfigRsp;

QEMU_BUILD_BUG_ON(sizeof(SnbPayloadHdr) != SNB_PAYLOAD_HDR_SIZE);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgHello) != 8);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgBusRegister) != 40);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI2cWrite) != 6);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI2cRead) != 6);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cWrite) != 5);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cRead) != 5);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cDataRsp) != 8);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cCcc) != 7);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cIbi) != 3);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cEnumI2cRsp) != 37);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cDev) != 11);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cEnumI3cRsp) != 181);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cDaAssigned) != 9);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cI2cWrite) != 5);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgI3cI2cRead) != 5);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgGpioSet) != 65);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgEspiFlashCycle) != 10);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgEspiCompletion) != 14);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgEspiPcCycle) != 15);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgEspiVw) != 5);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgEspiStatusRsp) != 8);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgEspiConfig) != 6);
QEMU_BUILD_BUG_ON(sizeof(SnbMsgEspiConfigRsp) != 12);

#endif /* HW_SNB_SNB_PROTO_H */
