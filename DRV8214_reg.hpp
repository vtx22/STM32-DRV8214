#pragma once

#include <cstdint>

enum class DRV8214_REG : uint8_t
{
    FAULT = 0x00,
    RC_STAT1 = 0x01,
    RC_STAT2 = 0x02,
    RC_STAT3 = 0x03,
    REG_STAT1 = 0x04,
    REG_STAT2 = 0x05,
    REG_STAT3 = 0x06,
    CONFIG0 = 0x09,
    CONFIG1 = 0x0A,
    CONFIG2 = 0x0B,
    CONFIG3 = 0x0C,
    CONFIG4 = 0x0D,
    REG_CTRL0 = 0x0E,
    REG_CTRL1 = 0x0F,
    REG_CTRL2 = 0x10,
    RC_CTRL0 = 0x11,
    RC_CTRL1 = 0x12,
    RC_CTRL2 = 0x13,
    RC_CTRL3 = 0x14,
    RC_CTRL4 = 0x15,
    RC_CTRL5 = 0x16,
    RC_CTRL6 = 0x17,
    RC_CTRL7 = 0x18,
    RC_CTRL8 = 0x19,
};

enum class DRV8214_FAULT : uint8_t
{
    FAULT = (1 << 7),
    STALL = (1 << 5),
    OCP = (1 << 4),
    OVP = (1 << 3),
    TSD = (1 << 2),
    NPOR = (1 << 1),
    CNT_DONE = (1 << 0),
};

enum class DRV8214_CONFIG0 : uint8_t
{
    EN_OUT = (1 << 7),
    EN_OVP = (1 << 6),
    EN_STALL = (1 << 5),
    VSNS_SEL = (1 << 4),
    VM_GAIN_SEL = (1 << 3),
    CLR_CNT = (1 << 2),
    CLR_FLT = (1 << 1),
    DUTY_CTRL = (1 << 0),
};

enum class DRV8214_CONFIG3 : uint8_t
{
    IMODE = (1 << 7) + (1 << 6),
    SMODE = (1 << 5),
    INT_VREF = (1 << 4),
    TBLANK = (1 << 3),
    TDEG = (1 << 2),
    OCP_MODE = (1 << 1),
    TSD_MODE = (1 << 0),
};

enum class DRV8214_CONFIG4 : uint8_t
{
    RC_REP = (1 << 7) + (1 << 6),
    STALL_REP = (1 << 5),
    CBC_REP = (1 << 4),
    PMODE = (1 << 3),
    I2C_BC = (1 << 2),
    I2C_EN_IN1 = (1 << 1),
    I2C_PH_IN2 = (1 << 0),
};

enum class DRV8214_REG_CTRL0 : uint8_t
{
    EN_SS = (1 << 5),
    REG_CNTRL = (1 << 4) + (1 << 3),
    PWM_FREQ = (1 << 2),
    W_SCALE = (1 << 1) + (1 << 0),
};

enum class DRV8214_REG_CTRL2 : uint8_t
{
    OUT_FLT = (1 << 7) + (1 << 6),
    EXT_DUTY = 0b00111111,
};

enum class DRV8214_RC_CTRL0 : uint8_t
{
    EN_RC = (1 << 7),
    DIS_EC = (1 << 6),
    RC_HIZ = (1 << 5),
    FLT_GAIN_SEL = (1 << 4) + (1 << 3),
    CS_GAIN_SEL = 0b111,
};

enum class DRV8214_RC_CTRL2 : uint8_t
{
    INV_R_SCALE = (1 << 7) + (1 << 6),
    KMC_SCALE = (1 << 5) + (1 << 4),
    RC_THR_SCALE = (1 << 3) + (1 << 2),
    RC_THR_9_8 = 0b11,
};

enum class DRV8214_RC_CTRL5 : uint8_t
{
    FLT_K = 0b1111000,
};

enum class DRV8214_RC_CTRL6 : uint8_t
{
    EC_PULSE_DIS = (1 << 7),
    T_MECH_FLT = (1 << 6) + (1 << 5) + (1 << 4),
    EC_FALSE_PER = (1 << 3) + (1 << 2),
    EC_MISS_PER = (1 << 1) + (1 << 0),
};

enum class DRV8214_RC_CTRL7 : uint8_t
{
    KP_DIV = 0b11100000,
    KP = 0b00011111,
};

enum class DRV8214_RC_CTRL8 : uint8_t
{
    KI_DIV = 0b11100000,
    KI = 0b00011111,
};

enum class DRV8214_IMODE : uint8_t
{
    FIXED_OFF_TIME = 0b00,
    CYCLE_BY_CYCLE = 0b01,
    SPEED_REGULATION = 0b10,
    VOLTAGE_REGULATION = 0b11,
};

enum class DRV8214_SMODE : uint8_t
{
    LATCHED_DISABLE = 0b00, // Disables OUTx pins when a stall occurs, STALL bit is set to 1
    INDICATION_ONLY = 0b01, // OUTx pins remain active when a stall occurs, STALL bit is set to 1
};