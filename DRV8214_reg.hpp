#pragma once

#include <cstdint>

enum class DRV8214Register : uint8_t
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
    RC_CNTRL0 = 0x11,
    RC_CNTRL1 = 0x12,
    RC_CNTRL2 = 0x13,
    RC_CNTRL3 = 0x14,
    RC_CNTRL4 = 0x15,
    RC_CNTRL5 = 0x16,
    RC_CNTRL6 = 0x17,
    RC_CNTRL7 = 0x18,
    RC_CNTRL8 = 0x19,
};