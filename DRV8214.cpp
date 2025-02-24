#include "DRV8214.hpp"

DRV8214::DRV8214(I2C_HandleTypeDef *hi2c, uint8_t address) : _hi2c(hi2c), _address(address)
{
}

uint8_t DRV8214::get_fault()
{
    return _read_reg8(DRV8214_REG::FAULT);
}

uint8_t DRV8214::get_estimated_speed()
{
    return _read_reg8(DRV8214_REG::RC_STAT1);
}

uint16_t DRV8214::get_ripple_count()
{
    uint8_t low = _read_reg8(DRV8214_REG::RC_STAT2);
    uint8_t high = _read_reg8(DRV8214_REG::RC_STAT3);

    return (low << 8) + high;
}

float DRV8214::get_motor_voltage()
{
    return _read_reg8(DRV8214_REG::REG_STAT1) * 11.0f / 0xB0;
}

float DRV8214::get_motor_current()
{
    return _read_reg8(DRV8214_REG::REG_STAT2) * _max_current / 0xC0;
}

uint8_t DRV8214::_read_reg8(DRV8214_REG reg)
{
    return read_i2c_reg_8(_hi2c, _address, static_cast<uint8_t>(reg));
}
