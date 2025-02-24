#include "DRV8214.hpp"

DRV8214::DRV8214(I2C_HandleTypeDef *hi2c, uint8_t address) : _hi2c(hi2c), _address(address)
{
}

uint8_t DRV8214::get_fault()
{
    return _read_reg8(DRV8214_REG::FAULT);
}

uint8_t DRV8214::_read_reg8(DRV8214_REG reg)
{
    return read_i2c_reg_8(_hi2c, _address, static_cast<uint8_t>(reg));
}