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

float DRV8214::get_internal_duty_cycle()
{
    return _read_reg8(DRV8214_REG::REG_STAT3) * 100.f / 0b00111111;
}

void DRV8214::enable_outputs(bool enable)
{
    _set_bit(DRV8214_REG::CONFIG0, static_cast<uint8_t>(DRV8214_CONFIG0::EN_OUT), enable);
}

void DRV8214::enable_ovp(bool enable)
{
    _set_bit(DRV8214_REG::CONFIG0, static_cast<uint8_t>(DRV8214_CONFIG0::EN_OVP), enable);
}

void DRV8214::enable_stall_detection(bool enable)
{
    _set_bit(DRV8214_REG::CONFIG0, static_cast<uint8_t>(DRV8214_CONFIG0::EN_STALL), enable);
}

void DRV8214::enable_vs_filter(bool enable)
{
    _set_bit(DRV8214_REG::CONFIG0, static_cast<uint8_t>(DRV8214_CONFIG0::VSNS_SEL), enable);
}

void DRV8214::enable_voltage_gain(bool gain)
{
    _set_bit(DRV8214_REG::CONFIG0, static_cast<uint8_t>(DRV8214_CONFIG0::VM_GAIN_SEL), gain);
}

void DRV8214::reset_ripple_count()
{
    _set_bit(DRV8214_REG::CONFIG0, static_cast<uint8_t>(DRV8214_CONFIG0::CLR_CNT), true);
}

void DRV8214::clear_faults()
{
    _set_bit(DRV8214_REG::CONFIG0, static_cast<uint8_t>(DRV8214_CONFIG0::CLR_FLT), true);
}

void DRV8214::enable_duty_control(bool enable)
{
    _set_bit(DRV8214_REG::CONFIG0, static_cast<uint8_t>(DRV8214_CONFIG0::DUTY_CTRL), enable);
}

/*
@param seconds Inrush time blanking in seconds. Valid range is 0.005s to 6.7s.
*/
void DRV8214::set_inrush_time_blanking(float seconds)
{
    if (seconds < 0.005f)
    {
        seconds = 0.005f;
    }
    if (seconds > 6.7f)
    {
        seconds = 6.7f;
    }

    float reg_float = round((seconds - 0.005f) / (6.7f - 0.005f) * 0xFFFF);
    uint16_t reg_value = static_cast<uint16_t>(reg_float);

    _write_reg_8(DRV8214_REG::CONFIG1, reg_value >> 8);
    _write_reg_8(DRV8214_REG::CONFIG2, reg_value & 0xFF);
}

uint8_t DRV8214::_read_reg8(DRV8214_REG reg)
{
    return read_i2c_reg_8(_hi2c, _address, static_cast<uint8_t>(reg));
}

void DRV8214::_write_reg_8(DRV8214_REG reg, uint8_t value)
{
    write_i2c_reg_8(_hi2c, _address, static_cast<uint8_t>(reg), value);
}

void DRV8214::_set_bit(DRV8214_REG reg, uint8_t mask, bool value)
{
    uint8_t reg_value = _read_reg8(reg);

    if (value)
    {
        reg_value |= mask;
    }
    else
    {
        reg_value &= ~mask;
    }

    _write_reg_8(reg, reg_value);
}
