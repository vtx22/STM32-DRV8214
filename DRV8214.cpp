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

void DRV8214::set_regulation_mode(DRV8214_IMODE mode)
{
    uint8_t reg = _read_reg8(DRV8214_REG::CONFIG3);

    reg &= (static_cast<uint8_t>(mode) << 6) + ~static_cast<uint8_t>(DRV8214_CONFIG3::IMODE);

    _write_reg_8(DRV8214_REG::CONFIG3, reg);
}

void DRV8214::set_stall_mode(DRV8214_SMODE mode)
{
    _set_bit(DRV8214_REG::CONFIG3, static_cast<uint8_t>(DRV8214_CONFIG3::SMODE), static_cast<bool>(mode));
}

void DRV8214::set_vref_internal(bool internal)
{
    _set_bit(DRV8214_REG::CONFIG3, static_cast<uint8_t>(DRV8214_CONFIG3::INT_VREF), internal);
}

/*
@param tblank Blanking time: 0=1.8us, 1=1us
*/
void DRV8214::set_current_sense_tblank(bool tblank)
{
    _set_bit(DRV8214_REG::CONFIG3, static_cast<uint8_t>(DRV8214_CONFIG3::TBLANK), tblank);
}

/*
@param deglitch Deglitch time: 0=2us, 1=1us
*/
void DRV8214::set_deglitch_time(bool deglitch)
{
    _set_bit(DRV8214_REG::CONFIG3, static_cast<uint8_t>(DRV8214_CONFIG3::TDEG), deglitch);
}

void DRV8214::set_ocp_mode(DRV8214_OCP_MODE mode)
{
    _set_bit(DRV8214_REG::CONFIG3, static_cast<uint8_t>(DRV8214_CONFIG3::OCP_MODE), static_cast<bool>(mode));
}

void DRV8214::set_tsd_mode(DRV8214_TSD_MODE mode)
{
    _set_bit(DRV8214_REG::CONFIG3, static_cast<uint8_t>(DRV8214_CONFIG3::TSD_MODE), static_cast<bool>(mode));
}

void DRV8214::set_rc_fault_reporting(DRV8214_RC_REP mode)
{
    uint8_t reg = _read_reg8(DRV8214_REG::CONFIG4);

    reg &= (static_cast<uint8_t>(mode) << 6) + ~static_cast<uint8_t>(DRV8214_CONFIG4::RC_REP);

    _write_reg_8(DRV8214_REG::CONFIG4, reg);
}

void DRV8214::set_stall_fault_reporting(bool on)
{
    _set_bit(DRV8214_REG::CONFIG4, static_cast<uint8_t>(DRV8214_CONFIG4::STALL_REP), on);
}

void DRV8214::set_cbc_fault_reporting(bool on)
{
    _set_bit(DRV8214_REG::CONFIG4, static_cast<uint8_t>(DRV8214_CONFIG4::CBC_REP), on);
}

void DRV8214::set_pmode(DRV8214_PMODE mode)
{
    _set_bit(DRV8214_REG::CONFIG4, static_cast<uint8_t>(DRV8214_CONFIG4::PMODE), static_cast<bool>(mode));
}

void DRV8214::set_control_interface(DRV8214_BRIDGE_CONTROL mode)
{
    _set_bit(DRV8214_REG::CONFIG4, static_cast<uint8_t>(DRV8214_CONFIG4::I2C_BC), static_cast<bool>(mode));
}

void DRV8214::set_i2c_en_in1(bool state)
{
    _set_bit(DRV8214_REG::CONFIG4, static_cast<uint8_t>(DRV8214_CONFIG4::I2C_EN_IN1), state);
}

void DRV8214::set_i2c_ph_in2(bool state)
{
    _set_bit(DRV8214_REG::CONFIG4, static_cast<uint8_t>(DRV8214_CONFIG4::I2C_PH_IN2), state);
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
