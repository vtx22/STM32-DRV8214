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

void DRV8214::set_current_regulation_mode(DRV8214_IMODE mode)
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

void DRV8214::set_soft_start(bool enable)
{
    _set_bit(DRV8214_REG::REG_CTRL0, static_cast<uint8_t>(DRV8214_REG_CTRL0::EN_SS), enable);
}

void DRV8214::set_regulation_control(DRV8214_REG_CTRL mode)
{
    uint8_t reg = _read_reg8(DRV8214_REG::REG_CTRL0);

    reg &= (static_cast<uint8_t>(mode) << 3) + ~static_cast<uint8_t>(DRV8214_REG_CTRL0::REG_CNTRL);

    _write_reg_8(DRV8214_REG::REG_CTRL0, reg);
}

void DRV8214::set_pwm_frequency(DRV8214_PWM_FREQ freq)
{
    _set_bit(DRV8214_REG::REG_CTRL0, static_cast<uint8_t>(DRV8214_REG_CTRL0::PWM_FREQ), static_cast<bool>(freq));
}

void DRV8214::set_w_scale(DRV8214_W_SCALE scale)
{
    uint8_t reg = _read_reg8(DRV8214_REG::REG_CTRL0);

    reg &= static_cast<uint8_t>(scale) + ~static_cast<uint8_t>(DRV8214_REG_CTRL0::W_SCALE);

    _write_reg_8(DRV8214_REG::REG_CTRL0, reg);
}

void DRV8214::set_target_voltage_speed(uint8_t target)
{
    _write_reg_8(DRV8214_REG::REG_CTRL1, target);
}

void DRV8214::set_output_filter_coutoff(DRV8214_OUT_FLT_CUTOFF frequency)
{
    uint8_t reg = _read_reg8(DRV8214_REG::REG_CTRL2);

    reg &= static_cast<uint8_t>(frequency) + ~static_cast<uint8_t>(DRV8214_REG_CTRL2::OUT_FLT);

    _write_reg_8(DRV8214_REG::REG_CTRL2, reg);
}

void DRV8214::set_external_duty_cycle(float percentage)
{
    if (percentage < 0.0f)
    {
        percentage = 0.0f;
    }
    if (percentage > 100.0f)
    {
        percentage = 100.0f;
    }

    uint8_t duty = static_cast<uint8_t>(round(percentage * 0b00111111 / 100.0f));

    uint8_t reg = _read_reg8(DRV8214_REG::REG_CTRL2) & 0b11000000;
    reg |= duty;

    _write_reg_8(DRV8214_REG::REG_CTRL2, reg);
}

void DRV8214::enable_ripple_counting(bool enable)
{
    _set_bit(DRV8214_REG::RC_CTRL0, static_cast<uint8_t>(DRV8214_RC_CTRL0::EN_RC), enable);
}

void DRV8214::enable_error_correction_module(bool enable)
{
    _set_bit(DRV8214_REG::RC_CTRL0, static_cast<uint8_t>(DRV8214_RC_CTRL0::DIS_EC), !enable);
}

void DRV8214::enable_rc_threshold_hiz(bool enable)
{
    _set_bit(DRV8214_REG::RC_CTRL0, static_cast<uint8_t>(DRV8214_RC_CTRL0::RC_HIZ), enable);
}

void DRV8214::set_filter_input_scaling(DRV8214_FLT_GAIN gain)
{
    uint8_t reg = _read_reg8(DRV8214_REG::RC_CTRL0);

    reg &= (static_cast<uint8_t>(gain) << 3) + ~static_cast<uint8_t>(DRV8214_RC_CTRL0::FLT_GAIN_SEL);

    _write_reg_8(DRV8214_REG::RC_CTRL0, reg);
}

void DRV8214::set_current_gain(DRV8214_CUR_GAIN gain)
{
    uint8_t reg = _read_reg8(DRV8214_REG::RC_CTRL0);

    reg &= static_cast<uint8_t>(gain) + ~static_cast<uint8_t>(DRV8214_RC_CTRL0::CS_GAIN_SEL);

    _write_reg_8(DRV8214_REG::RC_CTRL0, reg);
}

void DRV8214::set_ripple_count_threshold(uint16_t threshold)
{
    // Threshold is 10 bits
    if (threshold > 0x3FF)
    {
        threshold = 0x3FF;
    }

    _write_reg_8(DRV8214_REG::RC_CTRL1, threshold & 0xFF);

    uint8_t reg = _read_reg8(DRV8214_REG::RC_CTRL2);

    reg &= (threshold >> 8) + ~0b11;

    _write_reg_8(DRV8214_REG::RC_CTRL2, reg);
}

void DRV8214::set_inv_r_scale(DRV8214_INV_R_SCALE scale)
{
    uint8_t reg = _read_reg8(DRV8214_REG::RC_CTRL2);

    reg &= (static_cast<uint8_t>(scale) << 6) + ~static_cast<uint8_t>(DRV8214_RC_CTRL2::INV_R_SCALE);

    _write_reg_8(DRV8214_REG::RC_CTRL2, reg);
}

void DRV8214::set_kmc_scale(DRV8214_KMC_SCALE scale)
{
    uint8_t reg = _read_reg8(DRV8214_REG::RC_CTRL2);

    reg &= (static_cast<uint8_t>(scale) << 4) + ~static_cast<uint8_t>(DRV8214_RC_CTRL2::KMC_SCALE);

    _write_reg_8(DRV8214_REG::RC_CTRL2, reg);
}

void DRV8214::set_rc_thr_scale(DRV8214_RC_THR_SCALE scale)
{
    uint8_t reg = _read_reg8(DRV8214_REG::RC_CTRL2);

    reg &= (static_cast<uint8_t>(scale) << 2) + ~static_cast<uint8_t>(DRV8214_RC_CTRL2::RC_THR_SCALE);

    _write_reg_8(DRV8214_REG::RC_CTRL2, reg);
}

void DRV8214::set_inv_r(uint8_t inv_r)
{
    if (inv_r == 0)
    {
        return;
    }

    _write_reg_8(DRV8214_REG::RC_CTRL3, inv_r);
}

void DRV8214::set_kmc(uint8_t kmc)
{
    _write_reg_8(DRV8214_REG::RC_CTRL4, kmc);
}

void DRV8214::set_bandpass_filter_damping(uint8_t k)
{
    if (k > 0b1111)
    {
        k = 0b1111;
    }

    k = (k << 4);

    _write_reg_8(DRV8214_REG::RC_CTRL5, k);
}

void DRV8214::enable_error_correction_pulses(bool enable)
{
    _set_bit(DRV8214_REG::RC_CTRL6, static_cast<uint8_t>(DRV8214_RC_CTRL6::EC_PULSE_DIS), !enable);
}

void DRV8214::set_filter_t_mech(uint8_t t_mech)
{
    if (t_mech > 0b111)
    {
        t_mech = 0b111;
    }

    t_mech = (t_mech << 4);

    uint8_t reg = _read_reg8(DRV8214_REG::RC_CTRL6);

    reg &= t_mech + ~0b01111000;
}

void DRV8214::set_ec_false_percentage(DRV8214_EC_PER percentage)
{
    uint8_t reg = _read_reg8(DRV8214_REG::RC_CTRL6);

    reg &= (static_cast<uint8_t>(percentage) << 2) + ~0b1100;

    _write_reg_8(DRV8214_REG::RC_CTRL6, reg);
}

void DRV8214::set_ec_miss_percentage(DRV8214_EC_PER percentage)
{
    uint8_t reg = _read_reg8(DRV8214_REG::RC_CTRL6);

    reg &= static_cast<uint8_t>(percentage) + ~0b11;

    _write_reg_8(DRV8214_REG::RC_CTRL6, reg);
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
