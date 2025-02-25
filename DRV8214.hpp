#pragma once

#include <cstdint>
#include <cmath>

#include "I2C.h"

#include "DRV8214_reg.hpp"

// Use the following flags for compiling the right library, e.g.: -D STM32F1
#if defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F2)
#include "stm32f2xx_hal.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#else
#error "Unsupported STM32 microcontroller. Make sure you build with -D STM32F1 for example!"
#endif

class DRV8214
{
public:
    DRV8214(I2C_HandleTypeDef *hi2c, uint8_t address);

    // === FAULT === //

    uint8_t get_fault();

    // === RC_STAT === //

    uint8_t get_estimated_speed();
    uint16_t get_ripple_count();

    // === REG_STAT === //

    float get_motor_voltage();
    float get_motor_current();
    float get_internal_duty_cycle();

    // === CONFIG0 === //

    void enable_outputs(bool enable);
    void enable_ovp(bool enable);
    void enable_stall_detection(bool enable);
    void enable_vs_filter(bool enable);
    void enable_voltage_gain(bool gain);
    void reset_ripple_count();
    void clear_faults();
    void enable_duty_control(bool enable);

    // === CONFIG1/2 === //

    void set_inrush_time_blanking(float seconds);

    // === CONFIG3 === //

    void set_current_regulation_mode(DRV8214_IMODE mode);
    void set_stall_mode(DRV8214_SMODE mode);
    void set_vref_internal(bool internal);
    void set_current_sense_tblank(bool tblank);
    void set_deglitch_time(bool deglitch);
    void set_ocp_mode(DRV8214_OCP_MODE mode);
    void set_tsd_mode(DRV8214_TSD_MODE mode);

    // === CONFIG4 === //

    void set_rc_fault_reporting(DRV8214_RC_REP mode);
    void set_stall_fault_reporting(bool on);
    void set_cbc_fault_reporting(bool on);
    void set_pmode(DRV8214_PMODE mode);
    void set_control_interface(DRV8214_BRIDGE_CONTROL mode);
    void set_i2c_en_in1(bool state);
    void set_i2c_ph_in2(bool state);

    // === REG_CTRL0 === //

    void set_soft_start(bool enable);
    void set_regulation_control(DRV8214_REG_CTRL mode);
    void set_pwm_frequency(DRV8214_PWM_FREQ freq);
    void set_w_scale(DRV8214_W_SCALE scale);

    // === REG_CTRL1 === //

    void set_target_voltage_speed(uint8_t target);

    // === REG_CTRL2 === //

    void set_output_filter_coutoff(DRV8214_OUT_FLT_CUTOFF frequency);
    void set_external_duty_cycle(float percentage);

    // === RC_CTRL0 === //

    void enable_ripple_counting(bool enable);
    void enable_error_correction_module(bool enable);
    void enable_rc_threshold_hiz(bool enable);
    void set_filter_input_scaling(DRV8214_FLT_GAIN gain);
    void set_current_gain(DRV8214_CUR_GAIN gain);

    // === RC_CTRL1 === //

    void set_ripple_count_threshold(uint16_t threshold);

private:
    uint8_t _read_reg8(DRV8214_REG reg);
    void _write_reg_8(DRV8214_REG reg, uint8_t value);

    void _set_bit(DRV8214_REG reg, uint8_t mask, bool value);

    float _max_current = 4.f;

    I2C_HandleTypeDef *_hi2c;
    uint8_t _address;
};