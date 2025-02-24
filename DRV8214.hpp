#pragma once

#include <cstdint>
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

    uint8_t get_fault();

    uint8_t get_estimated_speed();
    uint16_t get_ripple_count();

    float get_motor_voltage();
    float get_motor_current();

private:
    uint8_t _read_reg8(DRV8214_REG reg);
    void _write_reg_8(DRV8214_REG reg, uint8_t value);

    void _set_bit(DRV8214_REG reg, uint8_t mask, bool value);

    float _max_current = 4.f;

    I2C_HandleTypeDef *_hi2c;
    uint8_t _address;
};