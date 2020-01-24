/*
 * ina260.h
 *
 * STM32 HAL driver for INA260 voltage/current sensor.
 * datasheet: http://www.ti.com/lit/ds/symlink/ina260.pdf
 *
 *  Created on: Jan 23, 2020
 *      Author: andrew
 */

#ifndef INA260_H_
#define INA260_H_

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------- includes --

#if defined (STM32L476xx)
#include "stm32l4xx_hal.h"
#elif defined(STM32F072xx)
#include "stm32f0xx_hal.h"
#elif defined(STM32F401xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32G431xx)
#include "stm32g4xx_hal.h"
#elif defined(STM32G031xx)
#include "stm32g0xx_hal.h"
#endif

// ------------------------------------------------------------------ defines --

/* nothing */

// ------------------------------------------------------------------- macros --

/* nothing */

// ----------------------------------------------------------- exported types --

typedef struct ina260_device ina260_device_t;

typedef HAL_StatusTypeDef ina260_status_t;

// Determines the number of samples that are collected and averaged.
typedef enum
{
  iamAverage1,    // = 0 (000b)
  iamAverage4,    // = 1 (001b)
  iamAverage16,   // = 2 (010b)
  iamAverage64,   // = 3 (011b)
  iamAverage128,  // = 4 (100b)
  iamAverage256,  // = 5 (101b)
  iamAverage512,  // = 6 (110b)
  iamAverage1024, // = 7 (111b)
}
ina260_averaging_mode_t;

// Sets the conversion time for the voltage/current measurement.
typedef enum
{
  ictConvert140us,   // = 0 (000b)
  ictConvert204us,   // = 1 (001b)
  ictConvert332us,   // = 2 (010b)
  ictConvert588us,   // = 3 (011b)
  ictConvert1p1ms,   // = 4 (100b)
  ictConvert2p116ms, // = 5 (101b)
  ictConvert4p156ms, // = 6 (110b)
  ictConvert8p244ms, // = 7 (111b)
}
ina260_conversion_time_t;

typedef
    ina260_conversion_time_t
    ina260_voltage_conversion_time_t;

typedef
    ina260_conversion_time_t
    ina260_current_conversion_time_t;

typedef enum
{
  iomTriggered,  // = 0 (000b)
  iomContinuous, // = 1 (001b)
}
ina260_operating_mode_t;

typedef enum
{
  iotShutdown, // = 0 (000b)
  iotCurrent,  // = 1 (001b)
  iotVoltage,  // = 2 (010b)
  iotPower,    // = 3 (011b)
}
ina260_operating_type_t;

typedef union
{
  uint16_t u16;
  struct {
    ina260_operating_type_t           type : 2;
    ina260_operating_mode_t           mode : 1;
    ina260_current_conversion_time_t ctime : 3;
    ina260_voltage_conversion_time_t vtime : 3;
    ina260_averaging_mode_t            avg : 3;
    uint8_t                                : 3;
    uint8_t                          reset : 1;
  };
}
ina260_configuration_t;

struct ina260_device
{
  I2C_HandleTypeDef *i2c_hal;
  uint8_t i2c_slave_addr;

  ina260_configuration_t config;
};

// ------------------------------------------------------- exported variables --

extern uint8_t const INA260_SLAVE_ADDRESS;

extern ina260_operating_type_t          const DEFAULT_OPERATING_TYPE;
extern ina260_operating_mode_t          const DEFAULT_OPERATING_MODE;
extern ina260_current_conversion_time_t const DEFAULT_CURRENT_CTIME;
extern ina260_voltage_conversion_time_t const DEFAULT_VOLTAGE_CTIME;
extern ina260_averaging_mode_t          const DEFAULT_AVERAGING_MODE;

// ------------------------------------------------------- exported functions --

ina260_device_t *ina260_device_new(
    I2C_HandleTypeDef *i2c_hal,
    uint8_t i2c_slave_addr);

ina260_status_t ina260_ready(ina260_device_t *dev);
void ina260_wait_until_ready(ina260_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* INA260_H_ */
