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

typedef struct ina260 ina260_t;

typedef HAL_StatusTypeDef ina260_status_t;

// determines the number of samples that are collected and averaged. each sample
// requires a given amount of ADC conversion time, and there are only a small
// number of discrete times that may be selected (see ina260_conversion_time_t).
typedef enum
{
  issSample1,    // = 0 (000b) -- default
  issSample4,    // = 1 (001b)
  issSample16,   // = 2 (010b)
  issSample64,   // = 3 (011b)
  issSample128,  // = 4 (100b)
  issSample256,  // = 5 (101b)
  issSample512,  // = 6 (110b)
  issSample1024, // = 7 (111b)
}
ina260_sample_size_t;

// sets the conversion time for the voltage and current measurement. these are
// the only intervals recognized by the INA260 hardware (per the datasheet).
// one complete lapse in the selected duration represents 1 sample. therefore
// the total time required for a single measurement is calculated as selected
// conversion time (ina260_conversion_time_t) multiplied by the selected number
// of samples (ina260_sample_size_t).
typedef enum
{
  ictConvert140us,   // = 0 (000b)
  ictConvert204us,   // = 1 (001b)
  ictConvert332us,   // = 2 (010b)
  ictConvert588us,   // = 3 (011b)
  ictConvert1p1ms,   // = 4 (100b) -- default (voltage, current)
  ictConvert2p116ms, // = 5 (101b)
  ictConvert4p156ms, // = 6 (110b)
  ictConvert8p244ms, // = 7 (111b)
}
ina260_conversion_time_t;

// determines how measurements should be performed and updated in internal
// data registers. you can read the contents at any time in both modes, but
// this will affect when they update.
typedef enum
{
  iomTriggered,  // = 0 (000b)
  iomContinuous, // = 1 (001b) -- default
}
ina260_operating_mode_t;

// specifies which measurements are performed for each conversion. you can
// perform current-only, voltage-only, or both current and voltage. note the
// bit patterns result in the following equalities:
//   iotShutdown == 0
//   iotPower    == ( iotVoltage | iotCurrent )
typedef enum
{
  iotShutdown, // = 0 (000b)
  iotCurrent,  // = 1 (001b)
  iotVoltage,  // = 2 (010b)
  iotPower,    // = 3 (011b) -- default
}
ina260_operating_type_t;

// format of CONFIGURATION register (00h)
typedef union
{
  uint16_t u16;
  __packed struct {
    ina260_operating_type_t   type : 2; //  0 -  1
    ina260_operating_mode_t   mode : 1; //  2
    ina260_conversion_time_t ctime : 3; //  3 -  5
    ina260_conversion_time_t vtime : 3; //  6 -  8
    ina260_sample_size_t     ssize : 3; //  9 - 11
    uint8_t                   resv : 3; // 12 - 14
    uint8_t                  reset : 1; // 15
  };
}
ina260_configuration_t;

// struct definition for primary device type
struct ina260
{
  I2C_HandleTypeDef *i2c_hal;
  uint8_t i2c_slave_addr;

  ina260_configuration_t config;
};

// ------------------------------------------------------- exported variables --

extern uint8_t const INA260_SLAVE_ADDRESS;

extern ina260_operating_type_t  const DEFAULT_OPERATING_TYPE;
extern ina260_operating_mode_t  const DEFAULT_OPERATING_MODE;
extern ina260_conversion_time_t const DEFAULT_CURRENT_CTIME;
extern ina260_conversion_time_t const DEFAULT_VOLTAGE_CTIME;
extern ina260_sample_size_t     const DEFAULT_SAMPLE_SIZE;

// ------------------------------------------------------- exported functions --

/**
  * @brief    initialize the INA260 with default configuration (continuous
  *           voltage and current measurements, each using 1 sample of a 1.1 ms
  *           ADC conversion)
  *
  * @param    i2c_hal - pointer to configured HAL I2C device handle
  *
  * @return   pointer to newly allocated ina260_t struct
  */
ina260_t *ina260_new(I2C_HandleTypeDef *i2c_hal, uint8_t i2c_slave_addr);

/**
  * @brief    check if we are communicating with the expected device over I2C
  *           correctly
  *
  * @note     this is NOT used to test if measurements are available; it only
  *           verifies we are able to communicate with the device. use the
  *           following routine to test if a measurement is ready for reading:
  *             ina260_status_t ina260_conversion_ready(ina260_t *)
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @return   status of the I2C read comparison with predefined device ID, may
  *           be one of the following values:
  *             HAL_OK      - value read over I2C matches expected device ID
  *             HAL_ERROR   - I2C read error or doesn't match expected value
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_READ_TIMEOUT_MS in ina260.c
  *
  */
ina260_status_t ina260_ready(ina260_t *pow);

/**
  * @brief    block until we are communicating with the expected device over
  *           I2C correctly
  *
  * @note     this is NOT used to test if measurements are available; it only
  *           verifies we are able to communicate with the device. use the
  *           following routine to test if a measurement is ready for reading:
  *             ina260_status_t ina260_conversion_ready(ina260_t *)
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    timeout - time (ms) to wait for device to become ready. to wait
  *                     indefinitely, use value HAL_MAX_DELAY. a value of 0
  *                     will attempt communication only once.
  *
  * @return   status of the I2C read comparison with predefined device ID, may
  *           be one of the following values:
  *             HAL_OK      - value read over I2C matches expected device ID
  *             HAL_ERROR   - I2C read error or doesn't match expected value
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout or failed to obtain the
  *                           expected device ID within given timeout period.
  */
ina260_status_t ina260_wait_until_ready(ina260_t *pow, uint32_t timeout);

/**
  * @brief    set all configuration parameters at once
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    operating_type - determines which conversions are performed
  *                            for each reading. may be one of the following:
  *             iotShutdown - put device in power-down state
  *             iotCurrent  - perform current readings only
  *             iotVoltage  - perform voltage readings only
  *             iotPower    - perform current and voltage readings
  *
  * @param    operating_mode - determines how conversions should be performed
  *                            for reading. may be one of the following:
  *             iomTriggered  - perform one-shot reading
  *             iomContinuous - continuously update readings
  *
  * @param    current_ctime - sets the conversion time for the current
  *                           measurement. total measure time is conversion
  *                           time multiplied by number of samples (specified
  *                           by sample_size). may be one of the following:
  *             ictConvert140us   - 140 us
  *             ictConvert204us   - 204 us
  *             ictConvert332us   - 332 us
  *             ictConvert588us   - 588 us
  *             ictConvert1p1ms   - 1.1 ms
  *             ictConvert2p116ms - 2.116 ms
  *             ictConvert4p156ms - 4.156 ms
  *             ictConvert8p244ms - 8.244 ms
  *
  * @param    voltage_ctime - sets the conversion time for the bus voltage
  *                           measurement. total measure time is conversion
  *                           time multiplied by number of samples (specified
  *                           by sample_size). may be one of the following:
  *             ictConvert140us   - 140 us
  *             ictConvert204us   - 204 us
  *             ictConvert332us   - 332 us
  *             ictConvert588us   - 588 us
  *             ictConvert1p1ms   - 1.1 ms
  *             ictConvert2p116ms - 2.116 ms
  *             ictConvert4p156ms - 4.156 ms
  *             ictConvert8p244ms - 8.244 ms
  *
  * @param    sample_size - determines the number of samples that are collected
                            and averaged for each measurement. may be one of the
                            following:
  *             issSample1    - 1 sample
  *             issSample4    - 4 samples
  *             issSample16   - 16 samples
  *             issSample64   - 64 samples
  *             issSample128  - 128 samples
  *             issSample256  - 256 samples
  *             issSample512  - 512 samples
  *             issSample1024 - 1024 samples
  *
  * @return   status of the I2C write operation with the new configuration, may
  *           be one of the following values:
  *             HAL_OK      - value successfully written to config register
  *             HAL_ERROR   - I2C write error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_WRITE_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_set_config(ina260_t *pow,
    ina260_operating_type_t  operating_type,
    ina260_operating_mode_t  operating_mode,
    ina260_conversion_time_t current_ctime,
    ina260_conversion_time_t voltage_ctime,
    ina260_sample_size_t     sample_size);

/**
  * @brief    set the operating type
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    operating_type - determines which conversions are performed for
  *                            each reading. may be one of the following:
  *             iotShutdown - put device in power-down state
  *             iotCurrent  - perform current readings only
  *             iotVoltage  - perform voltage readings only
  *             iotPower    - perform current and voltage readings
  *
  * @return   status of the I2C write operation with the new configuration, may
  *           be one of the following values:
  *             HAL_OK      - value successfully written to config register
  *             HAL_ERROR   - I2C write error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_WRITE_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_set_operating_type(ina260_t *pow,
    ina260_operating_type_t operating_type);

/**
  * @brief    set the operating mode
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    operating_mode - determines how conversions should be performed
  *                            for reading. may be one of the following:
  *             iomTriggered  - perform one-shot reading
  *             iomContinuous - continuously update readings
  *
  * @return   status of the I2C write operation with the new configuration, may
  *           be one of the following values:
  *             HAL_OK      - value successfully written to config register
  *             HAL_ERROR   - I2C write error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_WRITE_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_set_operating_mode(ina260_t *pow,
    ina260_operating_mode_t operating_mode);

/**
  * @brief    set the conversion time for both voltage and current readings
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    time - sets the conversion time for both voltage and current
  *                  measurements. total measure time is conversion time
  *                  multiplied by number of samples. may be one of the
  *                  following:
  *             ictConvert140us   - 140 us
  *             ictConvert204us   - 204 us
  *             ictConvert332us   - 332 us
  *             ictConvert588us   - 588 us
  *             ictConvert1p1ms   - 1.1 ms
  *             ictConvert2p116ms - 2.116 ms
  *             ictConvert4p156ms - 4.156 ms
  *             ictConvert8p244ms - 8.244 ms
  *
  * @return   status of the I2C write operation with the new configuration, may
  *           be one of the following values:
  *             HAL_OK      - value successfully written to config register
  *             HAL_ERROR   - I2C write error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_WRITE_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_set_conversion_time(ina260_t *pow,
    ina260_conversion_time_t time);

/**
  * @brief    set the conversion time for current readings
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    time - sets the conversion time for the current measurement. total
  *                  measure time is conversion time multiplied by number of
  *                  samples. may be one of the following:
  *             ictConvert140us   - 140 us
  *             ictConvert204us   - 204 us
  *             ictConvert332us   - 332 us
  *             ictConvert588us   - 588 us
  *             ictConvert1p1ms   - 1.1 ms
  *             ictConvert2p116ms - 2.116 ms
  *             ictConvert4p156ms - 4.156 ms
  *             ictConvert8p244ms - 8.244 ms
  *
  * @return   status of the I2C write operation with the new configuration, may
  *           be one of the following values:
  *             HAL_OK      - value successfully written to config register
  *             HAL_ERROR   - I2C write error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_WRITE_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_set_current_conversion_time(ina260_t *pow,
    ina260_conversion_time_t time);

/**
  * @brief    set the conversion time for voltage readings
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    time - sets the conversion time for the bus voltage measurement.
  *                  total measure time is conversion time multiplied by number
  *                  of samples. may be one of the following:
  *             ictConvert140us   - 140 us
  *             ictConvert204us   - 204 us
  *             ictConvert332us   - 332 us
  *             ictConvert588us   - 588 us
  *             ictConvert1p1ms   - 1.1 ms
  *             ictConvert2p116ms - 2.116 ms
  *             ictConvert4p156ms - 4.156 ms
  *             ictConvert8p244ms - 8.244 ms
  *
  * @return   status of the I2C write operation with the new configuration, may
  *           be one of the following values:
  *             HAL_OK      - value successfully written to config register
  *             HAL_ERROR   - I2C write error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_WRITE_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_set_voltage_conversion_time(ina260_t *pow,
    ina260_conversion_time_t time);

/**
  * @brief    set the sample size (averaging mode) for voltage and current
  *           readings
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    sample_size - determines the number of samples that are collected
  *                         and averaged for each measurement. may be one of the
  *                         following:
  *             issSample1    - 1 sample
  *             issSample4    - 4 samples
  *             issSample16   - 16 samples
  *             issSample64   - 64 samples
  *             issSample128  - 128 samples
  *             issSample256  - 256 samples
  *             issSample512  - 512 samples
  *             issSample1024 - 1024 samples
  *
  * @return   status of the I2C write operation with the new configuration, may
  *           be one of the following values:
  *             HAL_OK      - value successfully written to config register
  *             HAL_ERROR   - I2C write error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_WRITE_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_set_sample_size(ina260_t *pow,
    ina260_sample_size_t sample_size);

/**
  * @brief    software reset the INA260 device via configuration register,
  *           optionally resetting configuration to default or user settings.
  *           the configuration settings stored in the given ina260_t struct
  *           will reflect the actual resulting device configuration in either
  *           case.
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    init - initialize configuration using the current settings stored
  *                  in the ina260_t struct (if non-zero), or use the default
  *                  configuration (if zero).
  *
  * @return   status of the I2C write operation with the new configuration, may
  *           be one of the following values:
  *             HAL_OK      - value successfully written to config register
  *             HAL_ERROR   - I2C write error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_WRITE_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_reset(ina260_t *pow, uint8_t init);

/**
  * @brief    tests if a conversion is ready (following all conversions,
  *           averaging, and multiplications), to coordinate triggered
  *           measurements. once read, the flag is cleared and will not be set
  *           again until the next conversion completes.
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @return   status of the conversion ready flag as indicated in the
  *           mask/enable register. may be one of the following values:
  *             HAL_OK      - conversion ready
  *             HAL_ERROR   - I2C read error or invalid arguments
  *             HAL_BUSY    - conversion not ready
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_READ_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_conversion_ready(ina260_t *pow);

/**
  * @brief    starts or restarts a triggered "one-shot" conversion using the
  *           current device configuration. if the current operating mode is
  *           continuous, this routine does nothing and will not affect the
  *           device in any way.
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @return   status of the I2C write operation with the new configuration, may
  *           be one of the following values:
  *             HAL_OK      - value successfully written to config register
  *             HAL_ERROR   - I2C write error or invalid arguments
  *             HAL_BUSY    - I2C bus busy or operating mode is continuous
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_WRITE_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_conversion_start(ina260_t *pow);

/**
  * @brief    reads the bus voltage (mV) stored in the INA260 voltage register
  *           for both continuous and triggered conversions
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    voltage - pointer to output float variable to which a successful
  *                     reading of bus voltage (mV) will be written
  *
  * @return   status of the I2C read result. may be one of the following values:
  *             HAL_OK      - success
  *             HAL_ERROR   - I2C read error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_READ_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_get_voltage(ina260_t *pow, float *voltage);

/**
  * @brief    reads the current (mA) stored in the INA260 current register for
  *           both continuous and triggered conversions
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    current - pointer to output float variable to which a successful
  *                     reading of current (mA) will be written
  *
  * @return   status of the I2C read result. may be one of the following values:
  *             HAL_OK      - success
  *             HAL_ERROR   - I2C read error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_READ_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_get_current(ina260_t *pow, float *current);

/**
  * @brief    reads the power (mW) stored in the INA260 power register for both
  *           continuous and triggered conversions
  *
  * @param    pow - pointer to initialized ina260_t struct
  *
  * @param    power - pointer to output float variable to which a successful
  *                   reading of power (mW) will be written
  *
  * @return   status of the I2C read result. may be one of the following values:
  *             HAL_OK      - success
  *             HAL_ERROR   - I2C read error or invalid arguments
  *             HAL_BUSY    - I2C bus busy
  *             HAL_TIMEOUT - I2C communication timeout. see timeout constant
  *                           INA260_I2C_READ_TIMEOUT_MS in ina260.c
  */
ina260_status_t ina260_get_power(ina260_t *pow, float *power);

#ifdef __cplusplus
}
#endif

#endif /* INA260_H_ */
