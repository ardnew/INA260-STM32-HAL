/*
 * ina260.c
 *
 * STM32 HAL driver for INA260 voltage/current sensor.
 * datasheet: http://www.ti.com/lit/ds/symlink/ina260.pdf
 *
 *  Created on: Jan 23, 2020
 *      Author: andrew
 */

// ----------------------------------------------------------------- includes --

#include <stdlib.h>

#include "ina260.h"

// ---------------------------------------------------------- private defines --

#define __GPIO_PIN_CLR__ GPIO_PIN_RESET
#define __GPIO_PIN_SET__ GPIO_PIN_SET

// ----------------------------------------------------------- private macros --

// convert value at addr to little-endian (16-bit)
#define __LEu16(addr)                                      \
    ( ( (((uint16_t)(*(((uint8_t *)(addr)) + 1)))      ) | \
        (((uint16_t)(*(((uint8_t *)(addr)) + 0))) << 8U) ) )

// convert value at addr to little-endian (32-bit)
#define __LEu32(addr)                                       \
    ( ( (((uint32_t)(*(((uint8_t *)(addr)) + 3)))       ) | \
        (((uint32_t)(*(((uint8_t *)(addr)) + 2))) <<  8U) | \
        (((uint32_t)(*(((uint8_t *)(addr)) + 1))) << 16U) | \
        (((uint32_t)(*(((uint8_t *)(addr)) + 0))) << 24U) ) )

// swap two values `a` and `b` with a given type `t`
#define __SWAP(t, a, b) { t s; s = a; a = b; b = s; }

// resulting integer width given as t, e.g. __FROUND(uint16_t, -1.3)
#define __FROUND(t, x) ((x) < 0.0F ? -((t)((-(x)) + 0.5F)) : (t)((x) + 0.5F))
#define __FABS(v) ((v) < 0.0F ? -(v) : (v))

// 7-bit I2C addresses left-shifted by one bit. the HAL library takes care of
// setting the R/W bit, so no need to set it here
#define __I2C_SLAVE_READ_ADDR(addr)  ((addr) << 1)
#define __I2C_SLAVE_WRITE_ADDR(addr) ((addr) << 1)

// ------------------------------------------------------------ private types --

// format of the MASK/ENABLE register (06h)
/* TODO: add ALRT support */
typedef union
{
  uint16_t u16;
  __packed struct {
      uint8_t  alert_latch_enable : 1; //  0
      uint8_t      alert_polarity : 1; //  1
      uint8_t       math_overflow : 1; //  2
      uint8_t    conversion_ready : 1; //  3
      uint8_t alert_function_flag : 1; //  4
      uint8_t                resv : 5; //  5 -  9
      uint8_t    alert_conversion : 1; // 10
      uint8_t    alert_over_power : 1; // 11
      uint8_t alert_under_voltage : 1; // 12
      uint8_t  alert_over_voltage : 1; // 13
      uint8_t alert_under_current : 1; // 14
      uint8_t  alert_over_current : 1; // 15
  };
}
ina260_mask_enable_t;

// format of the DEVICE_ID register (FFh)
typedef union
{
  uint16_t u16;
  __packed struct {
    uint8_t revision :  4;
    uint16_t device  : 12;
  };
}
ina260_identification_t;

// ------------------------------------------------------- exported variables --

// default slave address for Adafruit INA260 breakout
uint8_t                  const INA260_SLAVE_ADDRESS   = 0x40;

// hardware default settings: continuous voltage and current measurements, each
// using 1 sample of a 1.1 ms ADC conversion
ina260_operating_type_t  const DEFAULT_OPERATING_TYPE = iotPower;
ina260_operating_mode_t  const DEFAULT_OPERATING_MODE = iomContinuous;
ina260_conversion_time_t const DEFAULT_CURRENT_CTIME  = ictConvert1p1ms;
ina260_conversion_time_t const DEFAULT_VOLTAGE_CTIME  = ictConvert1p1ms;
ina260_sample_size_t     const DEFAULT_SAMPLE_SIZE    = issSample1;

// -------------------------------------------------------- private variables --

// -- I2C configuration --
static uint32_t const INA260_I2C_READ_TIMEOUT_MS  = 2000;
static uint32_t const INA260_I2C_WRITE_TIMEOUT_MS = 2000;
// slave sub-address size, i.e. size of type used for MemAddress arguments
static uint16_t const INA260_I2C_MEM_ADDR_SIZE    = I2C_MEMADD_SIZE_8BIT;

// -- register addresses --
static uint8_t  const INA260_REG_CONFIG  = 0x00;
static uint8_t  const INA260_REG_CURRENT = 0x01;
static uint8_t  const INA260_REG_VOLTAGE = 0x02;
static uint8_t  const INA260_REG_POWER   = 0x03;
static uint8_t  const INA260_REG_MASK_EN = 0x06;
static uint8_t  const INA260_REG_ALRTLIM = 0x07;
static uint8_t  const INA260_REG_MFG_ID  = 0xFE;
static uint8_t  const INA260_REG_DEV_ID  = 0xFF;

// -- register LSB values --
static float   const INA260_LSB_CURRENT =  1.25F;
static float   const INA260_LSB_VOLTAGE =  1.25F;
static float   const INA260_LSB_POWER   = 10.00F;

// -- pre-defined device ID per datasheet */
static ina260_identification_t const INA260_DEVICE_ID = {
    .revision = 0x00,
    .device   = 0x227
};

// ---------------------------------------------- private function prototypes --

static ina260_status_t ina260_i2c_read(ina260_t *pow,
    uint8_t mem_addr, uint16_t *buff_dst, uint16_t buff_dst_sz);
static ina260_status_t ina260_i2c_write(ina260_t *pow,
    uint8_t mem_addr, uint16_t *buff_src, uint16_t buff_src_sz);
static ina260_status_t ina260_write_config(ina260_t *pow);
static ina260_status_t ina260_read_mask_enable(ina260_t *pow,
    ina260_mask_enable_t *mask_en);

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
ina260_t *ina260_new(I2C_HandleTypeDef *i2c_hal, uint8_t i2c_slave_addr)
{
  ina260_t *pow = NULL;

  if ((NULL != i2c_hal) && (i2c_slave_addr < 0x80)/* 7-bit */) {

    if (NULL != (pow = malloc(sizeof(ina260_t)))) {

      pow->i2c_hal        = i2c_hal;
      pow->i2c_slave_addr = i2c_slave_addr;

      if (HAL_OK != ina260_set_config(pow,
          DEFAULT_OPERATING_TYPE, DEFAULT_OPERATING_MODE,
          DEFAULT_CURRENT_CTIME, DEFAULT_VOLTAGE_CTIME, DEFAULT_SAMPLE_SIZE)) {
        free(pow);
        pow = NULL;
      }
    }
  }

  return pow;
}

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
ina260_status_t ina260_ready(ina260_t *pow)
{
  ina260_status_t status;
  ina260_identification_t id;

  if (NULL == pow)
    { return HAL_ERROR; }

  status = ina260_i2c_read(pow, INA260_REG_DEV_ID, &(id.u16), 2U);
  if (HAL_OK != status)
    { return status; }

  // INA260 returns MSB first
  id.u16 = __LEu16(&(id.u16));

  if (INA260_DEVICE_ID.u16 != id.u16)
    { return HAL_ERROR; }

  return HAL_OK;
}

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
ina260_status_t ina260_wait_until_ready(ina260_t *pow, uint32_t timeout)
{
  uint32_t start = HAL_GetTick();

  while (HAL_OK != ina260_ready(pow)) {
    if (HAL_MAX_DELAY != timeout) {
      if (((HAL_GetTick() - start) > timeout) || (0U == timeout)) {
        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

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
    ina260_sample_size_t     sample_size)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  pow->config.type  = operating_type;
  pow->config.mode  = operating_mode;
  pow->config.ctime = current_ctime;
  pow->config.vtime = voltage_ctime;
  pow->config.ssize = sample_size;

  return ina260_write_config(pow);
}

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
    ina260_operating_type_t operating_type)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  pow->config.type = operating_type;

  return ina260_write_config(pow);
}

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
    ina260_operating_mode_t operating_mode)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  pow->config.mode = operating_mode;

  return ina260_write_config(pow);
}

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
    ina260_conversion_time_t time)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  pow->config.ctime = time;
  pow->config.vtime = time;

  return ina260_write_config(pow);
}

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
    ina260_conversion_time_t time)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  pow->config.ctime = time;

  return ina260_write_config(pow);
}

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
    ina260_conversion_time_t time)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  pow->config.vtime = time;

  return ina260_write_config(pow);
}

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
    ina260_sample_size_t sample_size)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  pow->config.ssize = sample_size;

  return ina260_write_config(pow);
}

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
ina260_status_t ina260_reset(ina260_t *pow, uint8_t init)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  pow->config.reset = 1U;

  ina260_status_t status = ina260_write_config(pow);
  if (HAL_OK != status)
    { return status; }

  if (0U == init) {

    pow->config.type  = DEFAULT_OPERATING_TYPE;
    pow->config.mode  = DEFAULT_OPERATING_MODE;
    pow->config.ctime = DEFAULT_CURRENT_CTIME;
    pow->config.vtime = DEFAULT_VOLTAGE_CTIME;
    pow->config.ssize = DEFAULT_SAMPLE_SIZE;

    return HAL_OK;
  }
  else {
    return ina260_write_config(pow);
  }
}

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
ina260_status_t ina260_conversion_ready(ina260_t *pow)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  ina260_mask_enable_t mask_en;
  ina260_status_t status = ina260_read_mask_enable(pow, &mask_en);
  if (HAL_OK != status)
    { return status; }

  if (0U == mask_en.conversion_ready)
    { return HAL_BUSY; }
  else
    { return HAL_OK; }
}

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
ina260_status_t ina260_conversion_start(ina260_t *pow)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  if (iomContinuous == pow->config.mode)
    { return HAL_BUSY; }

  return ina260_write_config(pow);
}

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
ina260_status_t ina260_get_voltage(ina260_t *pow, float *voltage)
{
  ina260_status_t status;
  uint16_t v;

  if (NULL == pow)
    { return HAL_ERROR; }

  if (HAL_OK != (status = ina260_i2c_read(pow, INA260_REG_VOLTAGE, &v, 2U)))
    { return status; }

  v = __LEu16(&v);
  *voltage = (float)v * INA260_LSB_VOLTAGE;

  return HAL_OK;
}

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
ina260_status_t ina260_get_current(ina260_t *pow, float *current)
{
  ina260_status_t status;
  uint16_t u;
  int16_t c;

  if (NULL == pow)
    { return HAL_ERROR; }

  if (HAL_OK != (status = ina260_i2c_read(pow, INA260_REG_CURRENT, &u, 2U)))
    { return status; }

  c = (int16_t)__LEu16(&u);
  *current = (float)c * INA260_LSB_CURRENT;

  return HAL_OK;
}

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
ina260_status_t ina260_get_power(ina260_t *pow, float *power)
{
  ina260_status_t status;
  uint16_t p;

  if (NULL == pow)
    { return HAL_ERROR; }

  if (HAL_OK != (status = ina260_i2c_read(pow, INA260_REG_POWER, &p, 2U)))
    { return status; }

  p = __LEu16(&p);
  *power = (float)p * INA260_LSB_POWER;

  return HAL_OK;
}

// -------------------------------------------------------- private functions --

static ina260_status_t ina260_i2c_read(ina260_t *pow,
    uint8_t mem_addr, uint16_t *buff_dst, uint16_t buff_dst_sz)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  ina260_status_t status;

  while (HAL_BUSY == (status = HAL_I2C_Mem_Read(
      pow->i2c_hal,
      (uint16_t)__I2C_SLAVE_READ_ADDR(pow->i2c_slave_addr),
      (uint16_t)mem_addr,
      INA260_I2C_MEM_ADDR_SIZE,
      (uint8_t *)buff_dst,
      buff_dst_sz,
      INA260_I2C_READ_TIMEOUT_MS))) {

    // should not happen, unless during IRQ routine
    HAL_I2C_DeInit(pow->i2c_hal);
    HAL_I2C_Init(pow->i2c_hal);
  }

  return status;
}

static ina260_status_t ina260_i2c_write(ina260_t *pow,
    uint8_t mem_addr, uint16_t *buff_src, uint16_t buff_src_sz)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  ina260_status_t status;

  status = HAL_I2C_Mem_Write(
      pow->i2c_hal,
      (uint16_t)__I2C_SLAVE_WRITE_ADDR(pow->i2c_slave_addr),
      (uint16_t)mem_addr,
      INA260_I2C_MEM_ADDR_SIZE,
      (uint8_t *)buff_src,
      buff_src_sz,
      INA260_I2C_WRITE_TIMEOUT_MS);

  return status;
}

static ina260_status_t ina260_write_config(ina260_t *pow)
{
  if (NULL == pow)
    { return HAL_ERROR; }

  ina260_status_t status = ina260_wait_until_ready(
      pow, INA260_I2C_READ_TIMEOUT_MS);

  ina260_configuration_t conf;
  conf.u16 = __LEu16(&(pow->config.u16));

  if (HAL_OK == status) {
    status = ina260_i2c_write(pow, INA260_REG_CONFIG, &(conf.u16), 2U);
  }

  return status;
}

static ina260_status_t ina260_read_mask_enable(ina260_t *pow,
    ina260_mask_enable_t *mask_en)
{
  ina260_status_t status;
  ina260_mask_enable_t me;

  if (NULL == pow)
    { return HAL_ERROR; }

  status = ina260_i2c_read(pow, INA260_REG_MASK_EN, &(me.u16), 2U);
  if (HAL_OK != status)
    { return status; }

  mask_en->u16 = __LEu16(&(me.u16));

  return HAL_OK;
}
