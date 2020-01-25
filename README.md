# INA260-STM32-HAL
##### INA260 voltage/current sensor driver for STM32 using HAL I²C

Datasheet: http://www.ti.com/lit/ds/symlink/ina260.pdf

---

#### Features include:
- [x] HAL-based I²C interface making it easy to support many STM32 devices
- [x] Modular configuration supporting STM32CubeMX-generated I²C interfaces
- [x] Normal, Fast, and High-Speed (FastModePlus) I²C data rates
- [x] Rigorously documented source code for all external API functions
- [x] Read bus voltage (mV), current (mA), and power (mW)
- [x] Supports both continuous and "one-shot" triggered conversions
- [x] Supports all conversion, sampling, and averaging rates supported by device
   - Independent configurations for bus voltage and current
- [ ] Over/under voltage/current and conversion-ready ALERT interrupts not yet implemented

---

#### Configuration
`TBD`

#### API
The following functions are the primary interface for interacting with device. See [INA260/ina260.c](INA260/ina260.c) for documentation on all parameters and return values.

```C
/**
  * initialize the INA260 with default configuration
  */
ina260_device_t *ina260_device_new(
    I2C_HandleTypeDef *i2c_hal,
    uint8_t i2c_slave_addr);

/**
  * check if we are communicating with the expected device over I2C
  * correctly
  */
ina260_status_t ina260_device_ready(ina260_device_t *dev);

/**
  * block until we are communicating with the expected device over
  * I2C correctly
  */
ina260_status_t ina260_wait_until_device_ready(ina260_device_t *dev,
    uint32_t timeout);

/**
  * set all configuration parameters at once
  */
ina260_status_t ina260_set_config(
    ina260_device_t                  *dev,
    ina260_operating_type_t           operating_type,
    ina260_operating_mode_t           operating_mode,
    ina260_current_conversion_time_t  current_ctime,
    ina260_voltage_conversion_time_t  voltage_ctime,
    ina260_averaging_mode_t           averaging_mode);

/**
  * set the operating type
  */
ina260_status_t ina260_set_operating_type(
    ina260_device_t *dev, ina260_operating_type_t operating_type);

/**
  * set the operating mode, or start triggered conversions.
  */
ina260_status_t ina260_set_operating_mode(
    ina260_device_t *dev, ina260_operating_mode_t operating_mode);

/**
  * set the conversion time for both voltage and current readings
  */
ina260_status_t ina260_set_conversion_time(
    ina260_device_t *dev, ina260_conversion_time_t ctime);

/**
  * set the conversion time for current readings
  */
ina260_status_t ina260_set_current_conversion_time(
    ina260_device_t *dev, ina260_current_conversion_time_t current_ctime);

/**
  * set the conversion time for voltage readings
  */
ina260_status_t ina260_set_voltage_conversion_time(
    ina260_device_t *dev, ina260_voltage_conversion_time_t voltage_ctime);

/**
  * set the averaging mode for voltage and current readings
  */
ina260_status_t ina260_set_averaging_mode(
    ina260_device_t *dev, ina260_averaging_mode_t averaging_mode);

/**
  * software reset the INA260 device via configuration register,
  * optionally resetting configuration to default or user settings.
  * the configuration settings stored in the given ina260_device_t
  * struct will reflect the actual resulting device configuration.
  */
ina260_status_t ina260_reset(ina260_device_t *dev, uint8_t init);

/**
  * tests if a conversion is ready (following all conversions,
  * averaging, and multiplications), to coordinate triggered
  * measurements. once read, the flag is cleared and will not be
  * true again until the next conversion completes.
  */
ina260_status_t ina260_conversion_ready(ina260_device_t *dev);

/**
  * reads the bus voltage (mV) stored in the INA260 voltage register
  * for both continuous and triggered conversions
  */
ina260_status_t ina260_get_voltage(ina260_device_t *dev, float *voltage);

/**
  * reads the current (mA) stored in the INA260 current register for
  * both continuous and triggered conversions
  */
ina260_status_t ina260_get_current(ina260_device_t *dev, float *current);

/**
  * reads the power (mW) stored in the INA260 power register for both
  * continuous and triggered conversions
  */
ina260_status_t ina260_get_power(ina260_device_t *dev, float *power);
```
