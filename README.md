#### Freeze

This library is frozen. I'll still accept PRs for bug fixes, security issues (hah), and so on.

See [ardnew/libpvc](https://github.com/ardnew/libpvc) for an alternative, platform-agnostic library with much thinner API — including an Arduino-compatible adapter and example project for PlatformIO.

---

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

## External API

**See either [header](INA260/ina260.h) or [source](INA260/ina260.c) for complete documentation on all parameters and return values.**

##### Initialization
```C
/**
  * initialize the INA260 with default configuration (continuous voltage and
  * current measurements, each using 1 sample of a 1.1 ms ADC conversion)
  */
ina260_t *ina260_new(I2C_HandleTypeDef *i2c_hal, uint8_t i2c_slave_addr);
```

##### Configuration
```C
/**
  * set all configuration parameters at once
  */
ina260_status_t ina260_set_config(ina260_t *pow,
    ina260_operating_type_t           operating_type,
    ina260_operating_mode_t           operating_mode,
    ina260_current_conversion_time_t  current_ctime,
    ina260_voltage_conversion_time_t  voltage_ctime,
    ina260_averaging_mode_t           averaging_mode);

/**
  * set the operating type
  */
ina260_status_t ina260_set_operating_type(
    ina260_t *pow, ina260_operating_type_t operating_type);

/**
  * set the operating mode
  */
ina260_status_t ina260_set_operating_mode(
    ina260_t *pow, ina260_operating_mode_t operating_mode);

/**
  * set the conversion time for both voltage and current readings
  */
ina260_status_t ina260_set_conversion_time(
    ina260_t *pow, ina260_conversion_time_t ctime);

/**
  * set the conversion time for current readings
  */
ina260_status_t ina260_set_current_conversion_time(
    ina260_t *pow, ina260_current_conversion_time_t current_ctime);

/**
  * set the conversion time for voltage readings
  */
ina260_status_t ina260_set_voltage_conversion_time(
    ina260_t *pow, ina260_voltage_conversion_time_t voltage_ctime);

/**
  * set the averaging mode for voltage and current readings
  */
ina260_status_t ina260_set_averaging_mode(
    ina260_t *pow, ina260_averaging_mode_t averaging_mode);
```

##### Device status
```C
/**
  * check if we are communicating with the expected device over I2C
  * correctly.
  */
ina260_status_t ina260_ready(ina260_t *pow);

/**
  * block until we are communicating with the expected device over
  * I2C correctly
  */
ina260_status_t ina260_wait_until_ready(ina260_t *pow, uint32_t timeout);

/**
  * software reset the INA260 device via configuration register, optionally
  * resetting configuration to default or user settings. the configuration
  * settings stored in the given ina260_t struct will reflect the actual
  * resulting device configuration in either case.
  */
ina260_status_t ina260_reset(ina260_t *pow, uint8_t init);

/**
  * tests if a conversion is ready (following all conversions, averaging, and
  * multiplications) to coordinate triggered measurements. once read, the flag
  * is cleared and will not be set again until the next conversion completes.
  */
ina260_status_t ina260_conversion_ready(ina260_t *pow);

/**
  * starts or restarts a triggered "one-shot" conversion using the current
  * device configuration. if the current operating mode is continuous, this
  * routine does nothing and will not affect the device in any way.
  */
ina260_status_t ina260_conversion_start(ina260_t *pow);
```

##### Measurements
```C
/**
  * reads the bus voltage (mV) stored in the INA260 voltage register for both
  * continuous and triggered conversions
  */
ina260_status_t ina260_get_voltage(ina260_t *pow, float *voltage);

/**
  * reads the current (mA) stored in the INA260 current register for both
  * continuous and triggered conversions
  */
ina260_status_t ina260_get_current(ina260_t *pow, float *current);

/**
  * reads the power (mW) stored in the INA260 power register for both continuous
  * and triggered conversions
  */
ina260_status_t ina260_get_power(ina260_t *pow, float *power);
```

---

## Configuration
`TBD`
