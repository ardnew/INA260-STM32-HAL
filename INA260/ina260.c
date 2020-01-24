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

#define __GPIO_PIN_CLR__     GPIO_PIN_RESET
#define __GPIO_PIN_SET__     GPIO_PIN_SET

#define __INA260_I2C_MEM_ADDR_SIZE__     I2C_MEMADD_SIZE_8BIT // subaddress sz
#define __INA260_I2C_READ_TIMEOUT_MS__   2000
#define __INA260_I2C_WRITE_TIMEOUT_MS__  2000

// ----------------------------------------------------------- private macros --

#define __SWAP(t, a, b) { t s; s = a; a = b; b = s; }

// resulting integer width given as t, e.g. __FROUND(uint16_t, -1.3)
#define __FROUND(t, x) ((x) < 0.0F ? -((t)((-(x)) + 0.5F)) : (t)((x) + 0.5F))
#define __FABS(v) ((v) < 0.0F ? -(v) : (v))

#define __I2C_SLAVE_READ_ADDR(addr)  ((addr) << 1)
#define __I2C_SLAVE_WRITE_ADDR(addr) ((addr) << 1)

// ------------------------------------------------------------ private types --

typedef union
{
  uint16_t u16;
  struct {
    uint8_t revision :  4;
    uint16_t device  : 12;
  };
}
ina260_identification_t;

// ------------------------------------------------------- exported variables --

ina260_operating_type_t          const DEFAULT_OPERATING_TYPE = iotPower;
ina260_operating_mode_t          const DEFAULT_OPERATING_MODE = iomContinuous;
ina260_current_conversion_time_t const DEFAULT_CURRENT_CTIME  = ictConvert1p1ms;
ina260_voltage_conversion_time_t const DEFAULT_VOLTAGE_CTIME  = ictConvert1p1ms;
ina260_averaging_mode_t          const DEFAULT_AVERAGING_MODE = iamAverage1;

// -------------------------------------------------------- private variables --

// -- I2C slave address --
uint8_t const INA260_SLAVE_ADDRESS = 0x40;

// -- register addresses --
static uint8_t const INA260_REG_CONFIG  = 0x00;
static uint8_t const INA260_REG_CURRENT = 0x01;
static uint8_t const INA260_REG_VOLTAGE = 0x02;
static uint8_t const INA260_REG_POWER   = 0x03;
static uint8_t const INA260_REG_MASK_EN = 0x06;
static uint8_t const INA260_REG_ALRTLIM = 0x07;
static uint8_t const INA260_REG_MFG_ID  = 0xFE;
static uint8_t const INA260_REG_DEV_ID  = 0xFF;

// -- register LSB values --
static float const INA260_LSB_CURRENT   =  1.25F;
static float const INA260_LSB_VOLTAGE   =  1.25F;
static float const INA260_LSB_POWER     = 10.00F;

static ina260_identification_t const INA260_DEVICE_ID = {
    .revision = 0x00,
    .device   = 0x227
};

// ---------------------------------------------- private function prototypes --

static ina260_status_t ina260_i2c_read(ina260_device_t *dev,
    uint8_t mem_addr, uint8_t *buff_dst, uint16_t buff_dst_sz);
static ina260_status_t ina260_i2c_write(ina260_device_t *dev,
    uint8_t mem_addr, uint8_t *buff_src, uint16_t buff_src_sz);

// ------------------------------------------------------- exported functions --

ina260_device_t *ina260_device_new(
    I2C_HandleTypeDef *i2c_hal,
    uint8_t i2c_slave_addr)
{
  ina260_device_t *dev;

  if ((NULL != i2c_hal) && (i2c_slave_addr < 0x80)/* 7-bit */) {

    if (NULL != (dev = malloc(sizeof(ina260_device_t)))) {

      dev->i2c_hal        = i2c_hal;
      dev->i2c_slave_addr = i2c_slave_addr;

      dev->config.type  = DEFAULT_OPERATING_TYPE;
      dev->config.mode  = DEFAULT_OPERATING_MODE;
      dev->config.ctime = DEFAULT_CURRENT_CTIME;
      dev->config.vtime = DEFAULT_VOLTAGE_CTIME;
      dev->config.avg   = DEFAULT_AVERAGING_MODE;
      dev->config.reset = 0;
    }
  }

  return dev;
}

ina260_status_t ina260_ready(ina260_device_t *dev)
{
  ina260_status_t status;
  ina260_identification_t id;

  if (NULL == dev)
    { return HAL_ERROR; }

  if (HAL_OK != (status = ina260_i2c_read(dev,
      INA260_REG_DEV_ID, (uint8_t *)&(id.u16), 2U)))
    { return status; }

  if (INA260_DEVICE_ID.u16 != id.u16)
    { return HAL_ERROR; }

  return HAL_OK;
}

void ina260_wait_until_ready(ina260_device_t *dev)
{
  while (HAL_OK != ina260_ready(dev))
    { continue ; }
}

// -------------------------------------------------------- private functions --

static ina260_status_t ina260_i2c_read(ina260_device_t *dev,
    uint8_t mem_addr, uint8_t *buff_dst, uint16_t buff_dst_sz)
{
  ina260_status_t status;

  while (HAL_BUSY == (status = HAL_I2C_Mem_Read(
      dev->i2c_hal,
      __I2C_SLAVE_READ_ADDR(dev->i2c_slave_addr),
      mem_addr,
      __INA260_I2C_MEM_ADDR_SIZE__,
      buff_dst,
      buff_dst_sz,
      __INA260_I2C_READ_TIMEOUT_MS__))) {

    // should not happen, unless during IRQ routine
    HAL_I2C_DeInit(dev->i2c_hal);
    HAL_I2C_Init(dev->i2c_hal);
  }

  return status;
}

static ina260_status_t ina260_i2c_write(ina260_device_t *dev,
    uint8_t mem_addr, uint8_t *buff_src, uint16_t buff_src_sz)
{
  ina260_status_t status;

  status = HAL_I2C_Mem_Write(
      dev->i2c_hal,
      __I2C_SLAVE_WRITE_ADDR(dev->i2c_slave_addr),
      mem_addr,
      __INA260_I2C_MEM_ADDR_SIZE__,
      buff_src,
      buff_src_sz,
      __INA260_I2C_WRITE_TIMEOUT_MS__);

  return status;
}
