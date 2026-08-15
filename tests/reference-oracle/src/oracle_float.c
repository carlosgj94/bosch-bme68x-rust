/*
 * Test-only floating-point bridge to Bosch Sensortec BME68x SensorAPI v4.4.8.
 *
 * This is deliberately a separate translation unit from the fixed-point
 * bridge. With BME68X_DO_NOT_USE_FPU absent, the pinned, unmodified source
 * selects Bosch's BME68X_USE_FPU calculation path. It is never linked into
 * firmware or packaged with the published Rust crate.
 */
#include "../../reference/bosch/bme68x.c"

struct float_calibration_context
{
    const uint8_t *bytes;
};

static BME68X_INTF_RET_TYPE float_calibration_read(uint8_t reg_addr,
                                                    uint8_t *reg_data,
                                                    uint32_t length,
                                                    void *intf_ptr)
{
    const struct float_calibration_context *context =
        (const struct float_calibration_context *)intf_ptr;
    uint32_t offset;

    if (reg_addr == BME68X_REG_COEFF1 && length == BME68X_LEN_COEFF1)
    {
        offset = 0;
    }
    else if (reg_addr == BME68X_REG_COEFF2 && length == BME68X_LEN_COEFF2)
    {
        offset = BME68X_LEN_COEFF1;
    }
    else if (reg_addr == BME68X_REG_COEFF3 && length == BME68X_LEN_COEFF3)
    {
        offset = BME68X_LEN_COEFF1 + BME68X_LEN_COEFF2;
    }
    else
    {
        return -1;
    }

    for (uint32_t index = 0; index < length; index++)
    {
        reg_data[index] = context->bytes[offset + index];
    }

    return BME68X_INTF_RET_SUCCESS;
}

static BME68X_INTF_RET_TYPE float_unused_write(uint8_t reg_addr,
                                                const uint8_t *reg_data,
                                                uint32_t length,
                                                void *intf_ptr)
{
    (void)reg_addr;
    (void)reg_data;
    (void)length;
    (void)intf_ptr;
    return BME68X_INTF_RET_SUCCESS;
}

static void float_unused_delay(uint32_t period, void *intf_ptr)
{
    (void)period;
    (void)intf_ptr;
}

int8_t oracle_float_parse_calibration(const uint8_t bytes[42],
                                      struct bme68x_calib_data *calibration)
{
    struct float_calibration_context context = { bytes };
    struct bme68x_dev device = { 0 };
    device.intf = BME68X_I2C_INTF;
    device.intf_ptr = &context;
    device.read = float_calibration_read;
    device.write = float_unused_write;
    device.delay_us = float_unused_delay;

    const int8_t result = get_calib_data(&device);
    *calibration = device.calib;
    return result;
}

float oracle_float_temperature(uint32_t adc, struct bme68x_calib_data *calibration)
{
    struct bme68x_dev device = { 0 };
    device.calib = *calibration;
    const float result = calc_temperature(adc, &device);
    *calibration = device.calib;
    return result;
}

float oracle_float_pressure(uint32_t adc, const struct bme68x_calib_data *calibration)
{
    struct bme68x_dev device = { 0 };
    device.calib = *calibration;
    return calc_pressure(adc, &device);
}

float oracle_float_humidity(uint16_t adc, const struct bme68x_calib_data *calibration)
{
    struct bme68x_dev device = { 0 };
    device.calib = *calibration;
    return calc_humidity(adc, &device);
}

float oracle_float_gas_low(uint16_t adc,
                           uint8_t range,
                           const struct bme68x_calib_data *calibration)
{
    struct bme68x_dev device = { 0 };
    device.calib = *calibration;
    return calc_gas_resistance_low(adc, range, &device);
}

float oracle_float_gas_high(uint16_t adc, uint8_t range)
{
    return calc_gas_resistance_high(adc, range);
}

uint8_t oracle_float_heater_resistance(uint16_t temperature,
                                       int8_t ambient_temperature,
                                       const struct bme68x_calib_data *calibration)
{
    struct bme68x_dev device = { 0 };
    device.amb_temp = ambient_temperature;
    device.calib = *calibration;
    return calc_res_heat(temperature, &device);
}
