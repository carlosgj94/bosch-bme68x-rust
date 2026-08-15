/*
 * Test-only bridge to Bosch Sensortec BME68x SensorAPI v4.4.8.
 *
 * The pinned, unmodified implementation is included in this translation unit
 * so its static calculation functions can serve as a differential-test oracle.
 * It is never linked into firmware or the published bme68x crate.
 */
#include "../../reference/bosch/bme68x.c"

struct calibration_context
{
    const uint8_t *bytes;
};

static BME68X_INTF_RET_TYPE calibration_read(uint8_t reg_addr,
                                              uint8_t *reg_data,
                                              uint32_t length,
                                              void *intf_ptr)
{
    const struct calibration_context *context = (const struct calibration_context *)intf_ptr;
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

static BME68X_INTF_RET_TYPE unused_write(uint8_t reg_addr,
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

static void unused_delay(uint32_t period, void *intf_ptr)
{
    (void)period;
    (void)intf_ptr;
}

int8_t oracle_parse_calibration(const uint8_t bytes[42], struct bme68x_calib_data *calibration)
{
    struct calibration_context context = { bytes };
    struct bme68x_dev device = { 0 };
    device.intf = BME68X_I2C_INTF;
    device.intf_ptr = &context;
    device.read = calibration_read;
    device.write = unused_write;
    device.delay_us = unused_delay;

    const int8_t result = get_calib_data(&device);
    *calibration = device.calib;
    return result;
}

int16_t oracle_temperature(uint32_t adc, struct bme68x_calib_data *calibration)
{
    struct bme68x_dev device = { 0 };
    device.calib = *calibration;
    const int16_t result = calc_temperature(adc, &device);
    *calibration = device.calib;
    return result;
}

uint32_t oracle_pressure(uint32_t adc, const struct bme68x_calib_data *calibration)
{
    struct bme68x_dev device = { 0 };
    device.calib = *calibration;
    return calc_pressure(adc, &device);
}

uint32_t oracle_humidity(uint16_t adc, const struct bme68x_calib_data *calibration)
{
    struct bme68x_dev device = { 0 };
    device.calib = *calibration;
    return calc_humidity(adc, &device);
}

uint32_t oracle_gas_low(uint16_t adc, uint8_t range, const struct bme68x_calib_data *calibration)
{
    struct bme68x_dev device = { 0 };
    device.calib = *calibration;
    return calc_gas_resistance_low(adc, range, &device);
}

uint32_t oracle_gas_high(uint16_t adc, uint8_t range)
{
    return calc_gas_resistance_high(adc, range);
}

uint8_t oracle_heater_resistance(uint16_t temperature,
                                 int8_t ambient_temperature,
                                 const struct bme68x_calib_data *calibration)
{
    struct bme68x_dev device = { 0 };
    device.amb_temp = ambient_temperature;
    device.calib = *calibration;
    return calc_res_heat(temperature, &device);
}

uint8_t oracle_gas_wait(uint16_t duration)
{
    return calc_gas_wait(duration);
}

uint8_t oracle_shared_heater_duration(uint16_t duration)
{
    return calc_heatr_dur_shared(duration);
}

uint32_t oracle_measurement_duration(uint8_t mode, const struct bme68x_conf *configuration)
{
    struct bme68x_dev device = { 0 };
    device.read = calibration_read;
    device.write = unused_write;
    device.delay_us = unused_delay;
    struct bme68x_conf mutable_configuration = *configuration;
    return bme68x_get_meas_dur(mode, &mutable_configuration, &device);
}

