/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bma220

#include <drivers/i2c.h>
#include <init.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "bma220.h"

LOG_MODULE_REGISTER(BMA220, CONFIG_SENSOR_LOG_LEVEL);

static int bma220_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct bma220_data *drv_data = dev->data;
    uint8_t buf[3];

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

    /*
     * since all accel data register addresses are consecutive,
     * a burst read can be used to read all the samples
     */
    if (i2c_burst_read(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_ACCEL_X, buf, sizeof(buf)) < 0)
    {
        LOG_DBG("Could not read accel axis data");
        return -EIO;
    }

    drv_data->x_sample = (((int8_t)buf[0]) >> BMA220_ACCEL_SHIFT);
    drv_data->y_sample = (((int8_t)buf[1]) >> BMA220_ACCEL_SHIFT);
    drv_data->z_sample = (((int8_t)buf[2]) >> BMA220_ACCEL_SHIFT);

    return 0;
}

static void bma220_channel_accel_convert(struct sensor_value *val, int64_t raw_val)
{
    /* accel_val = (sample * BMA220_FULL_RAGE) / (2^data_width * 10^6) */
    raw_val = (raw_val * BMA220_FULL_RANGE) / (1 << (BMA220_ACCEL_BITS));
    val->val1 = raw_val / 1000000;
    val->val2 = raw_val % 1000000;

    /* normalize val to make sure val->val2 is positive */
    if (val->val2 < 0)
    {
        val->val1 -= 1;
        val->val2 += 1000000;
    }
}

static int bma220_channel_get(const struct device *dev, enum sensor_channel chan,
                              struct sensor_value *val)
{
    struct bma220_data *drv_data = dev->data;

    /*
     * See datasheet "Sensor data" section for
     * more details on processing sample data.
     */
    if (chan == SENSOR_CHAN_ACCEL_X)
    {
        bma220_channel_accel_convert(val, drv_data->x_sample);
    }
    else if (chan == SENSOR_CHAN_ACCEL_Y)
    {
        bma220_channel_accel_convert(val, drv_data->y_sample);
    }
    else if (chan == SENSOR_CHAN_ACCEL_Z)
    {
        bma220_channel_accel_convert(val, drv_data->z_sample);
    }
    else if (chan == SENSOR_CHAN_ACCEL_XYZ)
    {
        bma220_channel_accel_convert(val, drv_data->x_sample);
        bma220_channel_accel_convert(val + 1, drv_data->y_sample);
        bma220_channel_accel_convert(val + 2, drv_data->z_sample);
    }
    else
    {
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api bma220_driver_api = {
#if CONFIG_BMA220_TRIGGER
    .attr_set = bma220_attr_set,
    .trigger_set = bma220_trigger_set,
#endif
    .sample_fetch = bma220_sample_fetch,
    .channel_get = bma220_channel_get,
};

int bma220_init(const struct device *dev)
{
    struct bma220_data *drv_data = dev->data;
    uint8_t id = 0U;

    drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
    if (drv_data->i2c == NULL)
    {
        LOG_DBG("Could not get pointer to %s device", DT_INST_BUS_LABEL(0));
        return -EINVAL;
    }

    /* read device ID */
    if (i2c_reg_read_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_CHIP_ID, &id) < 0)
    {
        LOG_DBG("Could not read chip id");
        return -EIO;
    }

    if (id != BMA220_CHIP_ID)
    {
        LOG_DBG("Unexpected chip id (%x)", id);
        return -EIO;
    }

    if (i2c_reg_write_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_BW, BMA220_BW) < 0)
    {
        LOG_DBG("Could not set data filter bandwidth");
        return -EIO;
    }

    /* set g-range */
    if (i2c_reg_write_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_RANGE, BMA220_RANGE) < 0)
    {
        LOG_DBG("Could not set data g-range");
        return -EIO;
    }

#ifdef CONFIG_BMA220_TRIGGER
    if (bma220_init_interrupt(dev) < 0)
    {
        LOG_DBG("Could not initialize interrupts");
        return -EIO;
    }
#endif

    return 0;
}

struct bma220_data bma220_driver;

DEVICE_DT_INST_DEFINE(0, bma220_init, NULL, &bma220_driver, NULL, POST_KERNEL,
                      CONFIG_SENSOR_INIT_PRIORITY, &bma220_driver_api);
