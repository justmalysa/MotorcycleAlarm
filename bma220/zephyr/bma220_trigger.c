/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bma220

#include <device.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>

#include "bma220.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(BMA220, CONFIG_SENSOR_LOG_LEVEL);

static inline void setup_int(const struct device *dev, bool enable)
{
    struct bma220_data *data = dev->data;

    gpio_pin_interrupt_configure(data->gpio, DT_INST_GPIO_PIN(0, int_gpios),
                                 (enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE));
}

int bma220_attr_set(const struct device *dev, enum sensor_channel chan,
                    enum sensor_attribute attr, const struct sensor_value *val)
{
    struct bma220_data *drv_data = dev->data;
    uint64_t slope_th;

    if (chan != SENSOR_CHAN_ACCEL_XYZ)
    {
        return -ENOTSUP;
    }

    if (attr == SENSOR_ATTR_SLOPE_TH)
    {
        /* slope_th = (val * 10^6 * 2^4) / (BMA220_FULL_RANGE / 2) */
        slope_th = (uint64_t)val->val1 * 1000000U + (uint64_t)val->val2;
        slope_th = (slope_th * (1 << 4)) / (BMA220_FULL_RANGE / 2);
        if (i2c_reg_update_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_INT_SLOPE,
            BMA220_SLOPE_TH_MASK, slope_th << BMA220_SLOPE_TH_SHIFT) < 0)
        {
            LOG_DBG("Could not set slope threshold");
            return -EIO;
        }
    }
    else if (attr == SENSOR_ATTR_SLOPE_DUR)
    {
        if (i2c_reg_update_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_INT_SLOPE,
            BMA220_SLOPE_DUR_MASK, val->val1 << BMA220_SLOPE_DUR_SHIFT) < 0)
        {
            LOG_DBG("Could not set slope duration");
            return -EIO;
        }
    }
    else
    {
        return -ENOTSUP;
    }

    return 0;
}

static void bma220_gpio_callback(const struct device *dev, struct gpio_callback *cb,
                                 uint32_t pins)
{
    struct bma220_data *drv_data = CONTAINER_OF(cb, struct bma220_data, gpio_cb);

    ARG_UNUSED(pins);

    setup_int(drv_data->dev, false);

#if defined(CONFIG_BMA220_TRIGGER_OWN_THREAD)
    k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_BMA220_TRIGGER_GLOBAL_THREAD)
    k_work_submit(&drv_data->work);
#endif
}

static void bma220_thread_cb(const struct device *dev)
{
    struct bma220_data *drv_data = dev->data;
    uint8_t status = 0U;
    int err = 0;

    err = i2c_reg_read_byte(drv_data->i2c, BMA220_I2C_ADDRESS,
                            BMA220_REG_INT_STATUS, &status);

    /* check for data ready */
    if (status & BMA220_BIT_DATA_INT_STATUS &&
        drv_data->data_ready_handler != NULL &&
        err == 0)
    {
        drv_data->data_ready_handler(dev, &drv_data->data_ready_trigger);
    }

    /* check for any motion */
    if (status & BMA220_BIT_SLOPE_INT_STATUS &&
        drv_data->any_motion_handler != NULL &&
        err == 0)
    {
        drv_data->any_motion_handler(dev, &drv_data->data_ready_trigger);

        /* clear latched interrupt */
        err = i2c_reg_update_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_INT_LATCH,
                                  BMA220_BIT_INT_LATCH_RESET, BMA220_BIT_INT_LATCH_RESET);

        if (err < 0)
        {
            LOG_DBG("Could not update clear the interrupt");
            return;
        }
    }

    setup_int(dev, true);
}

#ifdef CONFIG_BMA220_TRIGGER_OWN_THREAD
static void bma220_thread(struct bma220_data *drv_data)
{
    while (1)
    {
        k_sem_take(&drv_data->gpio_sem, K_FOREVER);
        bma220_thread_cb(drv_data->dev);
    }
}
#endif

#ifdef CONFIG_BMA220_TRIGGER_GLOBAL_THREAD
static void bma220_work_cb(struct k_work *work)
{
    struct bma220_data *drv_data = CONTAINER_OF(work, struct bma220_data, work);
    bma220_thread_cb(drv_data->dev);
}
#endif

int bma220_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                       sensor_trigger_handler_t handler)
{
    struct bma220_data *drv_data = dev->data;

    if (trig->type == SENSOR_TRIG_DATA_READY)
    {
        /* disable data ready interrupt while changing trigger params */
        if (i2c_reg_update_byte(drv_data->i2c, BMA220_I2C_ADDRESS,
                                BMA220_REG_INT_EN, BMA220_BIT_EN_DATA, 0) < 0)
        {
            LOG_DBG("Could not disable data ready interrupt");
            return -EIO;
        }

        drv_data->data_ready_handler = handler;
        if (handler == NULL)
        {
            return 0;
        }
        drv_data->data_ready_trigger = *trig;

        /* enable data ready interrupt */
        if (i2c_reg_update_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_INT_EN,
                                BMA220_BIT_EN_DATA, BMA220_BIT_EN_DATA) < 0)
        {
            LOG_DBG("Could not enable data ready interrupt");
            return -EIO;
        }
    }
    else if (trig->type == SENSOR_TRIG_DELTA)
    {
        /* disable any-motion interrupt while changing trigger params */
        if (i2c_reg_update_byte(drv_data->i2c, BMA220_I2C_ADDRESS,
                                BMA220_REG_INT_EN, BMA220_EN_SLOPE_XYZ, 0) < 0)
        {
            LOG_DBG("Could not disable data ready interrupt");
            return -EIO;
        }

        drv_data->any_motion_handler = handler;
        if (handler == NULL)
        {
            return 0;
        }
        drv_data->any_motion_trigger = *trig;

        /* enable any-motion interrupt */
        if (i2c_reg_update_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_INT_EN,
                                BMA220_EN_SLOPE_XYZ, BMA220_EN_SLOPE_XYZ) < 0)
        {
            LOG_DBG("Could not enable data ready interrupt");
            return -EIO;
        }
    }
    else
    {
        return -ENOTSUP;
    }

    return 0;
}

int bma220_init_interrupt(const struct device *dev)
{
    struct bma220_data *drv_data = dev->data;

    /* set latched interrupts */
    if (i2c_reg_write_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_INT_LATCH,
                           BMA220_BIT_INT_LATCH_RESET | BMA220_INT_MODE_LATCH) < 0)
    {
        LOG_DBG("Could not set latched interrupts");
        return -EIO;
    }

    /* setup data ready gpio interrupt */
    drv_data->gpio = device_get_binding(DT_INST_GPIO_LABEL(0, int_gpios));
    if (drv_data->gpio == NULL)
    {
        LOG_DBG("Cannot get pointer to %s device", DT_INST_GPIO_LABEL(0, int_gpios));
        return -EINVAL;
    }

    gpio_pin_configure(drv_data->gpio, DT_INST_GPIO_PIN(0, int_gpios),
                       DT_INST_GPIO_FLAGS(0, int_gpios) | GPIO_INPUT);

    gpio_init_callback(&drv_data->gpio_cb, bma220_gpio_callback,
                       BIT(DT_INST_GPIO_PIN(0, int_gpios)));

    if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0)
    {
        LOG_DBG("Could not set gpio callback");
        return -EIO;
    }

    if (i2c_reg_update_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_INT_EN,
                            BMA220_BIT_EN_DATA, 0) < 0)
    {
        LOG_DBG("Could not disable data ready interrupt");
        return -EIO;
    }

    /* disable any-motion interrupt */
    if (i2c_reg_update_byte(drv_data->i2c, BMA220_I2C_ADDRESS, BMA220_REG_INT_EN,
                            BMA220_EN_SLOPE_XYZ, 0) < 0)
    {
        LOG_DBG("Could not disable data ready interrupt");
        return -EIO;
    }

    drv_data->dev = dev;

#if defined(CONFIG_BMA220_TRIGGER_OWN_THREAD)
    k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);
    k_thread_create(&drv_data->thread, drv_data->thread_stack, CONFIG_BMA220_THREAD_STACK_SIZE,
                    (k_thread_entry_t)bma220_thread, drv_data, NULL, NULL,
                    K_PRIO_COOP(CONFIG_BMA220_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_BMA220_TRIGGER_GLOBAL_THREAD)
    drv_data->work.handler = bma220_work_cb;
#endif

    setup_int(dev, true);

    return 0;
}
