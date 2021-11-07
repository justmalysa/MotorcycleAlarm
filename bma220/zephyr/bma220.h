/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BMA220_H_
#define ZEPHYR_DRIVERS_SENSOR_BMA220_H_

#include <device.h>
#include <sys/util.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>

#define BMA220_I2C_ADDRESS            DT_INST_REG_ADDR(0)

#define BMA220_REG_CHIP_ID            0x00
#define BMA220_CHIP_ID                0xDD

#define BMA220_REG_BW                 0x20

#if CONFIG_BMA220_BW_1
    #define BMA220_BW                 0x05
#elif CONFIG_BMA220_BW_2
    #define BMA220_BW                 0x04
#elif CONFIG_BMA220_BW_3
    #define BMA220_BW                 0x03
#elif CONFIG_BMA220_BW_4
    #define BMA220_BW                 0x02
#elif CONFIG_BMA220_BW_5
    #define BMA220_BW                 0x01
#elif CONFIG_BMA220_BW_6
    #define BMA220_BW                 0x00
#endif

/*
 * BMA220_FULL_RANGE measured in micro-m/s^2 instead
 * of m/s^2 to avoid using struct sensor_value for it
 */
#define BMA220_REG_RANGE              0x22

#if CONFIG_BMA220_RANGE_2G
    #define BMA220_RANGE              0x00
    #define BMA220_FULL_RANGE         (4 * SENSOR_G)
#elif CONFIG_BMA220_RANGE_4G
    #define BMA220_RANGE              0x01
    #define BMA220_FULL_RANGE         (8 * SENSOR_G)
#elif CONFIG_BMA220_RANGE_8G
    #define BMA220_RANGE              0x02
    #define BMA220_FULL_RANGE         (16 * SENSOR_G)
#elif CONFIG_BMA220_RANGE_16G
    #define BMA220_RANGE              0x03
    #define BMA220_FULL_RANGE         (32 * SENSOR_G)
#endif

#define BMA220_REG_INT_STATUS         0x18
#define BMA220_BIT_SLOPE_INT_STATUS   BIT(0)
#define BMA220_BIT_DATA_INT_STATUS    BIT(1)

#define BMA220_REG_INT_EN             0x1A
#define BMA220_BIT_EN_DATA            BIT(7)
#define BMA220_BIT_EN_SLOPE_X         BIT(5)
#define BMA220_BIT_EN_SLOPE_Y         BIT(4)
#define BMA220_BIT_EN_SLOPE_Z         BIT(3)
#define BMA220_EN_SLOPE_XYZ           (BMA220_BIT_EN_SLOPE_X | \
                                       BMA220_BIT_EN_SLOPE_Y | \
                                       BMA220_BIT_EN_SLOPE_Z)

#define BMA220_REG_INT_LATCH          0x1C
#define BMA220_INT_MODE_LATCH         0x07
#define BMA220_BIT_INT_LATCH_RESET    BIT(7)

#define BMA220_REG_INT_SLOPE          0x12
#define BMA220_SLOPE_DUR_SHIFT        0
#define BMA220_SLOPE_DUR_MASK         (3 << BMA220_SLOPE_DUR_SHIFT)
#define BMA220_SLOPE_TH_SHIFT         2
#define BMA220_SLOPE_TH_MASK          (15 << BMA220_SLOPE_TH_SHIFT)

#define BMA220_REG_ACCEL_X            0x04
#define BMA220_REG_ACCEL_Y            0x06
#define BMA220_REG_ACCEL_Z            0x08
#define BMA220_ACCEL_SHIFT            2
#define BMA220_ACCEL_BITS             6

#define BMA220_THREAD_PRIORITY        10
#define BMA220_THREAD_STACKSIZE_UNIT  1024

struct bma220_data
{
    const struct device *i2c;
    int16_t x_sample;
    int16_t y_sample;
    int16_t z_sample;

#ifdef CONFIG_BMA220_TRIGGER
    const struct device *dev;
    const struct device *gpio;
    struct gpio_callback gpio_cb;

    struct sensor_trigger data_ready_trigger;
    sensor_trigger_handler_t data_ready_handler;

    struct sensor_trigger any_motion_trigger;
    sensor_trigger_handler_t any_motion_handler;

#if defined(CONFIG_BMA220_TRIGGER_OWN_THREAD)
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_BMA220_THREAD_STACK_SIZE);
    struct k_thread thread;
    struct k_sem gpio_sem;
#elif defined(CONFIG_BMA220_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif

#endif /* CONFIG_BMA220_TRIGGER */
};

#ifdef CONFIG_BMA220_TRIGGER
int bma220_trigger_set(const struct device *dev,
                       const struct sensor_trigger *trig,
                       sensor_trigger_handler_t handler);

int bma220_attr_set(const struct device *dev,
                    enum sensor_channel chan,
                    enum sensor_attribute attr,
                    const struct sensor_value *val);

int bma220_init_interrupt(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_BMA220_H_ */
