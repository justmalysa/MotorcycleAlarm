#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <stdio.h>

/*
 * Get a device structure from a devicetree node with compatible
 * "bosch,bma220". (If there are multiple, just pick one.)
 */
static const struct device *get_bma220_device(void)
{
    const struct device *dev = DEVICE_DT_GET_ANY(bosch_bma220);

    if (dev == NULL)
    {
        /* No such node, or the node does not have status "okay". */
        printk("\nError: no device found.\n");
        return NULL;
    }

    if (!device_is_ready(dev))
    {
        printk("\nError: Device \"%s\" is not ready; "
               "check the driver initialization logs for errors.\n",
               dev->name);
        return NULL;
    }

    printk("Found device \"%s\", getting sensor data\n", dev->name);
    return dev;
}

void bma220_any_motion_handler(const struct device *dev, struct sensor_trigger *trigger)
{
    struct sensor_value accel[3];

    if (sensor_sample_fetch(dev) < 0)
    {
        printk("Error when fetching the data\n");
    }

    if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel[0]) < 0)
    {
        printk("Channel X get error\n");
        return;
    }

    if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel[1]) < 0)
    {
        printk("Channel Y get error\n");
        return;
    }

    if (sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel[2]) < 0)
    {
        printk("Channel Z get error\n");
        return;
    }

    printf("Motion detected:\n"
           "x: %.1f, y: %.1f, z: %.1f (m/s^2)\n",
           sensor_value_to_double(&accel[0]),
           sensor_value_to_double(&accel[1]),
           sensor_value_to_double(&accel[2]));
}

void main(void)
{
    const struct device *dev = get_bma220_device();

    if (dev == NULL)
    {
        return;
    }

    struct sensor_value val =
    {
        .val1 = SENSOR_G / 1000000,
        .val2 = SENSOR_G % 1000000,
    };
    int rc = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &val);
    if (rc < 0)
    {
        printk("\nError: threshold could not be configured.\n");
        return;
    }

    val.val1 = 0x03;
    val.val2 = 0x00;
    rc = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &val);
    if (rc < 0)
    {
        printk("\nError: duration could not be configured.\n");
        return;
    }

    struct sensor_trigger trig;
    trig.type = SENSOR_TRIG_DELTA;
    trig.chan = SENSOR_CHAN_ACCEL_XYZ;
    rc = sensor_trigger_set(dev, &trig, bma220_any_motion_handler);
    if (rc < 0)
    {
        printk("\nError: trigger could not be enabled.\n");
        return;
    }

    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
}