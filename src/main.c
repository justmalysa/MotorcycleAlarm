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

void main(void)
{
    const struct device *dev = get_bma220_device();

    if (dev == NULL)
    {
        return;
    }

    while (1)
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

        printf("x: %.1f, y: %.1f, z: %.1f (m/s^2)\n",
               sensor_value_to_double(&accel[0]),
               sensor_value_to_double(&accel[1]),
               sensor_value_to_double(&accel[2]));

        k_sleep(K_MSEC(1000));
    }
}