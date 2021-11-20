#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sim800l.h>

#define DETECTED_MOTIONS_LIMIT    20

struct k_timer timer_detection;
struct k_timer timer_call;
static uint8_t detected_motions_cnt;
static const char *p_user_number;

static void detection_timer_expiry(struct k_timer *timer_id)
{
    detected_motions_cnt = 0;
}

static void call_timer_expiry(struct k_timer *timer_id)
{
    sim800l_abort_outgoing_call();
}

static void call_user(void)
{
    sim800l_check_connection();
    sim800l_start_outgoing_call(p_user_number);
    k_timer_start(&timer_call, K_SECONDS(20), K_FOREVER);
}

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

static void motorcycle_alarm_any_motion_handler(const struct device *dev, struct sensor_trigger *trigger)
{
    if (!detected_motions_cnt)
    {
        k_timer_start(&timer_detection, K_SECONDS(5), K_FOREVER);
    }

    if (detected_motions_cnt < DETECTED_MOTIONS_LIMIT)
    {
        detected_motions_cnt++;
    }
    else if (detected_motions_cnt == DETECTED_MOTIONS_LIMIT)
    {
        sim800l_check_if_call_is_ongoing();
        if (!sim800l_ongoing_call())
        {
            call_user();
        }
    }
    printk("CNT:%d \n", detected_motions_cnt);
}

static void bma220_init(void)
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
    rc = sensor_trigger_set(dev, &trig, motorcycle_alarm_any_motion_handler);
    if (rc < 0)
    {
        printk("\nError: trigger could not be enabled.\n");
        return;
    }
}

void motorcycle_alarm_init(const char *user_number)
{
    if (sim800l_init() < 0)
    {
        printk("\nError: failed to init SIM800L modem.\n");
    }

    bma220_init();

    k_timer_init(&timer_detection, detection_timer_expiry, NULL);
    k_timer_init(&timer_call, call_timer_expiry, NULL);

    p_user_number = user_number;
}
