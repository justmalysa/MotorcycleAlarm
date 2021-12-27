#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sim800l.h>
#include <drivers/uart.h>
#include <modem_receiver.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(motorcycle_alarm, CONFIG_LOG_DEFAULT_LEVEL);

#define HC05_SERIAL                  DT_ALIAS(hc05_serial)
#define DETECTED_MOTIONS_LIMIT       20
#define HC05_BUFFER_SIZE             128

struct k_timer timer_detection;
struct k_timer timer_call;
static uint8_t detected_motions_cnt;
static const char *p_user_number;

struct alarm_work_item
{
    struct k_work work;
    bool enable;
} alarm_work;

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
        LOG_ERR("\nError: no device found.\n");
        return NULL;
    }

    if (!device_is_ready(dev))
    {
        LOG_ERR("\nError: Device \"%s\" is not ready; "
               "check the driver initialization logs for errors.\n",
               dev->name);
        return NULL;
    }

    return dev;
}

static void any_motion_handler(const struct device *dev, struct sensor_trigger *trigger)
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
    LOG_INF("Detection count: %d", detected_motions_cnt);
}

static void set_alarm_state(bool enable)
{
    const struct device *dev = get_bma220_device();
    struct sensor_trigger trig;

    trig.type = SENSOR_TRIG_DELTA;
    trig.chan = SENSOR_CHAN_ACCEL_XYZ;

    int rc = sensor_trigger_set(dev, &trig, enable ? any_motion_handler : NULL);
    if (rc < 0)
    {
        LOG_ERR("\nError: trigger could not be set.\n");
        return;
    }
}

static void set_alarm_state_workqueue(struct k_work *item)
{
    struct alarm_work_item *work = CONTAINER_OF(item, struct alarm_work_item, work);
    set_alarm_state(work->enable);
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
        LOG_ERR("\nError: threshold could not be configured.\n");
        return;
    }

    val.val1 = 0x03;
    val.val2 = 0x00;
    rc = sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &val);
    if (rc < 0)
    {
        LOG_ERR("\nError: duration could not be configured.\n");
        return;
    }

    set_alarm_state(false);
}

static void hc05_rx_isr(const struct device *dev, void *user_data)
{
    static uint8_t hc05_recv_buf[HC05_BUFFER_SIZE];
    static uint8_t index = 0;

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev))
    {
        uint8_t c;
        if (uart_fifo_read(dev, &c, 1) > 0 )
        {
            if (index >= HC05_BUFFER_SIZE)
            {
                index = 0;
            }

            if (c == '\n')
            {
                hc05_recv_buf[index] = '\0';
            }
            else
            {
                hc05_recv_buf[index++] = c;
                continue;
            }

            if ( 0 == strcmp(hc05_recv_buf, "ON"))
            {
                index = 0;
                alarm_work.enable = true;
                k_work_submit(&alarm_work.work);
                LOG_INF("ON received");
            }
            else if ( 0 == strcmp(hc05_recv_buf, "OFF"))
            {
                index = 0;
                alarm_work.enable = false;
                k_work_submit(&alarm_work.work);
                LOG_INF("OFF received");
            }
            else
            {
                index = 0;
            }
        }
    }
}

static void hc05_init(void)
{
    const struct device *dev = device_get_binding(DT_LABEL(HC05_SERIAL));

    if (dev == NULL)
    {
        /* No such node, or the node does not have status "okay". */
        LOG_ERR("\nError: no device found.\n");
        return;
    }

    k_work_init(&alarm_work.work, set_alarm_state_workqueue);

    /* Setup serial */
    uart_irq_rx_disable(dev);
    uart_irq_tx_disable(dev);

    uart_irq_callback_user_data_set(dev, hc05_rx_isr, NULL);
    uart_irq_rx_enable(dev);
}

void motorcycle_alarm_init(const char *user_number)
{
    if (sim800l_init() < 0)
    {
        LOG_ERR("\nError: failed to init SIM800L modem.\n");
    }

    bma220_init();

    hc05_init();

    k_timer_init(&timer_detection, detection_timer_expiry, NULL);
    k_timer_init(&timer_call, call_timer_expiry, NULL);

    p_user_number = user_number;
}
