/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT simcom_sim800l

#include <logging/log.h>
LOG_MODULE_REGISTER(modem_sim800l, CONFIG_MODEM_LOG_LEVEL);

#include <zephyr/types.h>
#include <stddef.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <device.h>
#include <init.h>

#include <drivers/uart.h>

#include <modem_receiver.h>
#include <sim800l.h>


#define DIGITS_PHONE_NUMBER     9
#define MDM_MAX_DATA_LENGTH     1500
#define INPUT_MAX_DATA_LENGTH   128

#define AT_CMD_AT               "AT"
#define AT_CMD_ATH              "ATH"
#define AT_CMD_CREG             "AT+CREG?"
#define AT_CMD_CPAS             "AT+CPAS"

#define MDM_UART_DEV            DEVICE_DT_GET(DT_INST_BUS(0))


static uint8_t mdm_recv_buf[MDM_MAX_DATA_LENGTH];
static char string_buf[INPUT_MAX_DATA_LENGTH];
static size_t string_index;
static volatile bool connection_established;
static volatile bool call_ongoing;

K_SEM_DEFINE(resp_ready, 0, 1);

/* RX thread structures */
K_THREAD_STACK_DEFINE(sim800l_rx_stack, CONFIG_MODEM_SIM800L_RX_STACK_SIZE);
struct k_thread sim800l_rx_thread;
#define RX_THREAD_PRIORITY K_PRIO_COOP(7)

static struct mdm_receiver_context ictx;

static void sim800l_input_parse(const char *buf)
{
    if ( 0 == strcmp(buf, "OK"))
    {
        k_sem_give(&resp_ready);
    }
    else if ( 0 == strcmp(buf, "+CREG: 0,1"))
    {
        connection_established = true;
    }
    else if ((0 == strcmp(buf, "+CPAS: 3")) || (0 == strcmp(buf, "+CPAS: 4")))
    {
        call_ongoing = true;
    }
    else if (0 == strcmp(buf, "+CPAS: 0"))
    {
        call_ongoing = false;
    }
}

static void sim800l_input_append(uint8_t *buf, size_t size_buf)
{
    static bool end_of_line_appeared;

    for (size_t i = 0; i < size_buf; i++)
    {
        if (buf[i] != '\r')
        {
            if (buf[i] != '\n')
            {
                end_of_line_appeared = false;
                string_buf[string_index++] = buf[i];
            }
        }
        else if (!end_of_line_appeared)
        {
            string_buf[string_index] = '\0';
            LOG_DBG("Modem response: %s", log_strdup(string_buf));
            sim800l_input_parse(string_buf);
            string_index = 0;
            end_of_line_appeared = true;
        }
    }
}

/* RX thread */
static void sim800l_rx(void)
{
    uint8_t uart_buffer[128];
    size_t bytes_read;
    int ret;

    while (true)
    {
        /* wait for incoming data */
        (void)k_sem_take(&ictx.rx_sem, K_FOREVER);

        ret = mdm_receiver_recv(&ictx, uart_buffer,
                                sizeof(uart_buffer), &bytes_read);
        if (ret < 0 || bytes_read == 0)
        {
            /* mdm_receiver buffer is empty */
            continue;
        }

        sim800l_input_append(uart_buffer, bytes_read);
    }
}

void sim800l_send_at_cmd(const char *at_cmd, size_t size_at_cmd)
{
    LOG_DBG("OUT: [%s]", log_strdup(at_cmd));
    mdm_receiver_send(&ictx, at_cmd, size_at_cmd);
    mdm_receiver_send(&ictx, "\r\n", 2);
    (void)k_sem_take(&resp_ready, K_FOREVER);
}

int sim800l_init(void)
{
    int ret = 0;

    ret = mdm_receiver_register(&ictx, MDM_UART_DEV,
                                mdm_recv_buf, sizeof(mdm_recv_buf));

    if (ret < 0)
    {
        LOG_ERR("Error registering modem receiver (%d)!", ret);
        return ret;
    }

    /* start RX thread */
    k_thread_name_set(k_thread_create(&sim800l_rx_thread, sim800l_rx_stack,
                                      K_THREAD_STACK_SIZEOF(sim800l_rx_stack),
                                      (k_thread_entry_t)sim800l_rx, NULL, NULL, NULL,
                                      RX_THREAD_PRIORITY, 0, K_NO_WAIT),
                      "sim800l rx");

    sim800l_send_at_cmd(AT_CMD_AT, sizeof(AT_CMD_AT) - 1);

    return ret;
}

int sim800l_start_outgoing_call(const char *number)
{
    char data[64];

    if (DIGITS_PHONE_NUMBER != strlen(number))
    {
        return -EINVAL;
    }

    for (size_t i = 0; i < DIGITS_PHONE_NUMBER; i++)
    {
        if (!isdigit(number[i]))
        {
            return -EINVAL;
        }
    }

    int bytes = snprintk(data, sizeof(data) - 1, "ATD+ +48%s;", number);
    sim800l_send_at_cmd(data, bytes);

    return 0;
}

void sim800l_abort_outgoing_call(void)
{
    sim800l_send_at_cmd(AT_CMD_ATH, sizeof(AT_CMD_ATH) - 1);
}

void sim800l_check_connection(void)
{
    while (!connection_established)
    {
        sim800l_send_at_cmd(AT_CMD_CREG, sizeof(AT_CMD_CREG) - 1);
        k_sleep(K_MSEC(1000));
    }
    connection_established = false;
}

void sim800l_check_if_call_is_ongoing(void)
{
    sim800l_send_at_cmd(AT_CMD_CPAS, sizeof(AT_CMD_CPAS) - 1);
}

bool sim800l_ongoing_call(void)
{
    return call_ongoing;
}