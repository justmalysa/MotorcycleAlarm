#include <zephyr.h>
#include "motorcycle_alarm.h"


void main(void)
{
    const char *user_number = "xxxxxxxxx";
    motorcycle_alarm_init(user_number);

    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
}