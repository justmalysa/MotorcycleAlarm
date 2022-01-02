#ifndef MOTORCYCLE_ALARM_H
#define MOTORCYCLE_ALARM_H

/**
 * @brief Function for initializing motorcycle alarm.
 *
 * @param user_number Pointer to user phone number.
*/
void motorcycle_alarm_init(const char *user_number);

/**
 * @brief Function for measuring acceleration using BMA220 sensor.
 *
 * @note  Used only for selecting the appropriate sensitivity value of the device.
 */
void bma220_measure(void);

#endif /* MOTORCYCLE_ALARM_H */