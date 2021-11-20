/** @file
 * @brief SIM800L modem public API header file.
 *
 * Allows an application to control the SIM800L modem.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_SIM800L_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_SIM800L_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

int sim800l_init(void);
int sim800l_start_outgoing_call(const char *number);
void sim800l_abort_outgoing_call(void);
void sim800l_check_connection(void);
void sim800l_check_if_call_is_ongoing(void);
bool sim800l_ongoing_call(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_SIM800L_H_ */
