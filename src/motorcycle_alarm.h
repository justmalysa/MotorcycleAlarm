#ifndef MOTORCYCLE_ALARM_H
#define MOTORCYCLE_ALARM_H

void motorcycle_alarm_init(const char *user_number);
void bma220_measure(void);

#endif /* MOTORCYCLE_ALARM_H */