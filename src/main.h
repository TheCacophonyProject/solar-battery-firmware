#ifndef MAIN_H
#define MAIN_H

void ledOn();
void ledOff();
void mainMode();
void restart(uint8_t errorCode);
void chargerInterrupt();
void balancerInterrupt();
void sleepMode();
void wakeUpBalancer();
void setupTempAndHumidityCheck(bool balancerOptional);

#endif
