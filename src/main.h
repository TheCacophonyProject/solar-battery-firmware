#ifndef MAIN_H
#define MAIN_H

#define BUZZER_PIN PIN_PA3

void ledOn();
void ledOff();
void mainMode();
void restart();
void chargerInterrupt();
void balancerInterrupt();
void sleepMode();
void wakeUpBalancer();

#endif