#ifndef MAIN_H
#define MAIN_H

#define BUZZER_PIN PIN_PA3

// TODO: Add charge voltage limits for the temperature.

// Below is the temperature points for the battery and what is allowed between each point. Values are in Celsius.

void ledOn();
void ledOff();
void mainMode();
void restart();
void chargerInterrupt();
void balancerInterrupt();
void sleepMode();
void wakeUpBalancer();

#endif