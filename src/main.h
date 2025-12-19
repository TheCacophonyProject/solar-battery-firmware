#ifndef MAIN_H
#define MAIN_H

// TODO: Add charge voltage limits for the temperature.

// Below is the temperature points for the battery and what is allowed between each point.
// Values are in Celsius.

//    ^
//    |-- No charging allowed. Too cold.
//    |
#define TEMPERATURE_POINT_1 0
//    |
//    |-- Reduced charging/discharging current. 
//    |
#define TEMPERATURE_POINT_2 10
//    |
//    |-- Normal operating conditions.
//    |
#define TEMPERATURE_POINT_3 50
//    |
//    |-- Reduced charging/discharging current.
//    |
#define TEMPERATURE_POINT_5 60
//    |
//    |-- No charging allowed. Too hot.
//    V

void ledOn();
void ledOff();
void mainLogic();
void restart();
void chargerInterrupt();
void balancerInterrupt();

#endif