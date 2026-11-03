#ifndef CARCONTROL_H
#define CARCONTROL_H

#include <Arduino.h>

void Car_init();
void apply_Car_Mix(int pitch, int roll);
void stop_Car();

#endif