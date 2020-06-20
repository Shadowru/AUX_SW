#include "Sonar.h"


Sonar::Sonar(int trig_pin, int echo_pin)
{
   _trig_pin = trig_pin;
   _echo_pin = echo_pin;   
}


int Sonar::readDistance()
{
   return -1;
}


boolean Sonar::isRealiable() // return the value of the median data sample
{
   return false;
}