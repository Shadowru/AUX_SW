#ifndef Sonar_h

   #define Sonar_h

   #include "Arduino.h"

   class Sonar
   {
      public:
	Sonar(int trig_pin, int echo_pin);
	int readDistance();
	boolean isReliable();
      private:
	int _trig_pin;
	int _echo_pin;
   };

#endif
