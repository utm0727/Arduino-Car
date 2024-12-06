#ifndef SIMPLETIMER_H
#define SIMPLETIMER_H

#include <Arduino.h>

class SimpleTimer {
  private:
    unsigned long previousMillis; 
    unsigned long interval;      

  public:
    SimpleTimer(unsigned long interval_ms);

    void setInterval(unsigned long interval_ms);

    bool check();
};

#endif
