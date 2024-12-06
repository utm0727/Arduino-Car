#include "SimpleTimer.h"

SimpleTimer::SimpleTimer(unsigned long interval_ms) {
    previousMillis = 0;
    interval = interval_ms;
}

void SimpleTimer::setInterval(unsigned long interval_ms) {
    interval = interval_ms;
}

bool SimpleTimer::check() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        return true;
    }
    return false;
}
