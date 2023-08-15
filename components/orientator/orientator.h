#ifndef ORIENTATOR_H
#define ORIENTATOR_H

#include <Arduino.h>
#include <driver/timer.h>
#include <bitset>

class orientator {

    public:
        orientator();
        ~orientator();

        void setup(uint8_t pin);
        boolean updatePeriod();
        void updateOrientation();
        double getOrientation();
        void setOffset(double offset);
        double getOffset();
        uint32_t getPeriod();

    private:
        static std::bitset<500> IRData; // 500 bit array for incomming IR data
        static uint8_t pin;
        double offset = 0;
        uint16_t rotationPeriod = 0; // milliseconds
        uint64_t peakTimeStamp;

        esp_timer_handle_t update_timer;
        static void checkIRCallback(void *args);

};
#endif