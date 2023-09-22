#ifndef ORIENTATOR_H
#define ORIENTATOR_H

#include <Arduino.h>
#include <driver/timer.h>
#include <ADXL375.h>
#include <bitset>
#define IR_DATA_SIZE 500

class orientator {

    public:
        orientator();
        ~orientator();

        void setup(uint8_t pin, ADXL375 accel);
        boolean updatePeriod();
        boolean updateOrientation();
        double getOrientation();
        void setOffset(double offset);
        double getOffset();
        void setAccelPos(double accelPos);
        double getAccelPos();
        uint32_t getPeriod();
        uint64_t orientationTimeStamp;
        boolean useIR, useAccel;

    private:
        static std::bitset<IR_DATA_SIZE> IRData; // 500 bit array for incomming IR data
        static uint8_t pin;
        ADXL375 accel;
        double accelPos = 0.03;
        double offset = 0;
        double rotationPeriod = 0; // milliseconds

        esp_timer_handle_t update_timer;
        static void checkIRCallback(void *args);
        boolean getIRPeriod(double* rotationPeriod);
        boolean getAccelPeriod(double* rotationPeriod);
        boolean getIROrientation(uint64_t* IROrientation);
        boolean getAccelOrientation(uint64_t* AccelOrientation);

};
#endif