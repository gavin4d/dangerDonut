#ifndef ORIENTATOR_H
#define ORIENTATOR_H

#include <Arduino.h>
#include <driver/timer.h>
#include <ADXL375.h>
#include <bitset>
#include "kalmanFilter.h"
#define IR_DATA_SIZE 500

class orientator {

    public:
        orientator();
        ~orientator();

        void setup(uint8_t pin, ADXL375 accel);
        boolean updatePeriod();
        boolean updateOrientation();
        double getOrientation();
        void setCallback(void(* callback)());
        void setOffset(double offset);
        double getOffset();
        void setAccelPos(double accelPos);
        double getAccelPos();
        double getPeriod();
        bool getIRData(int i);

    private:
        static std::bitset<IR_DATA_SIZE> IRData; // 500 bit array for incomming IR data
        static uint8_t pin;
        ADXL375 accel;
        double accelPos = 0.03;
        double offset = 0;
        static double rotationPeriod; // milliseconds
        uint64_t orientationTimeStamp = 0; // time stamp of last zero crossing
        uint64_t lastIROrientation = 0;
        double lastRotationPeriod = 0;

        kalmanFilter IRFilterO;
        kalmanFilter IRFilterP;
        kalmanFilter AccFilterO;
        kalmanFilter AccFilterP;
        esp_timer_handle_t update_timer;
        esp_timer_handle_t initTimer;
        static esp_timer_handle_t zeroHeadingTimer;
        static void (* userCallback)();
        static void initCallback(void *args);
        static void zeroHeadingCallback(void *args);
        static void checkIRCallback(void *args);
        boolean getIRPeriod(double* rotationPeriod);
        boolean getAccelPeriod(double* rotationPeriod);
        boolean getIROrientation(uint64_t* IROrientation);
        boolean getAccelOrientation(uint64_t* AccelOrientation);

};
#endif