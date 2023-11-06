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
        void update(double&, double&);
        //boolean updatePeriod();
        //boolean updateOrientation();
        double getAngle();
        void setZeroCrossCallback(void(* callback)());
        void stopZeroCrossCallback();
        void setOnStopCallback(void(* callback)());
        void setOffset(double offset);
        void adjustAngle(double angle);
        double getOffset();
        void setAccelPos(double accelPos);
        double getAccelPos();
        double getPeriod();
        bool getIRData(int i);

    private:
        static std::bitset<IR_DATA_SIZE> IRData; // 500 bit array for incomming IR data
        static uint8_t pin;
        ADXL375 accel;
        double accelPos = 0.0301;
        double offset = 0;
        static double rotationPeriod; // milliseconds
        double angularVelocity; // radians per second
        uint64_t zeroCrossingTime = 0; // time stamp of last zero crossing
        uint64_t lastIROrientation = 0;
        double lastRotationPeriod = 0;

        kalmanFilter filter;
        esp_timer_handle_t update_timer;
        esp_timer_handle_t initTimer;
        static esp_timer_handle_t zeroHeadingTimer;
        static void (* zeroCrossCallback)();
        static void (* onStopCallback)();
        static void initCallback(void *args);
        static void zeroHeadingCallback(void *args);
        static void checkIRCallback(void *args);
        boolean getIRVelocity(double& rotationPeriod);
        boolean getAccelVelocity(double& rotationPeriod);
        boolean getIROrientation(uint64_t& IROrientation);
        double getAngle(uint64_t period);

};
#endif