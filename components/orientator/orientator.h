#ifndef ORIENTATOR_H
#define ORIENTATOR_H

#include <Arduino.h>
#include <driver/timer.h>
#include <ADXL375.h>
#include <bitset>
#include "kalmanFilter.h"
#define IR_DATA_SIZE 500
#define VELOCITY_MAX 290

#define NUM_ACCEL_POS 10
#define ACCEL_POS_SPREAD (VELOCITY_MAX/(NUM_ACCEL_POS-1))

class orientator {

    public:
        orientator();
        ~orientator();

        void setup(uint8_t pin, ADXL375 accel);
        void update(double&, double&, double&, double&, double&);
        //boolean updatePeriod();
        //boolean updateOrientation();
        double getAngle();
        double getXAccel();
        double getXSign();
        double getYAccel();
        double getYSign();
        double getZAccel();
        double getZSign();
        void setZeroCrossCallback(void(* callback)());
        void stopZeroCrossCallback();
        void setOnStopCallback(void(* callback)());
        void setOffset(double offset);
        void adjustAngle(double angle);
        void adjustVelocity(double velocity);
        void adjustAccel(double accel);
        double getOffset();
        void setAccelPos(double accelPos);
        void setAccelPos(double accelPos, int index);
        double getAccelPos();
        double getAccelPos(int index);
        double getAproxAccelPos();
        double getPeriod();
        double getVelocity();
        bool getIRData(int i);

    private:
        static std::bitset<IR_DATA_SIZE> IRData; // 500 bit array for incomming IR data
        static uint8_t pin;
        ADXL375 accel;
        double accelPos[NUM_ACCEL_POS] = {0.030};
        double offset = 0;
        double xAccel = 0;
        double yAccel = 0;
        double zAccel = 0;
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