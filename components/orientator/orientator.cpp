#include "orientator.h"
#include <iostream>

#define RESOLUTION 1000 // microseconds
#define SAMPLE_WINDOW 250000/RESOLUTION
#define MAX_DELAY SAMPLE_WINDOW
#define CORRELATION_TOLERANCE 0.78*SAMPLE_WINDOW
#define MIN_IR_DETECTION_PERCENT 0.2 // use IR for orientation only if more than this amount of a revolution detects IR

uint8_t orientator::pin;
std::bitset<500> orientator::IRData;

orientator::orientator() {
}

orientator::~orientator() {
}

void orientator::setOffset(double offset) {
    orientator::offset = offset-floor(offset);
}

double orientator::getOffset() {
    return offset;
}

void orientator::setAccelPos(double accelPos) {
    orientator::accelPos = 0.02*((accelPos-0.02)/0.02-floor((accelPos-0.02)/0.02))+0.02;
}

double orientator::getAccelPos() {
    return accelPos;
}

uint32_t orientator::getPeriod() {
    return rotationPeriod;
}

double orientator::getOrientation() {
    uint16_t oneRotationTime = rotationPeriod*RESOLUTION;
    int64_t timeSinceZero = (esp_timer_get_time() - orientationTimeStamp - (int)(offset*rotationPeriod*RESOLUTION));
    return (double)(timeSinceZero % oneRotationTime)*2*PI/oneRotationTime;
}

void orientator::checkIRCallback(void *args) {
    IRData <<= 1;
    IRData[0] = digitalRead(pin);
}

void orientator::setup(uint8_t pin, ADXL375 accel) {
    orientator::pin = pin;
    orientator::accel = accel;
    pinMode(pin, INPUT);
    esp_timer_create_args_t new_timer;
    new_timer.callback = &checkIRCallback;
    new_timer.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&new_timer, &update_timer);
    esp_timer_start_periodic(update_timer, RESOLUTION);
}

boolean orientator::updatePeriod() {
    //boolean useIR, useAccel;
    double IRPeriod, AccelPeriod;
    double IRWeight = .3, AccelWeight = .7; // TODO: calculate variance for kalman filter

    useIR = getIRPeriod(&IRPeriod);
    useAccel = getAccelPeriod(&AccelPeriod);
    //ESP_LOGI("PeriodDiff", "%F", IRPeriod - AccelPeriod);
    if (!useAccel && !useIR) return false; // neither sensor gave a reading, uh oh!
    if (useAccel && useIR) {
        rotationPeriod =  (IRPeriod*IRWeight + AccelPeriod*AccelWeight)/(IRWeight+AccelWeight);
    } else if (useIR) {
        rotationPeriod = IRPeriod;
    } else {
        rotationPeriod = AccelPeriod;
    }

    return true;
}

boolean orientator::updateOrientation() {
    //boolean useIR, useAccel;
    uint64_t IROrientation, AccelOrientation;
    double IRWeight = 1, AccelWeight = 0; // TODO: calculate variance for kalman filter

    useIR = getIROrientation(&IROrientation);
    useAccel = getAccelOrientation(&AccelOrientation);
    if (!useAccel && !useIR) return false; // neither sensor gave a reading, uh oh!
    if (useAccel && useIR) {
        orientationTimeStamp =  (IROrientation*IRWeight + AccelOrientation*AccelWeight)/(IRWeight+AccelWeight);
    } else if (useIR) {
        orientationTimeStamp = IROrientation;
    } else {
        orientationTimeStamp = AccelOrientation;
    }
    return true;
}

boolean orientator::getAccelPeriod(double* rotationPeriod) {
    int16_t x;
    int16_t y;
    accel.getXY(&x, &y);
    //ESP_LOGI("frequency", "%F", sqrt(hypot(x,y)*LSB2MPS2_MULTIPLIER/0.03));
    double normAccel = hypot(x,y);
    if (normAccel*LSB2G_MULTIPLIER < 1) return false; // prevent absurdly large periods when stationary
    *rotationPeriod = (1000*2*PI/sqrt(normAccel*LSB2MPS2_MULTIPLIER/accelPos));
    //ESP_LOGI("Period", "%i", *rotationPeriod);

    return true; // TODO: return false if sensor error
}

boolean orientator::getIRPeriod(double* rotationPeriod) { // auto correlation
    //const uint32_t startTime = esp_timer_get_time();
    //using namespace std;
    boolean hasIncreased = false;
    boolean foundPeak = false;
    uint16_t lastsum = 0xffff; // init to max so first loop doesn't detect a increase
    uint16_t delay;
    for (delay = 0; delay < MAX_DELAY; delay ++) {

        uint32_t sum = SAMPLE_WINDOW - ((IRData ^ (IRData >> delay)) << (IR_DATA_SIZE-SAMPLE_WINDOW)).count();

        if (sum > CORRELATION_TOLERANCE) {
            if (hasIncreased && sum < lastsum) { // decreasing
                delay --; // shift it to the actual peak
                foundPeak = true;
                break;
            }
            if (sum > lastsum) // increasing
                hasIncreased = true;
        }
        lastsum = sum;
    }
    //const uint32_t endTime = esp_timer_get_time();
    //cout << "delay: " << delay << ", correlation: " << (double)((lastsum*100)/(SAMPLE_WINDOW))/100 << ", time: " << endTime - startTime << endl;
    *rotationPeriod = delay;

    if ((IRData << (IR_DATA_SIZE-SAMPLE_WINDOW)).count() < MIN_IR_DETECTION_PERCENT*SAMPLE_WINDOW) return false;
    if (SAMPLE_WINDOW - (IRData << (IR_DATA_SIZE-SAMPLE_WINDOW)).count() < MIN_IR_DETECTION_PERCENT*SAMPLE_WINDOW) return false;

    return foundPeak;
}

boolean orientator::getIROrientation(uint64_t* IROrientation) { // convolution
    if (rotationPeriod == 0) return false;
    const uint16_t convolutionSize = rotationPeriod/2;
    uint16_t convolutionOut[(uint16_t)rotationPeriod];
    uint16_t peak = 0;
    uint16_t maxOutput = 0;
    uint16_t minOutput = 0xFFFF;

    for (uint16_t i = 0; i < rotationPeriod; i++) { // first convolution using bitset
        uint16_t sum = 0;
        for (uint16_t j = 0; j < convolutionSize; j++) {
            sum += IRData[(j+i) % (uint16_t)rotationPeriod];
        }
        convolutionOut[i] = sum;
        //convolutionOut[i] = ((IRData << i | IRData >> (rotationPeriod-i)) << convolutionSize).count();
    }

    for (uint16_t i = 0; i < rotationPeriod; i++) { // second convolution to smooth the output
        uint16_t sum = 0;
        for (uint16_t j = 0; j < convolutionSize; j++) {
            sum += convolutionOut[(j+i) % (uint16_t)rotationPeriod];
        }

        if (sum > maxOutput) {
            maxOutput = sum;
            peak = i;
        } else if (sum < minOutput) {
            minOutput = sum;
        }
    }
    *IROrientation = esp_timer_get_time() - peak*RESOLUTION;
    return maxOutput - minOutput > (MIN_IR_DETECTION_PERCENT*rotationPeriod/4);
}

// estimate orientation using rotational period
boolean orientator::getAccelOrientation(uint64_t* AccelOrientation) {
    if (rotationPeriod == 0) return false;
    *AccelOrientation = orientationTimeStamp;
    while (esp_timer_get_time() - *AccelOrientation > rotationPeriod*RESOLUTION) {
        *AccelOrientation += rotationPeriod*RESOLUTION;
    }
    return true;
}