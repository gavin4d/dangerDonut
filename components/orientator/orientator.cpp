#include "orientator.h"
#include <iostream>

#define RESOLUTION 1000 // microseconds
#define SAMPLE_WINDOW 250000/RESOLUTION
#define MAX_DELAY SAMPLE_WINDOW
#define CORRELATION_TOLERANCE 0.78*SAMPLE_WINDOW
#define MIN_IR_DETECTION_PERCENT 0.2 // use IR for orientation only if more than this amount of a revolution detects IR

uint8_t orientator::pin;
std::bitset<500> orientator::IRData;
esp_timer_handle_t orientator::zeroHeadingTimer;
void (* orientator::userCallback)() = nullptr;
double orientator::rotationPeriod = 0;

orientator::orientator() {
}

orientator::~orientator() {
}

void orientator::initCallback(void *args) {
    zeroHeadingCallback(args);
    esp_timer_start_periodic(zeroHeadingTimer, rotationPeriod*RESOLUTION);
}

void orientator::zeroHeadingCallback(void *args) {
    if (userCallback != nullptr) userCallback();
}

void orientator::setCallback(void (* callback)()) {
    userCallback = callback;
}

bool orientator::getIRData(int i) {
    return IRData[i];
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

double orientator::getPeriod() {
    return rotationPeriod;
}

// radians since last zero crossing
double orientator::getOrientation() {
    if (rotationPeriod == 0) return 0;
    uint32_t oneRotationTime = rotationPeriod*RESOLUTION;
    uint64_t timeSinceZero = (esp_timer_get_time() - orientationTimeStamp + (int)(offset*oneRotationTime));
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

    new_timer.callback = &initCallback;
    new_timer.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&new_timer, &initTimer);

    new_timer.callback = &zeroHeadingCallback;
    new_timer.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&new_timer, &zeroHeadingTimer);
}

boolean orientator::updatePeriod() {
    //boolean useIR, useAccel;
    double IRPeriod, AccelPeriod;

    if (getAccelPeriod(&AccelPeriod)) {
        rotationPeriod = AccelPeriod;
        // ESP_LOGI("rotation period", "%lf", rotationPeriod);
    } else if (getIRPeriod(&IRPeriod)) {
        rotationPeriod = IRPeriod;
    } else { // neither sensor gave a reading, uh oh!
        return false;
    }

    return true;
}

boolean orientator::updateOrientation() {
    if (rotationPeriod == 0) return false;
    uint64_t IROrientation = 0, AccelOrientation = 0;

    bool useIR = getIROrientation(&IROrientation);
    bool useAccel = getAccelOrientation(&AccelOrientation);

    if (!useAccel && !useIR) { // neither sensor gave a reading, uh oh!
        esp_timer_stop(zeroHeadingTimer);
        return false;
    }
    //ESP_LOGI("orientation", "%lld, mod %lld", AccelOrientation, AccelOrientation % (uint32_t)(rotationPeriod*RESOLUTION));
    if (useAccel && useIR) {
        double IRWeight = 1; //IRFilterO.getWeight((IROrientation-lastIROrientation) % (uint32_t)(rotationPeriod*RESOLUTION));
        double AccelWeight = 1; //AccFilterO.getWeight((rotationPeriod-lastRotationPeriod)*RESOLUTION);
        //ESP_LOGI("weight", "%lf", AccelWeight);
        uint64_t IRZeroDelay = (esp_timer_get_time() - IROrientation) % (uint32_t)(rotationPeriod*RESOLUTION);
        uint64_t AccelZeroDelay = (esp_timer_get_time() - AccelOrientation) % (uint32_t)(rotationPeriod*RESOLUTION);
        orientationTimeStamp = esp_timer_get_time() - (uint64_t)((IRZeroDelay*IRWeight + AccelZeroDelay*AccelWeight)/(IRWeight+AccelWeight));
    } else if (useIR) {
        orientationTimeStamp = IROrientation;
    } else {
        orientationTimeStamp = AccelOrientation;
    }

    esp_timer_stop(zeroHeadingTimer);
    int64_t startDelay = rotationPeriod*RESOLUTION+orientationTimeStamp-esp_timer_get_time();
    while (startDelay < 0) startDelay += (rotationPeriod*RESOLUTION);
    esp_timer_start_once(initTimer, startDelay);
    //esp_timer_start_once(initTimer, rotationPeriod*RESOLUTION+orientationTimeStamp-esp_timer_get_time());

    return true;
}

boolean orientator::getAccelPeriod(double* accelPeriod) {
    int16_t x;
    int16_t y;
    accel.getXY(&x, &y);
    double normAccel = hypot(x,y);
    // prevent absurdly large periods when stationary and maxing out the sensor
    if (normAccel*LSB2G_MULTIPLIER < 0.2 || normAccel*LSB2G_MULTIPLIER > 280) return false;
    *accelPeriod = (1000*2*PI/sqrt(normAccel*LSB2MPS2_MULTIPLIER/accelPos));
    //ESP_LOGI("frequency", "%F", 1/(*rotationPeriod));
    //ESP_LOGI("Period", "%i", *rotationPeriod);

    return true;
}

boolean orientator::getIRPeriod(double* IRPeriod) { // auto correlation
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
    *IRPeriod = delay;

    if ((IRData << (IR_DATA_SIZE-SAMPLE_WINDOW)).count() < MIN_IR_DETECTION_PERCENT*SAMPLE_WINDOW) return false;
    if (SAMPLE_WINDOW - (IRData << (IR_DATA_SIZE-SAMPLE_WINDOW)).count() < MIN_IR_DETECTION_PERCENT*SAMPLE_WINDOW) return false;

    return foundPeak;
}

boolean orientator::getIROrientation(uint64_t* IROrientation) { // convolution
    if (rotationPeriod == 0) return false;
    const uint16_t convolutionSize = rotationPeriod/2;
    uint16_t convolutionOut[(uint16_t)rotationPeriod];
    uint16_t peak = 0;
    uint32_t maxOutput = 0;
    uint32_t minOutput = 0xFFFFFFFF;

    for (uint16_t i = 0; i < min(500, (int)rotationPeriod); i++) { // first convolution using bitset
        uint16_t sum = 0;
        for (uint16_t j = 0; j < convolutionSize; j++) {
            sum += IRData[(j+i) % min(500,(int)rotationPeriod)];
        }
        convolutionOut[i] = sum;
        //convolutionOut[i] = ((IRData << i | IRData >> (rotationPeriod-i)) << convolutionSize).count();
    }

    for (uint16_t i = 0; i < rotationPeriod; i++) { // second convolution to smooth the output
        uint32_t sum = 0;
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
    return (double)(maxOutput - minOutput) > (MIN_IR_DETECTION_PERCENT*rotationPeriod/4);
}

// estimate orientation using rotational period
boolean orientator::getAccelOrientation(uint64_t* AccelOrientation) {
    if (rotationPeriod == 0) return false;
    long catchUpRotations = (esp_timer_get_time() - orientationTimeStamp)/((int)(rotationPeriod*RESOLUTION));
    *AccelOrientation = orientationTimeStamp + catchUpRotations*(rotationPeriod*RESOLUTION);
    //ESP_LOGI("orientation", "catchup: %ld, diff: %lld", catchUpRotations, esp_timer_get_time() - orientationTimeStamp);
    // while (esp_timer_get_time() - *AccelOrientation > rotationPeriod*RESOLUTION) {
    //     *AccelOrientation += rotationPeriod*RESOLUTION;
    // }
    return true;
}