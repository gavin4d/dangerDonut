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
void (* orientator::zeroCrossCallback)() = nullptr;
void (* orientator::onStopCallback)() = nullptr;
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
    if (zeroCrossCallback != nullptr) zeroCrossCallback();
}

void orientator::stopZeroCrossCallback() {
    esp_timer_stop(zeroHeadingTimer);
}

void orientator::setZeroCrossCallback(void (* callback)()) {
    zeroCrossCallback = callback;
}

void orientator::setOnStopCallback(void (* callback)()) {
    onStopCallback = callback;
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

// returns radians since last zero crossing
double orientator::getAngle() {
    if (rotationPeriod == 0) return 0;
    uint32_t oneRotationTime = rotationPeriod*RESOLUTION;
    uint64_t timeSinceZero = (esp_timer_get_time() - zeroCrossingTime + (int)(offset*oneRotationTime));
    return (double)(timeSinceZero % oneRotationTime)*2*PI/oneRotationTime;
}

// returns radians since last zero crossing
double orientator::getAngle(uint64_t zeroCrossingTime) {
    if (rotationPeriod == 0) return 0;
    uint32_t oneRotationTime = rotationPeriod*RESOLUTION;
    uint64_t timeSinceZero = (esp_timer_get_time() - zeroCrossingTime + (int)(offset*oneRotationTime));
    return 2*PI*(double)(timeSinceZero % oneRotationTime)/oneRotationTime;
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

    systemState initialState;
    initialState.angle = 0;
    initialState.angularVelocity = 0;
    initialState.angularAcceleration = 0;
    initialState.variance_A = 0.1;
    initialState.variance_AV = 0;
    initialState.variance_AA = 0;
    initialState.time = esp_timer_get_time();
    filter.setInitState(initialState);
}

void orientator::update(double& angle, double& velocity) {
    float velocityVariance = 0.001;
    float angleVariance = 0.0001;

    double measuredVelocity = 0;
    if(!getAccelVelocity(measuredVelocity))
        if(!getIRVelocity(measuredVelocity))
            velocityVariance = 9999;

    uint64_t measuredZeroCrossingTime = zeroCrossingTime;
    if(!getIROrientation(measuredZeroCrossingTime)) {
        angleVariance = 9999;
        measuredZeroCrossingTime = zeroCrossingTime;
    }

    angle = getAngle(measuredZeroCrossingTime);
    velocity = measuredVelocity;
    filter.makeMeasurement(getAngle(measuredZeroCrossingTime), measuredVelocity, angleVariance, velocityVariance);
    systemState currentState = filter.stateUpdate();
    angularVelocity = currentState.angularVelocity;
    zeroCrossingTime = esp_timer_get_time() - (double)currentState.angle*LSB2ROT*rotationPeriod*RESOLUTION;

    esp_timer_stop(zeroHeadingTimer);
    if (angularVelocity > 10) {
        rotationPeriod = (double)(1000*2*PI)/angularVelocity;
        int64_t startDelay = rotationPeriod*RESOLUTION+zeroCrossingTime-esp_timer_get_time() - (int)(offset*rotationPeriod*RESOLUTION);
        while (startDelay < 0) startDelay += abs(rotationPeriod*RESOLUTION);
        esp_timer_start_once(initTimer, startDelay);
    } else {
        if (onStopCallback != nullptr)
            onStopCallback();
        rotationPeriod = 0;
    }

}

boolean orientator::getAccelVelocity(double& accelVelocity) {
    int16_t x;
    int16_t y;
    if (!accel.getXY(&x, &y)) return false; // return false if read fails
    double normAccel = hypot(x,y);
    accelVelocity = sqrt(normAccel*LSB2MPS2_MULTIPLIER/accelPos); // radians per second

    // report sensor error when maxing out the sensor
    if (normAccel*LSB2G_MULTIPLIER > 280) return false;

    return true;
}

boolean orientator::getIRVelocity(double& IRVelocity) { // auto correlation
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
    IRVelocity = 2*PI*RESOLUTION/(double)delay; // radians per second

    if ((IRData << (IR_DATA_SIZE-SAMPLE_WINDOW)).count() < MIN_IR_DETECTION_PERCENT*SAMPLE_WINDOW) return false;
    if (SAMPLE_WINDOW - (IRData << (IR_DATA_SIZE-SAMPLE_WINDOW)).count() < MIN_IR_DETECTION_PERCENT*SAMPLE_WINDOW) return false;

    return foundPeak;
}

boolean orientator::getIROrientation(uint64_t& IROrientation) { // convolution
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
    IROrientation = esp_timer_get_time() - peak*RESOLUTION;
    return (double)(maxOutput - minOutput) > (MIN_IR_DETECTION_PERCENT*rotationPeriod/4);
}