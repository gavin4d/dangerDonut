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
    if (rotationPeriod < 10 || rotationPeriod >= 500) return;
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

double orientator::getXSign() {
    return std::copysign(1, -xAccel);
}

double orientator::getXAccel() {
    return -xAccel*LSB2G_MULTIPLIER;
}

double orientator::getYSign() {
    return std::copysign(1, yAccel);
}

double orientator::getYAccel() {
    return yAccel*LSB2G_MULTIPLIER;
}

double orientator::getZSign() {
    return std::copysign(1, -zAccel);
}

double orientator::getZAccel() {
    return -zAccel*LSB2G_MULTIPLIER;
}

void orientator::setOffset(double offset) {
    orientator::offset = offset-floor(offset);
}

double orientator::getOffset() {
    return offset;
}

void orientator::setAccelPos(double accelPos) {
    orientator::accelPos[min((int)round(angularVelocity/ACCEL_POS_SPREAD),NUM_ACCEL_POS-1)] = accelPos;
}

void orientator::setAccelPos(double accelPos, int index) {
    orientator::accelPos[index] = accelPos;
}

void orientator::adjustAngle(double angle) {
    filter.adjustAngle(angle);
}

void orientator::adjustVelocity(double velocity) {
    filter.adjustVelocity(velocity);
}

void orientator::adjustAccel(double accel) {
    filter.adjustAccel(accel);
}

double orientator::getAccelPos() {
    return accelPos[min((int)round(angularVelocity/ACCEL_POS_SPREAD), NUM_ACCEL_POS-1)];
}

double orientator::getAccelPos(int index) {
    return accelPos[index];
}

double orientator::getAproxAccelPos() {
    double adjustedVelocity = min(abs(angularVelocity)/ACCEL_POS_SPREAD, (double)NUM_ACCEL_POS-1);
    double decmal = adjustedVelocity - floor(adjustedVelocity);

    return accelPos[(int)adjustedVelocity] + decmal*(accelPos[min((int)adjustedVelocity+1, NUM_ACCEL_POS-1)] - accelPos[(int)adjustedVelocity]);
}

double orientator::getPeriod() {
    return rotationPeriod;
}

double orientator::getVelocity() {
    if (angularVelocity > 13)
        return angularVelocity;
    else
        return 0;
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

void orientator::update(double& angle, double& velocity, double& angleEstimate, double& velocityEstimate, double& accelEstimate) {
    float velocityVariance = 0.003;
    float angleVariance = 0.0001;

    double measuredVelocity = 0;
    if(!getAccelVelocity(measuredVelocity)) {
        if(!getIRVelocity(measuredVelocity)) {
            velocityVariance = INFINITY;
            measuredVelocity = 0;
        }
    }

    uint64_t measuredZeroCrossingTime = zeroCrossingTime;
    if(!getIROrientation(measuredZeroCrossingTime)) {
        angleVariance = INFINITY;
        measuredZeroCrossingTime = zeroCrossingTime;
    }

    angle = getAngle(measuredZeroCrossingTime);
    velocity = measuredVelocity;
    filter.makeMeasurement(getAngle(measuredZeroCrossingTime), measuredVelocity, angleVariance, velocityVariance);
    systemState currentState = filter.stateUpdate();
    angleEstimate = currentState.angle*LSB2RAD;
    velocityEstimate = currentState.angularVelocity;
    accelEstimate = currentState.angularAcceleration;
    angularVelocity = max(min(currentState.angularVelocity, 1.5*VELOCITY_MAX), (double)0);
    //ESP_LOGI("Velocity", "measured: %lf, kalman: %lf", measuredVelocity, angularVelocity);

    esp_timer_stop(zeroHeadingTimer);
    if (angularVelocity > 13) {
        rotationPeriod = (double)(1000*2*PI)/angularVelocity;
        zeroCrossingTime = esp_timer_get_time() - (double)currentState.angle*LSB2ROT*rotationPeriod*RESOLUTION;
        int64_t startDelay = zeroCrossingTime - esp_timer_get_time() - (int)(offset*rotationPeriod*RESOLUTION);
        if (startDelay < 0) startDelay = (startDelay % (int)(abs(rotationPeriod*RESOLUTION))) + (int)(abs(rotationPeriod*RESOLUTION));
        esp_timer_start_once(initTimer, startDelay);
    } else {
        if (onStopCallback != nullptr)
            onStopCallback();
        rotationPeriod = 0;
        zeroCrossingTime = esp_timer_get_time();
    }

}

boolean orientator::getAccelVelocity(double& accelVelocity) {
    int16_t x;
    int16_t y;
    int16_t z;
    if (!accel.getXYZ(x, y, z)) return false; // return false if read fails
    double normAccel = hypot(x,y);
    accelVelocity = sqrt(normAccel*LSB2MPS2_MULTIPLIER/getAproxAccelPos()); // radians per second

    xAccel += 0.01*(x - xAccel); // rolling average of x axis accel
    yAccel += 0.01*(y - yAccel); // rolling average of y axis accel
    zAccel += 0.01*(z - zAccel); // rolling average of z axis accel

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
    if (rotationPeriod < 10 || rotationPeriod >= 500) return false;
    int numIRBits = IRData.count();
    if (numIRBits < MIN_IR_DETECTION_PERCENT*IR_DATA_SIZE || numIRBits > (1-MIN_IR_DETECTION_PERCENT)*IR_DATA_SIZE) return false;
    const uint16_t convolutionSize = rotationPeriod/2;
    uint16_t convolutionOut[(uint16_t)rotationPeriod];
    uint16_t peak = 0;
    uint32_t maxOutput = 0;

    for (uint16_t i = 0; i < (int)rotationPeriod; i++) { // first convolution using bitset
        uint16_t sum = 0;
        for (uint16_t j = 0; j < convolutionSize; j++) {
            sum += IRData[(j+i) % (int)rotationPeriod];
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
        }
    }
    IROrientation = esp_timer_get_time() - peak*RESOLUTION;
    return true;
}