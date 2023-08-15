#include "orientator.h"
#include <iostream>

#define RESOLUTION 1000 // microseconds
#define SAMPLE_WINDOW 250000/RESOLUTION
#define MAX_DELAY SAMPLE_WINDOW
#define CORRELATION_TOLERANCE 0.78*SAMPLE_WINDOW

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

uint32_t orientator::getPeriod() {
    return rotationPeriod;
}

double orientator::getOrientation() {
    return (double)((esp_timer_get_time() - peakTimeStamp) % (rotationPeriod*RESOLUTION))*2*PI/rotationPeriod/RESOLUTION;
}

void orientator::checkIRCallback(void *args) {
    IRData <<= 1;
    IRData[0] = digitalRead(pin);
}

void orientator::setup(uint8_t pin) {
    orientator::pin = pin;
    pinMode(pin, INPUT);
    esp_timer_create_args_t new_timer;
    new_timer.callback = &checkIRCallback;
    new_timer.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&new_timer, &update_timer);
    esp_timer_start_periodic(update_timer, RESOLUTION);
}

boolean orientator::updatePeriod() { // auto correlation
    //const uint32_t startTime = esp_timer_get_time();
    //using namespace std;
    boolean hasIncreased = false;
    boolean foundPeak = false;
    uint16_t lastsum = 0xffff; // init to max so first loop doesn't detect a increase
    uint16_t delay;
    for (delay = 0; delay < MAX_DELAY; delay ++) {

        uint32_t sum = SAMPLE_WINDOW - ((IRData ^ (IRData >> delay)) << SAMPLE_WINDOW).count();

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
    rotationPeriod = delay;
    return foundPeak;
}

void orientator::updateOrientation() { // convolution
    const uint16_t convolutionSize = rotationPeriod/2;
    uint16_t convolutionOut[rotationPeriod];
    uint16_t peak = 0;
    uint16_t maxOutput = 0;

    for (uint16_t i = 0; i < rotationPeriod; i++) { // first convolution using bitset
        uint16_t sum = 0;
        for (uint16_t j = 0; j < convolutionSize; j++) {
            sum += IRData[(j+i) % rotationPeriod];
        }
        convolutionOut[i] = sum;
        //convolutionOut[i] = ((IRData << i | IRData >> (rotationPeriod-i)) << convolutionSize).count();
    }

    for (uint16_t i = 0; i < rotationPeriod; i++) { // second convolution to smooth the output
        uint16_t sum = 0;
        for (uint16_t j = 0; j < convolutionSize; j++) {
            sum += convolutionOut[(j+i) % rotationPeriod];
        }

        if (sum > maxOutput) {
            maxOutput = sum;
            peak = i;
        }

    }

    peakTimeStamp = esp_timer_get_time() - peak*RESOLUTION - offset*(double)rotationPeriod*RESOLUTION;

}