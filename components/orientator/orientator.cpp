#include "orientator.h"
#include <iostream>

#define AUTO_CORRELATION_RESOLUTION 1000 // microseconds
#define SAMPLE_WINDOW 250000 // microseconds
#define MAX_DELAY SAMPLE_WINDOW // microseconds
#define CORRELATION_TOLERANCE 0.75

orientator::Node *orientator::head = nullptr;
uint8_t orientator::pin;
boolean orientator::lastState = false;

orientator::orientator() {
}

orientator::~orientator() {
    deleteLinkedList(&head);
}

void IRAM_ATTR orientator::IR_sensor_ISR() {
    Node *IRUpdate = new Node;
    IRUpdate->time = esp_timer_get_time();
    // if (head) {
    //     if (IRUpdate->time - head->time < 1000) {
    //         delete IRUpdate;
    //         return;
    //     }
    // }
    IRUpdate->next = head;
    IRUpdate->pinState = digitalRead(pin);
    head = IRUpdate;
}

void orientator::checkIRCallback(void *args) {
    boolean pinState = digitalRead(pin);
    if (pinState != lastState) {
        Node *IRUpdate = new Node;
        IRUpdate->time = esp_timer_get_time();
        IRUpdate->next = head;
        IRUpdate->pinState = pinState;
        head = IRUpdate;
        lastState = pinState;
    }
}

void orientator::fillArray(uint32_t *array) {
    Node *current = head;
    for (int i = 0; i < 127 && current; i++) {
        array[i] = current->time;
        current = current->next;
    }
}

void orientator::setOffset(double offset) {
    orientator::offset = offset;
}

double orientator::getOffset() {
    return offset;
}

uint32_t orientator::getPeriod() {
    return orientator::rotationPeriod;
}

void orientator::setup(uint8_t pin) {
    orientator::pin = pin;
    pinMode(pin, INPUT);
    // esp_timer_create_args_t new_timer;
    // new_timer.callback = &checkIRCallback;
    // new_timer.dispatch_method = ESP_TIMER_TASK;
    // esp_timer_create(&new_timer, &update_timer);
    // esp_timer_start_periodic(update_timer, 1000);
    attachInterrupt(pin, IR_sensor_ISR, CHANGE);
}

boolean orientator::updatePeriod() { // auto correlation
    using namespace std;
    if (head == nullptr) return false;
    if (head->next == nullptr) return false;
    Node *startHead = head;
    const uint64_t startTime = startHead->time;
    boolean hasIncreased = false;
    boolean foundPeak = false;
    uint32_t lastsum = 0xffffffff; // init to max so first loop doesn't detect a increase
    uint32_t delay;
    for (delay = 0; delay < MAX_DELAY; delay += AUTO_CORRELATION_RESOLUTION) {
        uint32_t sum = getCorrelation(startHead, startTime-SAMPLE_WINDOW, delay);
        if (sum > CORRELATION_TOLERANCE*(double)SAMPLE_WINDOW) {
            if (hasIncreased && sum < lastsum) { // decreasing
                delay -= AUTO_CORRELATION_RESOLUTION; // shift it to the actual peak
                foundPeak = true;
                break;
            }
            if (sum > lastsum) // increasing
                hasIncreased = true;
        }
        lastsum = sum;
    }

    Node *trimNode = startHead;
    while (trimNode) {
        if (trimNode->time < startTime-MAX_DELAY-SAMPLE_WINDOW) {
            deleteLinkedList(&(trimNode->next));
            trimNode->next = nullptr;
        }
        trimNode = trimNode->next;
    }

    cout << "delay: " << delay << ", correlation: " << (double)((lastsum*100)/SAMPLE_WINDOW)/100 << endl;
    rotationPeriod = delay;
    return foundPeak;
}

uint32_t orientator::getCorrelation(Node *head, uint64_t lowerLimit, uint32_t delay) {
    uint32_t sum = 0;
    Node *currentNode = head;
    Node *delayedNode = head;
    while (currentNode->next && delayedNode->next && lowerLimit < currentNode->time && lowerLimit < delayedNode->time+delay) {

        // jump past the newest node if its next node is still newer than the other one
        // makes sure there is actual overlap where we are testing
        if (delayedNode->time+delay > currentNode->time) {
            if (delayedNode->next->time+delay > currentNode->time) {
                delayedNode = delayedNode->next;
                continue;
            }
        } else {
            if (currentNode->next->time > delayedNode->time+delay) {
                currentNode = currentNode->next;
                continue;
            }
        }
        boolean xnor = !(delayedNode->pinState xor currentNode->pinState);

        // find the closest next node and move to it
        if (delayedNode->next->time+delay >= currentNode->next->time) {
            delayedNode = delayedNode->next;
        } else {
            currentNode = currentNode->next;
        }
        if (xnor) {
            sum += labs(max(delayedNode->time+delay, lowerLimit) - max(currentNode->time, lowerLimit));
        }

    }
    return sum;
}

double orientator::getOrientation() { // convolution
    return 0;
}

void orientator::deleteLinkedList(Node **deleteHead) {
    Node *current = *deleteHead;
    while (current) {
        *deleteHead = current->next;
        delete current;
        current = *deleteHead;
    }
    *deleteHead = NULL;
}