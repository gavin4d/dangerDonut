#ifndef ORIENTATOR_H
#define ORIENTATOR_H

#include <Arduino.h>
#include <driver/timer.h>

class orientator {

    public:
        orientator();
        ~orientator();

        void setup(uint8_t);
        boolean updatePeriod();
        double getOrientation();
        void setOffset(double);
        double getOffset();
        uint32_t getPeriod();
        void fillArray(uint32_t[]);

    private:
        struct Node {
            uint64_t time;
            boolean pinState = false;
            Node *next = nullptr;
        };
        static uint8_t pin;
        static Node *head;
        static boolean lastState;
        double offset;
        uint32_t rotationPeriod = 0; // microseconds
        esp_timer_handle_t update_timer;

        static void IRAM_ATTR IR_sensor_ISR();
        void deleteLinkedList(Node **deleteHead);
        static void checkIRCallback(void *args);

        // sums the overlap of a linked list with a delayed copy of itself up until a limit
        uint32_t getCorrelation(Node*, uint64_t, uint32_t);

};
#endif