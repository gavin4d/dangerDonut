#ifndef POVDISPLAY_H
#define POVDISPLAY_H

#include <Arduino.h>
#include <driver/timer.h>
#include "HD107S.h"
#include "frame.h"
#include "imageData.h"

class POVDisplay {

    public:

        POVDisplay() {};
        /**
         * @brief Construct a new POVDisplay object
         * 
         * @param LEDDriver The HD107S LED object to control the LEDs
         */
        POVDisplay(hd107s_config_t LEDConfig);

        /**
         * Sets the brightness of the display
         * @param brightness 0-255 value of brightness
         */
        void setBrightness(uint8_t brightness);

        /**
         * Writes frame data to LEDs
         * @param rotaionPeriod Period of one rotation in milliseconds
         * @param direction Direction the robot is spinning (1: clockwise, -1: withershin)
         */
        void makeFrame(double rotationPeriod, int direction);

        void makeLEDStrip();

        void setPixel(uint16_t x, uint16_t y, uint32_t color);

        void setStripPixel(int16_t y, uint32_t color);

        void drawChar(uint16_t x, uint16_t y, char character, uint32_t color);

        void drawDigit(uint16_t x, uint16_t y, int digit, uint32_t color);

        void drawSprite(int16_t x, uint16_t y, uint16_t spriteID);

        void drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint32_t color);

        void clear();

        void clearStrip();

        void setColumn(uint16_t column);

    private:
        static uint8_t brightness;
        static uint16_t columnPointer;
        static HD107S LED;
        static frame frameData;
        static esp_timer_handle_t writeLEDTimer;
        static int direction;
        uint32_t lineData[FRAME_HEIGHT];

        static void writeLEDColumn(void * args);


};
#endif