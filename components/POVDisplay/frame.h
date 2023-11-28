#ifndef FRAME_H
#define FRAME_H
#include <stdint.h>

#define FRAME_WIDTH 84
#define FRAME_HEIGHT 11

class frame {

    public:
        void setPixel(int16_t x, uint8_t y, uint32_t color);
        uint32_t getPixel(int16_t x, uint8_t y);
        void clearFrame();
        void drawSprite(int16_t x, uint8_t y, uint16_t width, uint16_t height, const uint32_t* spriteData);
        void drawMaskedSprite(int16_t x, uint8_t y, uint16_t width, uint16_t height, const uint32_t* spriteData, uint32_t color);
        void showFrame();

    private:
        uint32_t data[FRAME_WIDTH][FRAME_HEIGHT];
        uint32_t outputBuffer[FRAME_WIDTH][FRAME_HEIGHT];

};
#endif