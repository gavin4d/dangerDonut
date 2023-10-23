#include "frame.h"
#include <stack>

void frame::setPixel(uint8_t x, uint8_t y, uint32_t color) {
    data[x][y] = color;
}

uint32_t frame::getPixel(int8_t x, uint8_t y) {
    if (x < 0) x = x + FRAME_WIDTH;
    return outputBuffer[x][y];
}

void frame::clearFrame() {
    for (int i = 0; i < FRAME_WIDTH; i++) 
        for (int j = 0; j < FRAME_WIDTH; j++)
            setPixel(i, j, 0);
}

void frame::drawSprite(uint8_t x, uint8_t y, uint16_t width, uint16_t height, const uint32_t * spriteData) {
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            setPixel(x + i, y + j, ((spriteData[i+j*width] >> 24) > 0) ? spriteData[i+j*width] : 0);
        }
    }
}

void frame::showFrame() {
    std::copy(&data[0][0], &data[0][0]+FRAME_HEIGHT*FRAME_WIDTH,&outputBuffer[0][0]);
}