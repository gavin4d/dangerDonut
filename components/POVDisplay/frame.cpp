#include "frame.h"
#include <stack>
#include <cstring>

void frame::setPixel(int16_t x, uint8_t y, uint32_t color) {
    if (y < FRAME_HEIGHT) data[x % FRAME_WIDTH][y] = color;
}

uint32_t frame::getPixel(int16_t x, uint8_t y) {
    return outputBuffer[x % FRAME_WIDTH][y];
}

void frame::clearFrame() {
    std::memset(data, 0, sizeof(data[0][0]) * FRAME_WIDTH * FRAME_HEIGHT);
}

void frame::drawSprite(int16_t x, uint8_t y, uint16_t width, uint16_t height, const uint32_t * spriteData) {
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            setPixel(x + i, y + j, ((spriteData[i+j*width] >> 24) > 0) ? spriteData[i+j*width] : 0);
        }
    }
}

void frame::drawMaskedSprite(int16_t x, uint8_t y, uint16_t width, uint16_t height, const uint32_t * spriteData, uint32_t maskColor) {
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            setPixel(x + i, y + j, ((spriteData[i+j*width] >> 24) > 0) ? maskColor : 0);
        }
    }
}

void frame::showFrame() {
    std::copy(&data[0][0], &data[0][0]+FRAME_HEIGHT*FRAME_WIDTH,&outputBuffer[0][0]);
}