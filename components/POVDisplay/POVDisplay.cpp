#include "POVDisplay.h"
#include "frame.h"
#include <iostream>

HD107S POVDisplay::LED;
frame POVDisplay::frameData;
uint16_t POVDisplay::columnPointer = 0;
uint8_t POVDisplay::brightness = 16;
int POVDisplay::direction = 1;
esp_timer_handle_t POVDisplay::writeLEDTimer;

POVDisplay::POVDisplay(hd107s_config_t LEDConfig) {
    LED.setup(LEDConfig);;
    esp_timer_create_args_t new_timer;
    new_timer.callback = &writeLEDColumn;
    new_timer.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&new_timer, &writeLEDTimer);
}

void POVDisplay::writeLEDColumn(void *args) {
    //ESP_LOGI("write LEDs", "pointer %d", columnPointer);
    for (int i = 0; i < FRAME_HEIGHT; i++) {
        //ESP_LOGI("data", "%x", frameData.getPixel(columnPointer, i));
        LED.setLED(FRAME_HEIGHT-i, HD107S_INTL(frameData.getPixel(-direction*columnPointer, i), brightness));
    }
    LED.update();
    columnPointer++;
    if (columnPointer >= FRAME_WIDTH) {
        columnPointer = 0;
        esp_timer_stop(writeLEDTimer);
    }
}

void POVDisplay::setBrightness(uint8_t brightness) {
    this->brightness = brightness;
}

void POVDisplay::makeFrame(double rotationPeriod, int direction) {
    this->direction = direction;
    frameData.showFrame();
    esp_timer_stop(writeLEDTimer);
    columnPointer = 0;
    esp_timer_start_periodic(writeLEDTimer, (uint64_t)((rotationPeriod*1000)/FRAME_WIDTH));
}

void POVDisplay::makeLEDStrip() {
    esp_timer_stop(writeLEDTimer); // RIP, mysterious watchdog-defying bug (09/2023 - 11/22/2023)
    for (int i = 0; i < FRAME_HEIGHT; i++) {
        LED.setLED(FRAME_HEIGHT-i, HD107S_INTL(lineData[i], brightness));
    }
    LED.update();
}

void POVDisplay::setPixel(uint16_t x, uint16_t y, uint32_t color) {
    frameData.setPixel(x, y, color);
}

void POVDisplay::setStripPixel(int16_t y, uint32_t color) {
    lineData[y % FRAME_HEIGHT] = color;
}

void POVDisplay::drawChar(uint16_t x, uint16_t y, char character, uint32_t color) {
}

void POVDisplay::drawDigit(uint16_t x, uint16_t y, int digit, uint32_t color) {
    frameData.drawMaskedSprite(x, y, spriteSizes[digit % 10][0], spriteSizes[digit % 10][1], spriteArray[digit % 10], color);
}

void POVDisplay::drawSprite(int16_t x, uint16_t y, uint16_t spriteID) {
    frameData.drawSprite(x, y, spriteSizes[spriteID][0], spriteSizes[spriteID][1], spriteArray[spriteID]);
}

void POVDisplay::setColumn(uint16_t column) {
    columnPointer = column;
}

void POVDisplay::clear() {
    frameData.clearFrame();
}

void POVDisplay::clearStrip() {
    for (int i = 0; i < FRAME_HEIGHT; i++)
        lineData[i] = 0xff000000;
}