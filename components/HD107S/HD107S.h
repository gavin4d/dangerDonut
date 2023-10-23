#ifndef HD107S_H
#define HD107S_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_system.h>
#include <esp_log.h>
#include <driver/spi_master.h>

typedef uint32_t hd107s_color_t;

#define HD107S_RGBL(r,g,b,lum) (((b & 0xFF)<<16) | ((g & 0xFF)<<8) | (r & 0xFF) | ((((lum & 0xFF) >> 3) | 0xE0) << 24))
#define HD107S_INTL(int,lum) (((int & 0xFF)<<16) | (((int >> 8) & 0xFF)<<8) | ((int >> 16) & 0xFF) | ((((lum & 0xFF) >> 3) | 0xE0) << 24))

#define RGBL  HD107S_RGBL

struct hd107s_config_t{
    uint8_t dataPin;
    uint8_t clockPin;
    uint16_t numLEDs = 1;
    uint32_t clockSpeed = 1000000;
    uint8_t DMAChannel = SPI_DMA_CH_AUTO;
    spi_host_device_t SPIHost = SPI3_HOST;
};

class HD107S {

    public:
        HD107S();
        ~HD107S();
		HD107S& operator=(const HD107S &inputHD107S);

        void setup(hd107s_config_t config);
        void setLED(uint16_t index, hd107s_color_t color);
        void update();
        uint32_t HSVL(double h, double s, double v, char lum);

    private:
        hd107s_color_t* buffer;
        int16_t numLEDs;
        spi_host_device_t SPIHost;
	    int DMAChannel;
        uint32_t clockSpeed;
        spi_bus_config_t bus_config;
        spi_device_interface_config_t dev_config;
        spi_transaction_t transaction;
        spi_device_handle_t device;
};
#endif