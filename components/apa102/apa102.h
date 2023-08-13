#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_system.h>
#include <esp_log.h>
#include <driver/spi_master.h>

typedef uint32_t apa102_color_t;

// every element is scaled from 0 to 0xFF
#define APA102_RGBL(r,g,b,lum) ((((r) & 0xFF)<<24) | (((g) & 0xFF)<<16) | (((b) & 0xFF)<<8) | ((((lum) & 0xFF) >> 3)) | 0xE0) 
#define APA102_RGB32(v) (\
	  ((v) << 8)  \
	|(((v) >> 24) \
	? ((v) >> 27) \
	: 0x0000001F) \
	| 0x000000E0;

uint32_t HSV(double h, double s, double v, char lum) {
	double      hh, p, q, t, ff;
    long        i;
	char  r, g, b;

    if(s <= 0.0) {       // < is bogus, just shuts up warnings
        r = v*255;
        g = v*255;
        b = v*255;
        return APA102_RGBL(r,g,b,lum);
    }
    hh = h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = v * (1.0 - s);
    q = v * (1.0 - (s * ff));
    t = v * (1.0 - (s * (1.0 - ff)));

    switch(i) {
    case 0:
        r = v*255;
        g = t*255;
        b = p*255;
        break;
    case 1:
        r = q*255;
        g = v*255;
        b = p*255;
        break;
    case 2:
        r = p*255;
        g = v*255;
        b = t*255;
        break;

    case 3:
        r = p*255;
        g = q*255;
        b = v*255;
        break;
    case 4:
        r = t*255;
        g = p*255;
        b = v*255;
        break;
    case 5:
    default:
        r = v*255;
        g = p*255;
        b = q*255;
        break;
    }
	return APA102_RGBL(r,g,b,lum);
} 

#define RGBL  APA102_RGBL
#define RGB32 APA102_RGB32

#if (CONFIG_LMTZ_APA102_SPI_HSPI)
#define CONFIG_LMTZ_APA102_SPI_HOST SPI2_HOST
#elif (CONFIG_LMTZ_APA102_SPI_VSPI)
#define CONFIG_LMTZ_APA102_SPI_HOST VSPI_HOST
#endif

typedef struct
{
	union 
	{
		apa102_color_t* txbuffer;
		apa102_color_t* leds;
	};
		//[CONFIG_LMTZ_APA102_MAX_TRANSFER/sizeof(apa102_color_t)];

	int (*init)(int nleds);
	int (*deinit)();
	int (*refresh)();
	int (*update)();

	uint16_t phase;
	int spi_host;
	int dma_channel;
	int count;
	spi_bus_config_t bus_config;
	spi_device_interface_config_t dev_config;
	spi_transaction_t transaction;
	spi_device_handle_t device;

} apa102_driver_t;

//typedef void (*apa102_refresh_cb)(apa102_t* sender, apa102_color_t* color, size_t len, void* context);

extern apa102_driver_t APA102;

#define LEDSTRIP APA102
