#include "HD107S.h"
#include "iostream"

#define TAG "HD107S"

using namespace std;

HD107S::HD107S() {

}

HD107S::~HD107S() {
	free(buffer);
}

// HD107S& HD107S::operator=(const HD107S &inputHD107S) {
// 	if (this != &inputHD107S) {
//         free(buffer);
//         size_t size = (inputHD107S.numLEDs + 2) * sizeof(hd107s_color_t);
//         buffer = (hd107s_color_t*)heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
//         std::copy(&inputHD107S.buffer[0], &inputHD107S.buffer[0]+size, &buffer[0]);
// 	}
// 	return *this;
// }

void HD107S::setup(hd107s_config_t config) {
	HD107S::numLEDs = config.numLEDs;
	HD107S::DMAChannel = config.DMAChannel;
	HD107S::SPIHost = config.SPIHost;
	HD107S::clockSpeed = config.clockSpeed;
	bus_config = {0};
	bus_config.miso_io_num = -1;
	bus_config.mosi_io_num = config.dataPin;
	bus_config.sclk_io_num = config.clockPin;
	bus_config.quadwp_io_num = -1;
	bus_config.quadhd_io_num = -1;
	dev_config = {0};
	dev_config.clock_speed_hz = config.clockSpeed;
	dev_config.mode = 3;
	dev_config.spics_io_num = -1;
	dev_config.queue_size = 1;
	transaction.length = (8 * ((2 + numLEDs) * sizeof(hd107s_color_t)));
	bus_config.max_transfer_sz = transaction.length;

	spi_bus_initialize(SPIHost, &bus_config, DMAChannel);
	spi_bus_add_device(SPIHost, &dev_config, &device);

	size_t size = (numLEDs + 2) * sizeof(hd107s_color_t);
	buffer = (hd107s_color_t*)heap_caps_malloc(size, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
	memset(buffer, 0, size);
	buffer[0] = 0x0; // start frame
	buffer[numLEDs+1] = 0xFFFFFFFF; // end frame
	for (int i=0; i<numLEDs; i++) buffer[i+1] = SPI_SWAP_DATA_TX(0xE0000000, 32); // LED frame

}

void HD107S::setLED(uint16_t index, hd107s_color_t color) {
	buffer[index] = SPI_SWAP_DATA_TX(color, 32);
}

void HD107S::update() {
	if (!transaction.tx_buffer) {
		transaction.tx_buffer = buffer;
	}

	spi_device_queue_trans(device, &transaction, portMAX_DELAY);
	spi_transaction_t* t;
	spi_device_get_trans_result(device, &t, portMAX_DELAY);
}

uint32_t HD107S::HSVL(double h, double s, double v, char lum) {
	double      hh, p, q, t, ff;
    long        i;
	char  r, g, b;

    if(s <= 0.0) {       // < is bogus, just shuts up warnings
        r = v*255;
        g = v*255;
        b = v*255;
        return HD107S_RGBL(r,g,b,lum);
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
	return HD107S_RGBL(r,g,b,lum);
} 
