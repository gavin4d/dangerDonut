#include "ADXL375.h"
#include "esp_log.h"
#include <cstring>


ADXL375::ADXL375() {
}

ADXL375::~ADXL375() {
    free(rx_buffer);
    free(tx_buffer);
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register

    @param reg The register to write to
    @param value The value to write to the register
*/
/**************************************************************************/

void ADXL375::writeRegister(uint8_t reg, uint8_t value) {
    spi_transaction_t transaction = {0};
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.addr = 0;
    transaction.tx_data[0] = reg | ADXL3XX_WRITE;
    transaction.tx_data[1] = value;
    transaction.length = 8*2;
    ESP_LOGI("addr_w", "%2x", (uint8_t)transaction.tx_data[0]);

    ret=spi_device_transmit(device, &transaction);
    assert(ret==ESP_OK);
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register

    @param reg register to read

    @return The results of the register read request
*/
/**************************************************************************/
uint8_t ADXL375::readRegister(uint8_t reg) {
    spi_transaction_t transaction = {0};
    transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    transaction.addr = 0;
    transaction.length = 8*2;
    transaction.tx_data[0] = reg | ADXL3XX_READ;
    transaction.tx_data[1] = 0xFF;
    transaction.rxlength = 8;
    ESP_LOGI("addr_r", "%2x", (uint8_t)transaction.tx_data[0]);

    ret=spi_device_transmit(device, &transaction);
    assert(ret==ESP_OK);
    ESP_LOGI("read", "%2x", transaction.rx_data[1]);

    return transaction.rx_data[1];
}

/**************************************************************************/
/*!
    @brief  Reads 16-bits from the specified register

    @param reg The register to read two bytes from

    @return The 16-bit value read from the reg starting address
*/
/**************************************************************************/

int16_t ADXL375::read16(uint8_t reg) {
    spi_transaction_t transaction;
    transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    transaction.addr = 0;
    transaction.length = 8*3;
    transaction.tx_data[0] = reg | ADXL3XX_READ | ADXL3XX_MULTIPLE_BYTES;
    transaction.tx_data[1] = 0xFF;
    transaction.tx_data[2] = 0xFF;
    transaction.rxlength = 8*2;

    ret=spi_device_transmit(device, &transaction);
    assert(ret==ESP_OK);
    
    uint32_t data;
    std::memcpy(&data, transaction.rx_data, sizeof(int));
    //ESP_LOGI("read", "%4X", transaction.rx_data[1] | (transaction.rx_data[2] << 8));

    return (int16_t)(transaction.rx_data[1] | (transaction.rx_data[2] << 8));
}

/**************************************************************************/
/*!
    @brief  Read the device ID (can be used to check connection)

    @return The 8-bit device ID
*/
/**************************************************************************/
uint8_t ADXL375::getDeviceID(void) {
  // Check device ID register
  return readRegister(ADXL3XX_REG_DEVID);
}

/**************************************************************************/
/*!
    @brief  Enables (1) or disables (0) the interrupts on the specified
            interrupt pin.

    @param cfg The bitfield of the interrupts to enable or disable.

    @return True if the operation was successful, otherwise false.
*/
/**************************************************************************/
bool ADXL375::enableInterrupts(int_config cfg) {
  /* Update the INT_ENABLE register with 'config'. */
  writeRegister(ADXL3XX_REG_INT_ENABLE, cfg.value);

  /* ToDo: Add proper error checking! */
  return true;
}

/**************************************************************************/
/*!
    @brief  'Maps' the specific interrupt to either pin INT1 (bit=0),
            of pin INT2 (bit=1).

    @param cfg The bitfield of the interrupts to enable or disable.

    @return True if the operation was successful, otherwise false.
*/
/**************************************************************************/
bool ADXL375::mapInterrupts(int_config cfg) {
  /* Update the INT_MAP register with 'config'. */
  writeRegister(ADXL3XX_REG_INT_MAP, cfg.value);

  /* ToDo: Add proper error checking! */
  return true;
}

/**************************************************************************/
/*!
    @brief  Reads the status of the interrupt pins. Reading this register
            also clears or deasserts any currently active interrupt.

    @return The 8-bit content of the INT_SOURCE register.
*/
/**************************************************************************/
uint8_t ADXL375::checkInterrupts(void) {
  return readRegister(ADXL3XX_REG_INT_SOURCE);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent X axis value

    @return The 16-bit signed value for the X axis
*/
/**************************************************************************/
int16_t ADXL375::getX(void) { return read16(ADXL3XX_REG_DATAX0) + xOffset; }

/**************************************************************************/
/*!
    @brief  Gets the most recent Y axis value

    @return The 16-bit signed value for the Y axis
*/
/**************************************************************************/
int16_t ADXL375::getY(void) { return read16(ADXL3XX_REG_DATAY0) + yOffset; }

/**************************************************************************/
/*!
    @brief  Gets the most recent Z axis value

    @return The 16-bit signed value for the Z axis
*/
/**************************************************************************/
int16_t ADXL375::getZ(void) { return read16(ADXL3XX_REG_DATAZ0) + zOffset; }

/**************************************************************************/
/*!
    @brief  Reads 3x16-bits from the x, y, and z data register
    @param x reference to return x acceleration data
    @param y reference to return y acceleration data
    @param z reference to return z acceleration data
    @return True if the operation was successful, otherwise false.
*/
/**************************************************************************/
bool ADXL375::getXYZ(int16_t &x, int16_t &y, int16_t &z) { // 30 us
    *tx_buffer = ADXL3XX_REG_DATAX0 | ADXL3XX_READ | ADXL3XX_MULTIPLE_BYTES;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.addr = 0;
    transaction.length = 8*7;
    transaction.rxlength = 0; // same as length
    transaction.tx_buffer = tx_buffer;
    transaction.rx_buffer = rx_buffer;

    ret=spi_device_acquire_bus(device, portMAX_DELAY);
    assert(ret==ESP_OK);
    ret=spi_device_polling_transmit(device, &transaction);
    assert(ret==ESP_OK);
    spi_device_release_bus(device);

    x = (*(rx_buffer+1) | *(rx_buffer+2) << 8) + xOffset;
    y = (*(rx_buffer+3) | *(rx_buffer+4) << 8) + yOffset;
    z = (*(rx_buffer+5) | *(rx_buffer+6) << 8) + zOffset;
    return true;
}

/**************************************************************************/
/*!
    @brief  Reads 2x16-bits from the x, and y data registers
    @param x reference to return x acceleration data
    @param y reference to return y acceleration data
    @return True if the operation was successful, otherwise false.
*/
/**************************************************************************/
bool ADXL375::getXY(int16_t &x, int16_t &y) {
    *tx_buffer = ADXL3XX_REG_DATAX0 | ADXL3XX_READ | ADXL3XX_MULTIPLE_BYTES;
    spi_transaction_t transaction;
    transaction.flags = 0;
    transaction.addr = 0;
    transaction.length = 8*5;
    transaction.rxlength = 0; // same as length
    transaction.tx_buffer = tx_buffer;
    transaction.rx_buffer = rx_buffer;

    ret=spi_device_polling_transmit(device, &transaction);
    assert(ret==ESP_OK);

    x = (*(rx_buffer+1) | *(rx_buffer+2) << 8) + xOffset;
    y = (*(rx_buffer+3) | *(rx_buffer+4) << 8) + yOffset;
    return true;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @return True if the sensor was successfully initialised.
*/
/**************************************************************************/
bool ADXL375::setup(adxl375_spi_config_t config) {

    rx_buffer = (uint8_t*)heap_caps_aligned_alloc(32, 7, MALLOC_CAP_DMA);
    tx_buffer = (uint8_t*)heap_caps_aligned_alloc(32, 7, MALLOC_CAP_DMA);

    DMAChannel = config.DMAChannel;
    SPIHost = config.SPIHost;
    clockSpeed = config.clockSpeed;
    bus_config.miso_io_num = config.misoPin;
    bus_config.mosi_io_num = config.mosiPin;
    bus_config.sclk_io_num = config.clockPin;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 0;
    dev_config = {0};
    dev_config.address_bits = 0;
    dev_config.clock_speed_hz = config.clockSpeed;
    dev_config.mode = 3;
    dev_config.spics_io_num = config.csPin;
    dev_config.queue_size = 10;
    xOffset = config.xOffset;
    yOffset = config.yOffset;
    zOffset = config.zOffset;

    ret=spi_bus_initialize(SPIHost, &bus_config, DMAChannel);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(SPIHost, &dev_config, &device);
    assert(ret==ESP_OK);

    /* Check connection */
    uint8_t deviceid = getDeviceID();
    if (deviceid != 0xE5) {
        /* No ADXL375 detected ... return false */
        return false;
    }

    writeRegister(ADXL3XX_REG_OFSX, (uint8_t)(0));
    writeRegister(ADXL3XX_REG_OFSY, (uint8_t)(0));
    writeRegister(ADXL3XX_REG_OFSZ, (uint8_t)(0));

    // Set measurement frequency
    writeRegister(ADXL3XX_REG_BW_RATE, 0b00001100);

    // Enable measurements
    writeRegister(ADXL3XX_REG_POWER_CTL, 0x08);

    // Force full range (fixes issue with DATA_FORMAT register's reset value)
    // Per datasheet, needs to be D4=0, D3=D1=D0=1
    writeRegister(ADXL3XX_REG_DATA_FORMAT, 0b00001011);

    return true;
}
