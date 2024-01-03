#ifndef ADXL375_H
#define ADXL375_H

#include "driver/spi_master.h"
#include "esp_heap_caps.h"

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
#define ADXL3XX_REG_DEVID (0x00)        /**< Device ID */
#define ADXL3XX_REG_THRESH_TAP (0x1D)   /**< Tap threshold */
#define ADXL3XX_REG_OFSX (0x1E)         /**< X-axis offset */
#define ADXL3XX_REG_OFSY (0x1F)         /**< Y-axis offset */
#define ADXL3XX_REG_OFSZ (0x20)         /**< Z-axis offset */
#define ADXL3XX_REG_DUR (0x21)          /**< Tap duration */
#define ADXL3XX_REG_LATENT (0x22)       /**< Tap latency */
#define ADXL3XX_REG_WINDOW (0x23)       /**< Tap window */
#define ADXL3XX_REG_THRESH_ACT (0x24)   /**< Activity threshold */
#define ADXL3XX_REG_THRESH_INACT (0x25) /**< Inactivity threshold */
#define ADXL3XX_REG_TIME_INACT (0x26)   /**< Inactivity time */
#define ADXL3XX_REG_ACT_INACT_CTL                                              \
  (0x27) /**< Axis enable control for activity and inactivity detection */
#define ADXL3XX_REG_THRESH_FF (0x28) /**< Free-fall threshold */
#define ADXL3XX_REG_TIME_FF (0x29)   /**< Free-fall time */
#define ADXL3XX_REG_TAP_AXES (0x2A)  /**< Axis control for single/double tap */
#define ADXL3XX_REG_ACT_TAP_STATUS (0x2B) /**< Source for single/double tap */
#define ADXL3XX_REG_BW_RATE (0x2C)     /**< Data rate and power mode control */
#define ADXL3XX_REG_POWER_CTL (0x2D)   /**< Power-saving features control */
#define ADXL3XX_REG_INT_ENABLE (0x2E)  /**< Interrupt enable control */
#define ADXL3XX_REG_INT_MAP (0x2F)     /**< Interrupt mapping control */
#define ADXL3XX_REG_INT_SOURCE (0x30)  /**< Source of interrupts */
#define ADXL3XX_REG_DATA_FORMAT (0x31) /**< Data format control */
#define ADXL3XX_REG_DATAX0 (0x32)      /**< X-axis data 0 */
#define ADXL3XX_REG_DATAX1 (0x33)      /**< X-axis data 1 */
#define ADXL3XX_REG_DATAY0 (0x34)      /**< Y-axis data 0 */
#define ADXL3XX_REG_DATAY1 (0x35)      /**< Y-axis data 1 */
#define ADXL3XX_REG_DATAZ0 (0x36)      /**< Z-axis data 0 */
#define ADXL3XX_REG_DATAZ1 (0x37)      /**< Z-axis data 1 */
#define ADXL3XX_REG_FIFO_CTL (0x38)    /**< FIFO control */
#define ADXL3XX_REG_FIFO_STATUS (0x39) /**< FIFO status */
/*=========================================================================*/

#define ADXL3XX_WRITE (0x00)           /**< write bit */
#define ADXL3XX_READ (0x80)            /**< read bit */
#define ADXL3XX_MULTIPLE_BYTES (0x40)  /**< read multiple bytes */

#define LSB2G_MULTIPLIER (0.049) /**< 49mg per lsb */
#define LSB2MPS2_MULTIPLIER (0.48) /**< 0.48 m/s/s per lsb */

/** Used with register 0x2C (ADXL3XX_REG_BW_RATE) to set bandwidth */
typedef enum {
  ADXL3XX_DATARATE_3200_HZ = 0b1111, /**< 3200Hz Bandwidth */
  ADXL3XX_DATARATE_1600_HZ = 0b1110, /**< 1600Hz Bandwidth */
  ADXL3XX_DATARATE_800_HZ = 0b1101,  /**<  800Hz Bandwidth */
  ADXL3XX_DATARATE_400_HZ = 0b1100,  /**<  400Hz Bandwidth */
  ADXL3XX_DATARATE_200_HZ = 0b1011,  /**<  200Hz Bandwidth */
  ADXL3XX_DATARATE_100_HZ = 0b1010,  /**<  100Hz Bandwidth */
  ADXL3XX_DATARATE_50_HZ = 0b1001,   /**<   50Hz Bandwidth */
  ADXL3XX_DATARATE_25_HZ = 0b1000,   /**<   25Hz Bandwidth */
  ADXL3XX_DATARATE_12_5_HZ = 0b0111, /**< 12.5Hz Bandwidth */
  ADXL3XX_DATARATE_6_25HZ = 0b0110,  /**< 6.25Hz Bandwidth */
  ADXL3XX_DATARATE_3_13_HZ = 0b0101, /**< 3.13Hz Bandwidth */
  ADXL3XX_DATARATE_1_56_HZ = 0b0100, /**< 1.56Hz Bandwidth */
  ADXL3XX_DATARATE_0_78_HZ = 0b0011, /**< 0.78Hz Bandwidth */
  ADXL3XX_DATARATE_0_39_HZ = 0b0010, /**< 0.39Hz Bandwidth */
  ADXL3XX_DATARATE_0_20_HZ = 0b0001, /**< 0.20Hz Bandwidth */
  ADXL3XX_DATARATE_0_10_HZ = 0b0000  /**< 0.10Hz Bandwidth (default value) */
} adxl3xx_dataRate_t;

/** Possible interrupts sources on the ADXL343. */
union int_config {
  uint8_t value; /**< Composite 8-bit value of the bitfield.*/
  struct {
    uint8_t overrun : 1;    /**< Bit 0 */
    uint8_t watermark : 1;  /**< Bit 1 */
    uint8_t freefall : 1;   /**< Bit 2 */
    uint8_t inactivity : 1; /**< Bit 3 */
    uint8_t activity : 1;   /**< Bit 4 */
    uint8_t double_tap : 1; /**< Bit 5 */
    uint8_t single_tap : 1; /**< Bit 6 */
    uint8_t data_ready : 1; /**< Bit 7 */
  } bits;                   /**< Individual bits in the bitfield. */
};

/** Possible interrupt pin outputs on the ADXL343. */
typedef enum {
  ADXL3XX_INT1 = 0,
  ADXL3XX_INT2 = 1,
} adxl3xx_int_pin;

struct adxl375_spi_config_t{
    uint8_t misoPin;
    uint8_t mosiPin;
    uint8_t clockPin;
    uint8_t csPin;
    uint32_t clockSpeed = 20000000; // this is 4 times faster than it should be able to do, but it works
    uint8_t DMAChannel = SPI_DMA_CH_AUTO;
    spi_host_device_t SPIHost = SPI2_HOST;
    int16_t xOffset = 0;
    int16_t yOffset = 0;
    int16_t zOffset = 0;
};

class ADXL375 {

private:
    esp_err_t ret;
    spi_host_device_t SPIHost;
    int DMAChannel;
    uint32_t clockSpeed;
    spi_bus_config_t bus_config;
    spi_device_interface_config_t dev_config;
    spi_device_handle_t device;
    uint8_t *rx_buffer;
    uint8_t *tx_buffer;
    int16_t xOffset;
    int16_t yOffset;
    int16_t zOffset;
public:
    ADXL375();
    ~ADXL375();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    int16_t read16(uint8_t reg);
    uint8_t getDeviceID(void);
    bool enableInterrupts(int_config cfg);
    bool mapInterrupts(int_config cfg);
    uint8_t checkInterrupts(void);
    int16_t getX(void);
    int16_t getY(void);
    int16_t getZ(void);
    bool getXY(int16_t &x, int16_t &y);
    bool getXYZ(int16_t &x, int16_t &y, int16_t &z);
    bool setup(adxl375_spi_config_t config);
};


#endif