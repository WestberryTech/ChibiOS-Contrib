#include "spi_master.h"
#include "stdbool.h"
#include "flash_spi.h"

#define NO_PIN (pin_t)(~0)

static pin_t currentSlavePin = NO_PIN;

static SPIConfig spiConfig = {false, NULL, 0, 0, 0, 0};

void spi_init(void) {
    static bool is_initialised = false;
    if (!is_initialised) {
        is_initialised = true;

        // Try releasing special pins for a short time
        setPinInput(SPI_SCK_PIN);
        setPinInput(SPI_MOSI_PIN);
        setPinInput(SPI_MISO_PIN);

        chThdSleepMilliseconds(1);

        palSetPadMode(PAL_PORT(SPI_SCK_PIN), PAL_PAD(SPI_SCK_PIN), PAL_MODE_ALTERNATE(SPI_SCK_PAL_MODE) | PAL_WB32_OTYPE_PUSHPULL | PAL_WB32_OSPEED_HIGH | PAL_WB32_CURRENT_LEVE3);
        palSetPadMode(PAL_PORT(SPI_MOSI_PIN), PAL_PAD(SPI_MOSI_PIN), PAL_MODE_ALTERNATE(SPI_MOSI_PAL_MODE) | PAL_WB32_OTYPE_PUSHPULL | PAL_WB32_OSPEED_HIGH);
        palSetPadMode(PAL_PORT(SPI_MISO_PIN), PAL_PAD(SPI_MISO_PIN), PAL_MODE_ALTERNATE(SPI_MISO_PAL_MODE) | PAL_WB32_OTYPE_PUSHPULL | PAL_WB32_OSPEED_HIGH);
//        PAL_PORT(SPI_SCK_PIN)->CFGMSK = ~PAL_PAD(SPI_SCK_PIN);
//        PAL_PORT(SPI_SCK_PIN)->CURRENT = 0x03 * 0x55555555U;
//        GPIOA->CFGMSK = ~(1 << 5);
//        GPIOA->CURRENT = 0x03 * 0x55555555U;
    }
}

bool spi_start(pin_t slavePin, bool lsbFirst, uint8_t mode, uint16_t divisor) {
    if (currentSlavePin != NO_PIN || slavePin == NO_PIN) {
        return false;
    }

    if (lsbFirst) {
        osalDbgAssert(lsbFirst == FALSE, "unsupported lsbFirst");
    }

    if (divisor < 1) {
        return false;
    }

    spiConfig.SPI_BaudRatePrescaler = (divisor << 2);

    switch (mode) {
        case 0:
            spiConfig.SPI_CPHA = SPI_CPHA_1Edge;
            spiConfig.SPI_CPOL = SPI_CPOL_Low;
            break;
        case 1:
            spiConfig.SPI_CPHA = SPI_CPHA_2Edge;
            spiConfig.SPI_CPOL = SPI_CPOL_Low;
            break;
        case 2:
            spiConfig.SPI_CPHA = SPI_CPHA_1Edge;
            spiConfig.SPI_CPOL = SPI_CPOL_High;
            break;
        case 3:
            spiConfig.SPI_CPHA = SPI_CPHA_2Edge;
            spiConfig.SPI_CPOL = SPI_CPOL_High;
            break;
    }

    currentSlavePin  = slavePin;
    spiConfig.ssport = PAL_PORT(slavePin);
    spiConfig.sspad  = PAL_PAD(slavePin);

    setPinOutput(slavePin);
    spiStart(&SPI_DRIVER, &spiConfig);
//    spiUnselect(&SPI_DRIVER);
    spiSelect(&SPI_DRIVER);

    return true;
}

spi_status_t spi_write(uint8_t data) {
    uint8_t rxData;
    spiExchange(&SPI_DRIVER, 1, &data, &rxData);

    return rxData;
}

spi_status_t spi_read(void) {
    uint8_t data = 0;
    spiReceive(&SPI_DRIVER, 1, &data);

    return data;
}

spi_status_t spi_transmit(const uint8_t *data, uint16_t length) {
    spiSend(&SPI_DRIVER, length, data);
    return SPI_STATUS_SUCCESS;
}

spi_status_t spi_receive(uint8_t *data, uint16_t length) {
    spiReceive(&SPI_DRIVER, length, data);
    return SPI_STATUS_SUCCESS;
}

void spi_stop(void) {
    if (currentSlavePin != NO_PIN) {
        spiUnselect(&SPI_DRIVER);
        spiStop(&SPI_DRIVER);
        currentSlavePin = NO_PIN;
    }
}
