#include "w25q64.h"

#define w25q64_WRITE_ENABLE 0x06
#define w25q64_READ_STATUS1 0x05
#define w25q64_READ_STATUS2 0x35
#define w25q64_READ_STATUS3 0x15
#define w25q64_READ_DATA 0x03
#define w25q64_CHIP_ERASE 0xC7
#define w25q64_DEVICE_ID 0xAB
#define w25q64_PAGE_PROGRAM 0x02
#define w25q64_DEVICE_ID_VALUE 0x16
#define w25q64_MANUF_ID_VALUE 0xEF
#define w25q64_MANUF_ID 0x90

#define PAGE_SIZE 0x100
#define SECTOR_SIZE 0x1000
#define BLOCK_SIZE 0x10000
#define CHIP_SIZE 0x800000
#define MAX_SPI_DELAY 5

SPI_HandleTypeDef* hspi = &hspi1;


HAL_StatusTypeDef w25q64_sector_erase(uint32_t addr, SECTOR_ERASE_OPTION option) {
    if (w25q64_we() != HAL_OK) {
        return HAL_ERROR;
    }
    uint8_t instr[4] = {option, (addr>>16) & 0xFF, (addr>>8) & 0xFF, (addr) & 0xFF};
    w25q64_setCS(GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, instr, 4, MAX_SPI_DELAY);
    w25q64_setCS(GPIO_PIN_SET);
    return status;
}

HAL_StatusTypeDef w25q64_isbusy(bool* busy) {
    uint8_t instr[2] = {w25q64_READ_STATUS1, 0x00};
    w25q64_setCS(GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, instr, instr, 2, MAX_SPI_DELAY);
    w25q64_setCS(GPIO_PIN_SET);
    if (status == HAL_OK) {
        *busy = instr[1] & 0x01;
    }
    return status;
}

HAL_StatusTypeDef w25q64_we() {
    w25q64_setCS(GPIO_PIN_RESET);
    uint8_t instr = w25q64_WRITE_ENABLE;
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &instr, 1, MAX_SPI_DELAY);
    w25q64_setCS(GPIO_PIN_SET);
    return status;
}

void w25q64_transfer_done() {
    w25q64_setCS(GPIO_PIN_SET);
}

HAL_StatusTypeDef w25q64_device_info(device_info* data) {
    uint8_t instr[6] = {w25q64_MANUF_ID, 0x00, 0x00, 0x00, 0xFF, 0xFF};
    w25q64_setCS(GPIO_PIN_RESET);
    uint8_t data_rx[6] = {0};
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, instr, data_rx, 6, MAX_SPI_DELAY);
    w25q64_setCS(GPIO_PIN_SET);
    data->device_id = data_rx[5];
    data->manuf_id = data_rx[4];
    return status;
}

void w25q64_setCS(GPIO_PinState state) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, state);
}

HAL_StatusTypeDef w25q64_chip_erase() {
    uint8_t instr[1] = {w25q64_CHIP_ERASE};
    if (w25q64_we() != HAL_OK) {
        return HAL_ERROR;
    }
    w25q64_setCS(GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, instr, 1, MAX_SPI_DELAY);
    w25q64_setCS(GPIO_PIN_SET);
    return status;
}

HAL_StatusTypeDef w25q64_read_data(uint8_t* data_rxtx, uint16_t size, uint32_t addr) {
    data_rxtx[0] = w25q64_READ_DATA;
    data_rxtx[1] = (addr>>16) & 0xFF;
    data_rxtx[2] = (addr>>8) & 0xFF;
    data_rxtx[3] = (addr) & 0xFF;
    w25q64_setCS(GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, data_rxtx, data_rxtx, size, HAL_MAX_DELAY);
    w25q64_setCS(GPIO_PIN_SET);
    return status;
}

HAL_StatusTypeDef w25q64_read_data_IT(uint8_t* data_rxtx, uint16_t size, uint32_t addr) {
    data_rxtx[0] = w25q64_READ_DATA;
    data_rxtx[1] = (addr>>16) & 0xFF;
    data_rxtx[2] = (addr>>8) & 0xFF;
    data_rxtx[3] = (addr) & 0xFF;
    w25q64_setCS(GPIO_PIN_RESET);
    return HAL_SPI_TransmitReceive_IT(hspi, data_rxtx, data_rxtx, size);
}

HAL_StatusTypeDef w25q64_read_data_DMA(uint8_t* data_rxtx, uint16_t size, uint32_t addr) {
    data_rxtx[0] = w25q64_READ_DATA;
    data_rxtx[1] = (addr>>16) & 0xFF;
    data_rxtx[2] = (addr>>8) & 0xFF;
    data_rxtx[3] = (addr) & 0xFF;
    w25q64_setCS(GPIO_PIN_RESET);
    return HAL_SPI_TransmitReceive_DMA(hspi, data_rxtx, data_rxtx, size);
}

HAL_StatusTypeDef w25q64_page_program_DMA(uint8_t* data_rx, uint16_t size, uint32_t addr) {
    data_rx[0] = w25q64_PAGE_PROGRAM;
    data_rx[1] = (addr>>16) & 0xFF;
    data_rx[2] = (addr>>8) & 0xFF;
    data_rx[3] = (addr) & 0xFF;

    if (w25q64_we() != HAL_OK) {
        return HAL_ERROR;
    }
    w25q64_setCS(GPIO_PIN_RESET);
    return HAL_SPI_Transmit_DMA(hspi, data_rx, size);
}

HAL_StatusTypeDef w25q64_page_program_IT(uint8_t* data_rx, uint16_t size, uint32_t addr) {
    data_rx[0] = w25q64_PAGE_PROGRAM;
    data_rx[1] = (addr>>16) & 0xFF;
    data_rx[2] = (addr>>8) & 0xFF;
    data_rx[3] = (addr) & 0xFF;

    if (w25q64_we() != HAL_OK) {
        return HAL_ERROR;
    }
    w25q64_setCS(GPIO_PIN_RESET);
    return HAL_SPI_Transmit_IT(hspi, data_rx, size);
}

HAL_StatusTypeDef w25q64_status(status_registers* status_bits) {
    uint8_t txrx[6] = {w25q64_READ_STATUS1, 0, w25q64_READ_STATUS2, 0, w25q64_READ_STATUS3, 0};
    w25q64_setCS(GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, txrx, txrx, 6, MAX_SPI_DELAY);
    w25q64_setCS(GPIO_PIN_SET);
    status_bits->REG0 = txrx[1];
    status_bits->REG1 = txrx[3];
    status_bits->REG2 = txrx[5];
    return status;
}

void Flash_SPI_TxCpltHanlder() {
    w25q64_transfer_done();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(defaultTaskHandle, 0x02, eSetBits, &xHigherPriorityTaskWoken);
    // vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Flash_SPI_TxRxCpltHandler() {
    w25q64_transfer_done();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(defaultTaskHandle, 0x02, eSetBits, &xHigherPriorityTaskWoken);
    // vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}