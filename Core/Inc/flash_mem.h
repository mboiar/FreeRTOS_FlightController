#pragma once

#include "spi.h"
#include "gpio.h"

#define FLASHMEM_WRITE_ENABLE 0x06
#define FLASHMEM_READ_STATUS1 0x05
#define FLASHMEM_READ_STATUS2 0x35
#define FLASHMEM_READ_STATUS3 0x15
#define FLASHMEM_READ_DATA 0x03
#define FLASHMEM_CHIP_ERASE 0xC7
#define FLASHMEM_DEVICE_ID 0xAB
#define FLASHMEM_PAGE_PROGRAM 0x02
#define FLASHMEM_DEVICE_ID_VALUE 0x16
#define FLASHMEM_MANUF_ID_VALUE 0xEF
#define FLASHMEM_MANUF_ID 0x90

#define MAX_PAGE_SIZE 256
#define MAX_READ_SIZE 256

typedef enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
} SPI_State;

typedef struct {
    uint8_t cmd;
    uint32_t addr;
} spi_instruction;


typedef struct {
    uint8_t BUSY, WEL, BP0, BP1, BP2, TB, SEC, SRP, SRL, QE, LB1, LB2, LB3, CMP, SUS, WPS, DRV0, DRV1;
} status_registers;

typedef enum {
    SECTOR_ERASE_4K = 0x20,
    SECTOR_ERASE_32K = 0x52,
    SECTOR_ERASE_64K = 0xD8,
} SECTOR_ERASE_OPTION;

HAL_StatusTypeDef flashmem_receive(uint8_t* data, uint16_t size);
HAL_StatusTypeDef flashmem_transmit(uint8_t* data, uint16_t size);
HAL_StatusTypeDef flashmem_transmit_receive(uint8_t* tx_data, uint8_t* rx_data, uint16_t size);
HAL_StatusTypeDef flashmem_page_program(uint8_t* data, uint16_t size, uint32_t addr);
HAL_StatusTypeDef flashmem_waitbusy();
void flashmem_setCS(GPIO_PinState state);
HAL_StatusTypeDef flashmem_status(uint8_t* status_bits);
void flashmem_transfer_done();
bool flashmem_isbusy();

HAL_StatusTypeDef flashmem_sector_erase(uint32_t addr, SECTOR_ERASE_OPTION option) {
    uint8_t instr[5] = {FLASHMEM_WRITE_ENABLE, option, (addr>>16) & 0xFF, (addr>>8) & 0xFF, (addr) & 0xFF};
    if (flashmem_transmit(instr, 5) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

bool flashmem_isbusy() {
    return HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY;
}

void flashmem_transfer_done() {
    flashmem_setCS(GPIO_PIN_SET);
}

HAL_StatusTypeDef flashmem_device_id(uint8_t* data) {
    uint8_t instr[6] = {FLASHMEM_MANUF_ID, 0x00, 0x00, 0x00, 0xFF, 0xFF};
    flashmem_setCS(GPIO_PIN_RESET);
    if ((flashmem_transmit_receive(instr, data, 6) != HAL_OK)) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

void flashmem_setCS(GPIO_PinState state) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, state);
}

HAL_StatusTypeDef flashmem_chip_erase() {
    uint8_t instr[2] = {FLASHMEM_WRITE_ENABLE, FLASHMEM_CHIP_ERASE};
    return flashmem_transmit(instr, 2);
}

// Data buffer must have 4 MSB bytes reserved for the instruction. Ignored if BUSY = 1.
HAL_StatusTypeDef flashmem_read_data(uint8_t* data_rxtx, uint16_t size, uint32_t addr) {
    data_rxtx[0] = FLASHMEM_READ_DATA;
    data_rxtx[1] = (addr>>16) & 0xFF;
    data_rxtx[2] = (addr>>8) & 0xFF;
    data_rxtx[3] = (addr) & 0xFF;
    return flashmem_transmit_receive(data_rxtx, data_rxtx, size)!=HAL_OK;
}

HAL_StatusTypeDef flashmem_receive(uint8_t* data, uint16_t size) {
    return HAL_SPI_Receive_DMA(&hspi1, data, size);
}

HAL_StatusTypeDef flashmem_transmit(uint8_t* data, uint16_t size) {
    return HAL_SPI_Transmit_DMA(&hspi1, data, size);
}

HAL_StatusTypeDef flashmem_transmit_receive(uint8_t* tx_data, uint8_t* rx_data, uint16_t size) {
    return HAL_SPI_TransmitReceive_DMA(&hspi1, tx_data, rx_data, size);
}

// Data must have 5 MSB bytes reserved for the instruction. Ignored if BUSY = 1
HAL_StatusTypeDef flashmem_page_program(uint8_t* data_rx, uint16_t size, uint32_t addr) {
    data_rx[0] = FLASHMEM_WRITE_ENABLE;
    data_rx[1] = FLASHMEM_PAGE_PROGRAM;
    data_rx[2] = (addr>>16) & 0xFF;
    data_rx[3] = (addr>>8) & 0xFF;
    data_rx[4] = (addr) & 0xFF;

    // Check for bit protection
    // ...

    return flashmem_transmit(data_rx, size) != HAL_OK;
}

HAL_StatusTypeDef flashmem_status(uint8_t* status_bits) {
    uint8_t txrx[6] = {FLASHMEM_READ_STATUS1, 0, FLASHMEM_READ_STATUS2, 0, FLASHMEM_READ_STATUS3, 0};
    flashmem_transmit_receive(txrx, txrx, 6);
    status_bits[0] = txrx[1];
    status_bits[1] = txrx[3];
    status_bits[2] = txrx[5];
    return HAL_OK;
}
