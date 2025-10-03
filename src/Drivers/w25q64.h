#pragma once

#include "gpio.h"
#include "spi.h"
#include <stdbool.h>

typedef enum { TRANSFER_WAIT, TRANSFER_COMPLETE, TRANSFER_ERROR } SPI_State;

typedef struct {
  uint8_t REG0; // BUSY, WEL, BP1-3, BP, SEC, SRP
  uint8_t REG1; // SRL, QE, (R), LB1-3, CMP, SUS
  uint8_t REG2; // (R), (R), WPS, (R), (R), DRV0-1, (R)
} status_registers;

typedef enum {
  SECTOR_ERASE_4K = 0x20,
  SECTOR_ERASE_32K = 0x52,
  SECTOR_ERASE_64K = 0xD8,
} SECTOR_ERASE_OPTION;

typedef struct {
  uint8_t device_id;
  uint8_t manuf_id;
} device_info;

/**
 * @brief  Program up to 256 (1 page) previously erased (0xFF) registers in DMA
 * mode. BUSY = 1 while operation in progress.
 * @param  data  tx buffer
 * @param  size  size of data in bytes
 * @param  addr  address of first register to write to
 * @retval HAL_StatusTypeDef
 * @warning If data exceeds page size, wrap to the first address of the page.
 */
HAL_StatusTypeDef w25q64_page_program_DMA(uint8_t *data, uint16_t size,
                                          uint32_t addr);

/**
 * @brief  Program up to 256 (1 page) previously erased (0xFF) registers in
 * interrupt mode. BUSY = 1 while operation in progress.
 * @param  data  tx buffer
 * @param  size  size of data in bytes
 * @param  addr  address of first register to write to
 * @retval HAL_StatusTypeDef
 * @warning If data exceeds page size, wrap to the first address of the page.
 */
HAL_StatusTypeDef w25q64_page_program_IT(uint8_t *data, uint16_t size,
                                         uint32_t addr);

/**
 * @brief  Set SPI CS line high or low
 * @param  state GPIO_PIN_RESET / GPIO_PIN_SET
 * @retval void
 */
void w25q64_setCS(GPIO_PinState state);

/**
 * @brief  Read status bits (3 bytes) in blocking mode
 * @param  status_bits  rx buffer
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef w25q64_status(status_registers *status_bits);

/**
 * @brief  SPI transfer done callback
 * @retval void
 */
void w25q64_transfer_done();

/**
 * @brief  Check if device is busy
 * @retval HAL Status
 */
HAL_StatusTypeDef w25q64_isbusy(bool *busy);

/**
 * @brief  Erase sector of specified size. BUSY = 1 while operation in progress.
 * @param  addr sector address
 * @param  option sector size: 4K/32K/64K
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef w25q64_sector_erase(uint32_t addr,
                                      SECTOR_ERASE_OPTION option);

/**
 * @brief  Write enable - is called before every write operation
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef w25q64_we();

/**
 * @brief  Reads device info
 * @param  data rx buffer
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef w25q64_device_info(device_info *data);

/**
 * @brief  Chip erase. BUSY = 1 while operation in progress.
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef w25q64_chip_erase();

/**
 * @brief  Read data in blocking mode.
 *
 *         Unlimited amount of data can be read. First 4 bytes contain
 * instruction.
 * @param  data_rxtx data buffer of size size+4
 * @param size rx data size
 * @param addr address to read from
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef w25q64_read_data(uint8_t *data_rxtx, uint16_t size,
                                   uint32_t addr);

/**
 * @brief  Read data in non-blocking (interrupt) mode.
 *
 *         Unlimited amount of data can be read. First 4 bytes contain
 * instruction.
 * @param  data_rxtx data buffer of size size+4
 * @param size rx data size
 * @param addr address to read from
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef w25q64_read_data_IT(uint8_t *data_rxtx, uint16_t size,
                                      uint32_t addr);

/**
 * @brief  Read data in non-blocking (DMA) mode.
 *
 *         Unlimited amount of data can be read. First 4 bytes contain
 * instruction.
 * @param  data_rxtx data buffer of size size+4
 * @param size rx data size
 * @param addr address to read from
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef w25q64_read_data_DMA(uint8_t *data_rxtx, uint16_t size,
                                       uint32_t addr);

/**
 * @brief  (NOT IMPLEMENTED) Write (up to 64 MB of data). BUSY = 1 while
 * operation in progress.
 * @param  data  tx buffer
 * @param  size  size of data in bytes
 * @param  addr  address of first register to write to
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef w25q64_write(uint8_t *data, uint16_t size, uint32_t page,
                               uint16_t offset);
