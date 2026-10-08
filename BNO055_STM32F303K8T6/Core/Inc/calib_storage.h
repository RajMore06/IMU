/*
 * calib_storage.h
 *
 *  Created on: May 18, 2026
 *      Author: raj_2806
 */

#ifndef INC_CALIB_STORAGE_H_
#define INC_CALIB_STORAGE_H_

#include "stm32f3xx_hal.h"

#define CALIB_FLASH_PAGE_ADDR   0x0800F800UL   /* page 63 of 64 */
#define CALIB_FLASH_PAGE_NUMBER 63U
#define CALIB_MAGIC             0xBEEFCAFEUL
#define CALIB_OFFSET_COUNT      22U          /* bytes: 0x55–0x6A in BNO055 */

uint8_t calib_storage_load(uint8_t *offsets);
HAL_StatusTypeDef calib_storage_save(const uint8_t *offsets);
HAL_StatusTypeDef calib_storage_invalidate(void);

#endif
