/*
 * calib_storage.c
 *
 *  Created on: May 18, 2026
 *      Author: raj_2806
 */

#include "calib_storage.h"
#include "string.h"

/* -------------------------------------------------------------------------
 * calib_storage_load
 * -------------------------------------------------------------------------
 * Flash is memory-mapped on STM32 — plain pointer reads work fine.
 * No HAL unlock is required for reads.
 * ------------------------------------------------------------------------- */
uint8_t calib_storage_load(uint8_t *offsets)
{
    const uint32_t *flash_ptr = (const uint32_t *)CALIB_FLASH_PAGE_ADDR;

    /* Check magic number at the start of the page */
    if (flash_ptr[0] != CALIB_MAGIC)
    {
        return 0; /* no valid calibration data stored */
    }

    /* Copy the 22 offset bytes that follow the 4-byte magic */
    const uint8_t *data_ptr = (const uint8_t *)(CALIB_FLASH_PAGE_ADDR + sizeof(uint32_t));
    memcpy(offsets, data_ptr, CALIB_OFFSET_COUNT);

    return 1; /* valid data loaded */
}

/* -------------------------------------------------------------------------
 * calib_storage_save
 * -------------------------------------------------------------------------
 * STM32F3 Flash rules:
 *   • Must unlock before erase/program, lock after.
 *   • Must erase the entire page before any write (can't overwrite 0→1).
 *   • HAL_FLASH_Program writes one 16-bit halfword at a time with
 *     FLASH_TYPEPROGRAM_HALFWORD.
 *   • All addresses and sizes must be halfword (2-byte) aligned.
 *
 * Layout written (28 bytes = 14 halfwords):
 *   [0x00] magic low halfword   0xCAFE
 *   [0x02] magic high halfword  0xBEEF
 *   [0x04–0x19] 22 offset bytes as 11 halfwords
 *   [0x1A] 1 padding halfword (0xFFFF — keeps erased Flash value)
 * ------------------------------------------------------------------------- */
HAL_StatusTypeDef calib_storage_save(const uint8_t *offsets)
{
    HAL_StatusTypeDef status;

    /* --- Unlock Flash ---------------------------------------------------- */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return status;
    }

    /* --- Erase the calibration page ------------------------------------- */
    FLASH_EraseInitTypeDef erase_cfg;
    uint32_t page_error = 0;

    erase_cfg.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_cfg.PageAddress = CALIB_FLASH_PAGE_ADDR;
    erase_cfg.NbPages     = 1;

    status = HAL_FLASHEx_Erase(&erase_cfg, &page_error);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    /* --- Write magic number (32-bit as two halfwords) ------------------- */
    uint32_t write_addr = CALIB_FLASH_PAGE_ADDR;

    /* Low halfword of magic: 0xCAFE */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                               write_addr,
                               (uint64_t)(CALIB_MAGIC & 0xFFFFU));
    if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }
    write_addr += 2U;

    /* High halfword of magic: 0xBEEF */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                               write_addr,
                               (uint64_t)((CALIB_MAGIC >> 16U) & 0xFFFFU));
    if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }
    write_addr += 2U;

    /* --- Write 22 offset bytes as 11 halfwords -------------------------- */
    for (uint8_t i = 0; i < CALIB_OFFSET_COUNT; i += 2U)
    {
        uint16_t halfword = (uint16_t)offsets[i];

        /*
         * CALIB_OFFSET_COUNT is 22 (even), so we always have a valid
         * second byte — but guard anyway for safety.
         */
        if ((i + 1U) < CALIB_OFFSET_COUNT)
        {
            halfword |= (uint16_t)((uint16_t)offsets[i + 1U] << 8U);
        }

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                   write_addr,
                                   (uint64_t)halfword);
        if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }
        write_addr += 2U;
    }

    /* --- Lock Flash ----------------------------------------------------- */
    HAL_FLASH_Lock();
    return HAL_OK;
}

/* -------------------------------------------------------------------------
 * calib_storage_invalidate
 * -------------------------------------------------------------------------
 * Simply erases the page.  On the next boot calib_storage_load() will find
 * no magic number and return 0, triggering a fresh calibration run.
 * ------------------------------------------------------------------------- */
HAL_StatusTypeDef calib_storage_invalidate(void)
{
    HAL_StatusTypeDef status;

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return status;
    }

    FLASH_EraseInitTypeDef erase_cfg;
    uint32_t page_error = 0;

    erase_cfg.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_cfg.PageAddress = CALIB_FLASH_PAGE_ADDR;
    erase_cfg.NbPages     = 1;

    status = HAL_FLASHEx_Erase(&erase_cfg, &page_error);
    HAL_FLASH_Lock();
    return status;
}

