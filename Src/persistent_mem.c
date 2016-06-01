/*
 * upsat-comms-software: Communication Subsystem Software for UPSat satellite
 *
 *  Copyright (C) 2016, Libre Space Foundation <http://librespacefoundation.org/>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "persistent_mem.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"

static inline uint8_t
is_valid_flash_mem(uint32_t *addr)
{
  return 0;
}

/**
 * Stores a word at the flash memory of the processor
 * @param addr the address to write
 * @param word the data that are going to be stored
 */
void
comms_write_persistent_word(uint32_t *addr, uint32_t word)
{
  //HAL_FLASH_Lock();
  //HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word);
  //HAL_FLASH_Unlock();
}

/**
 * Reads a word from the flash memory
 * @param addr the address to read
 * @return the word value of the address \p addr
 */
uint32_t
comms_read_persistent_word(uint32_t *addr)
{
  return *addr;
}
