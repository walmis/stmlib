/*
 * flash.cpp
 *
 *  Created on: Nov 5, 2021
 *      Author: walmis
 */


#include "flash.h"

#define ALIGN(x, size) ((x + (size-1)) & ~(size-1))
#define FLASH_WRITE_ALIGN(addr) (addr - (addr & (WRITE_SIZE-1)))
#define FLASH_PAGE_ALIGN(x) ALIGN(x, FLASH_PAGESIZE)

void FlashBlockBase::erase() {
  uint32_t start = ALIGN(addr, FLASH_PAGESIZE);
  uint32_t end = addr + length;
  flash_unlock();
  for (uint32_t i = start; i < end; i += FLASH_PAGESIZE) {
    flash_erase_page_at_addr(i);
  }
  flash_lock();
}

void FlashBlockBase::write(uint32_t address, uint8_t *data, size_t len) {
  if (abs(int(cache_address - address)) > WRITE_SIZE) {
    flush_cache();
    cache_address = FLASH_WRITE_ALIGN(address);
  }
  int coffset = address & (WRITE_SIZE - 1);
  //printf("=cache_address %x offs: %x\n", cache_address, coffset);
  for (uint32_t i = 0; i < len; i++) {
    cache_value[coffset++] = data[i];
    cache_dirty = true;
    if (coffset == WRITE_SIZE) {
      uint32_t addr = cache_address;
      flush_cache();
      coffset = 0;
      cache_address = addr + WRITE_SIZE;
    }
  }
}

void FlashBlockBase::flush_cache() {
  if (cache_address && cache_dirty) {
    if (cache_address >= addr + length) {
      memset(cache_value, 0xFF, WRITE_SIZE);
      cache_address = 0;
      cache_dirty = false;
      return;
    }
#if WRITE_SIZE == 8
#if IWDG_TIMEOUT_MS
        iwdg_reset();
#endif
        flash_program_double_word(cache_address, *((uint64_t*)cache_value));
#if IWDG_TIMEOUT_MS
        iwdg_reset();
#endif
        if(FLASH_SR) {
          asm("bkpt");
        }
#endif
#if WRITE_SIZE == 2
    flash_unlock();
    flash_program_half_word(cache_address, *((uint16_t*) cache_value));
    flash_lock();
#endif
    memset(cache_value, 0xFF, WRITE_SIZE);
    cache_address = 0;
    cache_dirty = false;
  }
}

#ifdef __arm__
void flash_erase_page_at_addr(uint32_t addr) {
  addr = FLASH_PAGE_ALIGN(addr);
#if defined(STM32F1) || defined(GD32F1X0)
  flash_erase_page(addr);
#elif defined(STM32L4)
#if IWDG_TIMEOUT_MS
  iwdg_reset();
#endif
  if(!flash_is_erased_sector(addr)) {
    flash_erase_page((addr - FLASH_BASE) / FLASH_PAGESIZE);
  }
#if IWDG_TIMEOUT_MS
  iwdg_reset();
#endif
#endif
}
#endif

bool flash_is_erased_sector(uint32_t addr) {
  volatile uint32_t start = FLASH_PAGE_ALIGN(addr);
  volatile uint32_t end = start + FLASH_PAGESIZE;
  for(uint32_t addr = start; addr < end; addr+=4) {
    if(*(uint32_t*)addr != 0xFFFFFFFF) {
      return false;
    }
  }
  return true;
}
