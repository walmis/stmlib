#pragma once

#include <stdint.h>
#include <cstdlib>
#include <string.h>
#ifdef __arm__
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/cm3/cortex.h>
#endif

#if defined(STM32F0) || defined(STM32F1) || defined(STM32F4) || defined(GD32F1X0)
#define WRITE_SIZE 2
#else
#define WRITE_SIZE 8
#endif

#if defined(STM32F1) || defined(STM32F0) || defined(GD32F1X0)
#define FLASH_PAGESIZE 1024
#elif defined(STM32L4)
#define FLASH_PAGESIZE 2048
#endif

#define FLASH_ALIGN(addr) (addr - (addr & (WRITE_SIZE-1)))
#define ALIGN(x, size) ((x + (size-1)) & ~(size-1))

void flash_erase_page_addr(uint32_t addr);
bool flash_is_erased_sector(uint32_t addr);

template<typename T>
class FlashRef;

template<typename T>
class FlashBlock {
public:
  FlashBlock(uint32_t start_addr, uint32_t length) :
      addr(start_addr), length(length) {
    memset(cache_value, 0xFF, WRITE_SIZE);
  }
  ~FlashBlock() {
    flush_cache();
  }

  void erase() {
    uint32_t start = ALIGN(addr, FLASH_PAGESIZE);
    uint32_t end = addr + length;

    flash_unlock();
    for (uint32_t i = start; i < end; i += FLASH_PAGESIZE) {
      flash_erase_page_addr(i);
    }
    flash_lock();
  }
  
  FlashRef<T> operator [](int index);

  uint32_t addr;
  uint32_t length;

private:
  void flush_cache() {
    if(cache_address && cache_dirty) {
      if(cache_address >= addr+length) {
        memset(cache_value, 0xFF, WRITE_SIZE);
        cache_address = 0;
        cache_dirty = false;
        return;
      }
#if WRITE_SIZE == 8
        flash_unlock();
#if IWDG_TIMEOUT_MS
        iwdg_reset();
#endif
        flash_program_double_word(cache_address, *((uint64_t*)cache_value));
        flash_lock();
#if IWDG_TIMEOUT_MS
        iwdg_reset();
#endif
        if(FLASH_SR) {
          asm("bkpt");
        }
#endif
#if WRITE_SIZE == 2
        flash_unlock();
        flash_program_half_word(cache_address, *((uint16_t*)cache_value));
        flash_lock();
#endif
        memset(cache_value, 0xFF, WRITE_SIZE);
        cache_address = 0;
        cache_dirty = false;
    }
  }

  uint8_t cache_value[WRITE_SIZE];
  bool cache_dirty = false;
  uint32_t cache_address = 0;

 
  friend class FlashRef<T>;

};

template<typename T>
class FlashRef {
public:
  FlashRef(FlashBlock<T> *b, uint32_t address);

  FlashRef& operator =(const T &val);

  operator T&() {
    return *(T*) address;
  }

  T& value() {
    return *(T*) address;
  }

  uint32_t address;
  FlashBlock<T> *b;
};

inline bool flash_is_erased_sector(uint32_t addr) {
  volatile uint32_t start = ALIGN(addr, FLASH_PAGESIZE);
  volatile uint32_t end = start + FLASH_PAGESIZE;
  for(uint32_t addr = start; addr < end; addr+=4) {
    if(*(uint32_t*)addr != 0xFFFFFFFF) {
      return false;
    }
  }
  return true;
}

#ifdef __arm__
inline void flash_erase_page_addr(uint32_t addr) {
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

template<typename T>
inline FlashRef<T> FlashBlock<T>::operator [](int index) {
	if (index < 0) {
		return FlashRef<T>(this, addr + length + index * sizeof(T));
	} else {
		return FlashRef<T>(this, addr + index * sizeof(T));
	}
}

template<typename T>
inline FlashRef<T>& FlashRef<T>::operator =(const T &val) {
	uint32_t v;
	uint32_t pos = 0;
	if (abs(int(b->cache_address - address)) > WRITE_SIZE) {
		b->flush_cache();
		b->cache_address = FLASH_ALIGN(address);
	}
	int coffset = address & (WRITE_SIZE - 1);
	//printf("=cache_address %x offs: %x\n", cache_address, coffset);
	for (int i = 0; i < sizeof(T); i++) {
		b->cache_value[coffset++] = ((uint8_t*) &val)[i];
		b->cache_dirty = true;
		if (coffset == WRITE_SIZE) {
			uint32_t addr = b->cache_address;
			b->flush_cache();
			coffset = 0;
			b->cache_address = addr + WRITE_SIZE;
		}
	}
	return *this;
}

template<typename T>
inline FlashRef<T>::FlashRef(FlashBlock<T> *b, uint32_t address) :
		address(address), b(b) {
}
