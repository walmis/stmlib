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

#define FLASH_SECTOR(address) (((address) - FLASH_BASE) / FLASH_PAGESIZE)

void flash_erase_page_at_addr(uint32_t addr);
bool flash_is_erased_sector(uint32_t addr);

template<typename T>
class FlashRef;

class FlashBlockBase {
public:
  FlashBlockBase(uint32_t start_addr, uint32_t length) :
      addr(start_addr), length(length) {
    memset(cache_value, 0xFF, WRITE_SIZE);
  }
  ~FlashBlockBase() {
    flush_cache();
  }

  void erase();

  void write(uint32_t address, uint8_t *data, size_t len);

  uint32_t addr;
  uint32_t length;

protected:
  void flush_cache();

  uint8_t cache_value[WRITE_SIZE];
  bool cache_dirty = false;
  uint32_t cache_address = 0;
};

template<typename T = uint8_t>
class FlashBlock : public FlashBlockBase {
public:
  using FlashBlockBase::FlashBlockBase;

  FlashRef<T> operator [](int index);

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

bool flash_is_erased_sector(uint32_t addr);
void flash_erase_page_at_addr(uint32_t addr);

template<typename T>
FlashRef<T> FlashBlock<T>::operator [](int index) {
	if (index < 0) {
		return FlashRef<T>(this, addr + length + index * sizeof(T));
	} else {
		return FlashRef<T>(this, addr + index * sizeof(T));
	}
}

template<typename T>
FlashRef<T>& FlashRef<T>::operator =(const T &val) {
  b->write(address, (uint8_t*) &val, sizeof(val));
	return *this;
}

template<typename T>
FlashRef<T>::FlashRef(FlashBlock<T> *b, uint32_t address) :
		address(address), b(b) {
}


