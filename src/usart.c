/*
 *
 * Copyright (C) 2021 Valmantas Paliksa <walmis@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "usart.h"
#include "CBUF.h"

#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/cortex.h>

#include <libopencm3/cm3/nvic.h>
#include <sys/param.h>

//compatibility
#ifdef USART_ISR
#define USART_SR USART_ISR
#define USART_SR_TC USART_ISR_TC
#define USART_SR_IDLE USART_ISR_IDLE
#endif

#if defined(STM32F1)
#define USART_RDR USART_DR
#define USART_TDR USART_DR
#endif

#ifdef GD32F1X0
#define USART1_DMA              DMA1
#define USART1_TX_DMAC          DMA_CHANNEL2
#define USART1_RX_DMAC          DMA_CHANNEL3
#define USART1_TX_DMAC_NVIC     NVIC_DMA_CHANNEL2_3_IRQ
#define USART1_RX_DMAC_NVIC     NVIC_DMA_CHANNEL2_3_IRQ
#define USART1_DMA_TXRX_ISR       dma_channel2_3_isr
#else
#define USART1_DMA DMA1
#define USART1_TX_DMAC          DMA_CHANNEL4
#define USART1_RX_DMAC          DMA_CHANNEL5
#define USART1_TX_DMAC_NVIC     NVIC_DMA1_CHANNEL4_IRQ
#define USART1_RX_DMAC_NVIC     NVIC_DMA1_CHANNEL5_IRQ
#define USART1_DMA_TX_ISR       dma1_channel4_isr
#define USART1_DMA_RX_ISR       dma1_channel5_isr
#endif

#define USART2_DMA DMA1
#define USART2_TX_DMAC DMA_CHANNEL7
#define USART2_RX_DMAC DMA_CHANNEL6

#define USART3_TX_DMA DMA1
#define USART3_TX_DMAC DMA_CHANNEL2
#define USART3_RX_DMAC DMA_CHANNEL3

#define UART4_DMA DMA1
#define UART4_TX_DMAC DMA_CHANNEL5
#define UART4_RX_DMAC DMA_CHANNEL3


int _write(int file, char *ptr, int len);
static void dma_rx_init(struct usart_drv_s* priv);
static void usart_irq_handler(struct usart_drv_s* priv);
static void enqueue_tx_dma(struct usart_drv_s* priv);
static void dma_tx_complete_irq_process(struct usart_drv_s* priv);
static void dma_rx_irq_process(struct usart_drv_s* priv);
static void dma_tx_init(struct usart_drv_s* priv);
static void dma_rx_init(struct usart_drv_s* priv);
static void usart_irq_handler(struct usart_drv_s* priv);


#ifdef USE_USART1
static struct usart_drv_s* _usart1;

void usart1_isr(void) {
  usart_irq_handler(_usart1);
}

#ifdef USART1_DMA_TXRX_ISR
void USART1_DMA_TXRX_ISR() {
  dma_tx_complete_irq_process(_usart1);
  dma_rx_irq_process(_usart1);
}
#else
//rx isr
void USART1_DMA_RX_ISR() {
  dma_rx_irq_process(_usart1);
}
//tx isr
void USART1_DMA_TX_ISR() {
  dma_tx_complete_irq_process(_usart1);
}
#endif
#endif

#ifdef USE_USART2
static struct usart_drv_s* usart2;

void usart2_isr(void) {
  usart_irq_handler(usart2);
}
//rx isr
void dma1_channel6_isr() {
  dma_rx_irq_process(usart2);
}
//tx isr
void dma1_channel7_isr() {
  dma_tx_complete_irq_process(usart2);
}
#endif

#ifdef USE_USART3
static struct usart_drv_s* usart3;

void usart2_isr(void) {
  usart_irq_handler(usart3);
}
//rx isr
void dma1_channel3_isr() {
  dma_rx_irq_process(usart3);
}
//tx isr
void dma1_channel2_isr() {
  dma_tx_complete_irq_process(usart3);
}
#endif

struct usart_drv_s* usart_setup(struct usart_drv_s* priv, uint32_t usart, uint32_t baudrate) {
  memset(priv, 0, sizeof(struct usart_drv_s));

  switch(usart) {
#ifdef USE_USART1
  case USART1:
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_USART1);
    _usart1 = priv;
    priv->usart = USART1;
    priv->dma = USART1_DMA;
    priv->dma_rx_channel = USART1_RX_DMAC;
    priv->dma_tx_channel = USART1_TX_DMAC;
    break;
#endif
#ifdef USE_USART2
  case USART2:
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_USART2);
    usart2 = priv;
    priv->usart = USART2;
    priv->dma = USART2_DMA;
    priv->dma_rx_channel = USART2_RX_DMAC;
    priv->dma_tx_channel = USART2_TX_DMAC;
    break;
#endif
#ifdef USE_USART3
  case USART3:
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_USART3);
    usart3 = priv;
    priv->usart = USART3;
    priv->dma = USART3_DMA;
    priv->dma_rx_channel = USART3_RX_DMAC;
    priv->dma_tx_channel = USART3_TX_DMAC;
    break;
#endif
  }

  if(!priv) return NULL;

  CBUF_Init(priv->rx_buffer);
  CBUF_Init(priv->tx_buffer);

  /* Setup UART parameters. */
  usart_set_baudrate(priv->usart, baudrate);
  usart_set_databits(priv->usart, 8);
  usart_set_stopbits(priv->usart, USART_STOPBITS_1);
  usart_set_parity(priv->usart, USART_PARITY_NONE);
  usart_set_flow_control(priv->usart, USART_FLOWCONTROL_NONE);
  usart_set_mode(priv->usart, USART_MODE_TX_RX);

  /* Enable USART2 Receive interrupt. */
  USART_CR1(priv->usart) |= USART_CR1_IDLEIE;
  USART_CR3(priv->usart) |= USART_CR3_EIE;

  switch(priv->usart) {
  case USART1:
    nvic_enable_irq(NVIC_USART1_IRQ);
    nvic_enable_irq(USART1_TX_DMAC_NVIC);
    nvic_enable_irq(USART1_RX_DMAC_NVIC);
    break;
#ifdef USE_USART2
  case USART2:
    nvic_enable_irq(NVIC_USART2_IRQ);
    nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);
    nvic_enable_irq(NVIC_DMA1_CHANNEL6_IRQ);
    break;
#endif
#ifdef USE_USART3
  case USART3:
    nvic_enable_irq(NVIC_USART3_IRQ);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ);
    nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
    break;
#endif
  }

  /* Finally enable the USART. */
  usart_enable(priv->usart);

  dma_rx_init(priv);
  dma_tx_init(priv);

  return priv;
}


static void _rx_irq_process(struct usart_drv_s* priv) {
    uint32_t irqsave = cm_mask_interrupts(true);
    uint32_t cur_idx = sizeof(priv->dma_rx_buf) - dma_get_number_of_data(priv->dma, priv->dma_rx_channel);
    uint32_t last_idx = priv->dma_last_idx;
    priv->dma_last_idx = cur_idx;
    cm_mask_interrupts(irqsave);

    uint32_t bufspace = CBUF_Space(priv->rx_buffer);

    uint32_t count;
    if(cur_idx < last_idx) { /* dma pointer has wrapped */
      /* add number of data from current pointer to end of buffer
       * and from start to cur_idx */
      count = sizeof(priv->dma_rx_buf) - last_idx + cur_idx;
    } else {
      count = cur_idx - last_idx;
    }
    for(int i = 0; i < count; i++) {
      if(bufspace-- > 0) {
        CBUF_Push(priv->rx_buffer,
            priv->dma_rx_buf[(last_idx+i) % sizeof(priv->dma_rx_buf)]);
      } else {
        priv->rx_overruns++;
      }
    }
    if(count) {
      if(priv->rx_complete_event) {
        priv->rx_complete_event(priv);
      }
    }

}

static void dma_rx_irq_process(struct usart_drv_s* priv) {

  if (dma_get_interrupt_flag(priv->dma, priv->dma_rx_channel, DMA_HTIF)) {
    _rx_irq_process(priv);
    dma_clear_interrupt_flags(priv->dma, priv->dma_rx_channel, DMA_HTIF);
  }
  if (dma_get_interrupt_flag(priv->dma, priv->dma_rx_channel, DMA_TCIF)) {
    _rx_irq_process(priv);
    dma_clear_interrupt_flags(priv->dma, priv->dma_rx_channel, DMA_TCIF);
  }

}

static void dma_tx_complete_irq_process(struct usart_drv_s* priv) {
  CM_ATOMIC_BLOCK()
  {
    if (dma_get_interrupt_flag(priv->dma, priv->dma_tx_channel, DMA_TCIF)) {
      dma_clear_interrupt_flags(priv->dma, priv->dma_tx_channel, DMA_TCIF);

      if(priv->tx_dma_pending) {
        dma_disable_channel(priv->dma, priv->dma_tx_channel);
        CBUF_AdvancePopIdxBy(priv->tx_buffer, priv->tx_dma_pending);
        priv->tx_dma_pending = 0;
        enqueue_tx_dma(priv);
      }

    }
    if (dma_get_interrupt_flag(priv->dma, priv->dma_tx_channel, DMA_TEIF)) {
      dma_disable_channel(priv->dma, priv->dma_tx_channel);
      dma_clear_interrupt_flags(priv->dma, priv->dma_tx_channel, DMA_TEIF);
      priv->tx_dma_pending = 0;

    }
  }
}

static void dma_rx_init(struct usart_drv_s* priv)
{
  /* Reset DMA channel*/
  dma_channel_reset(priv->dma, priv->dma_rx_channel);
#if defined(STM32F1) || defined(GD32F1X0)
  dma_set_peripheral_address(priv->dma, priv->dma_rx_channel, (uint32_t)&USART_RDR(priv->usart));
#else
  dma_set_peripheral_address(priv->dma, priv->dma_rx_channel, (uint32_t)&USART_RDR(priv->usart));
  dma_set_channel_request(priv->dma, priv->dma_rx_channel, 2);
#endif
  dma_set_memory_address(priv->dma, priv->dma_rx_channel, (uint32_t)priv->dma_rx_buf);
  dma_set_number_of_data(priv->dma, priv->dma_rx_channel, sizeof(priv->dma_rx_buf));
  dma_set_read_from_peripheral(priv->dma, priv->dma_rx_channel);
  dma_enable_memory_increment_mode(priv->dma, priv->dma_rx_channel);
  dma_enable_circular_mode(priv->dma, priv->dma_rx_channel);
  dma_set_peripheral_size(priv->dma, priv->dma_rx_channel, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(priv->dma, priv->dma_rx_channel, DMA_CCR_MSIZE_8BIT);
  dma_set_priority(priv->dma, priv->dma_rx_channel, DMA_CCR_PL_VERY_HIGH);

  dma_enable_transfer_complete_interrupt(priv->dma, priv->dma_rx_channel);
  dma_enable_half_transfer_interrupt(priv->dma, priv->dma_rx_channel);

  dma_enable_channel(priv->dma, priv->dma_rx_channel);

  usart_enable_rx_dma(priv->usart);

}

static void dma_tx_init(struct usart_drv_s* priv) {
  dma_channel_reset(priv->dma, priv->dma_tx_channel);
#if defined(STM32F1) || defined(GD32F1X0)
  dma_set_peripheral_address(priv->dma, priv->dma_tx_channel, (uint32_t)&USART_TDR(priv->usart));
#else
  dma_set_peripheral_address(priv->dma, priv->dma_tx_channel, (uint32_t)&USART_TDR(priv->usart));
  dma_set_channel_request(priv->dma, priv->dma_tx_channel, 2);
#endif
  //dma_set_memory_address(priv->dma, priv->dma_tx_channel, (uint32_t)priv->dma_tx_buf);
  //dma_set_number_of_data(priv->dma, priv->dma_tx_channel, sizeof(priv->dma_tx_buf));
  dma_set_read_from_memory(priv->dma, priv->dma_tx_channel);
  dma_enable_memory_increment_mode(priv->dma, priv->dma_tx_channel);
  //dma_enable_circular_mode(priv->dma, priv->dma_tx_channel);
  dma_set_peripheral_size(priv->dma, priv->dma_tx_channel, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(priv->dma, priv->dma_tx_channel, DMA_CCR_MSIZE_8BIT);
  dma_set_priority(priv->dma, priv->dma_tx_channel, DMA_CCR_PL_MEDIUM);

  dma_enable_transfer_complete_interrupt(priv->dma, priv->dma_tx_channel);
  dma_enable_transfer_error_interrupt(priv->dma, priv->dma_tx_channel);

  usart_enable_tx_dma(priv->usart);

}

static void enqueue_tx_dma(struct usart_drv_s* priv) {

  if(!priv->tx_dma_pending) {
    priv->tx_dma_pending = CBUF_ContigLen(priv->tx_buffer);
    if(!priv->tx_dma_pending) {
      //no more data to send
      CBUF_Init(priv->tx_buffer);

      if (USART_SR(priv->usart) & USART_SR_TC) {
        if(priv->den_pin) {
            gpio_clear(priv->den_port, priv->den_pin);
        }
        USART_SR(priv->usart) &= ~USART_SR_TC;
      }

    } else {
      USART_SR(priv->usart) &= ~USART_SR_TC;
      USART_CR1(priv->usart) |= USART_CR1_TCIE;

      if(USART_CR3(priv->usart) & USART_CR3_HDSEL) {
        USART_CR1(priv->usart) &= ~USART_CR1_RE; //disable receiver
      }

      dma_set_number_of_data(priv->dma, priv->dma_tx_channel, priv->tx_dma_pending);
      dma_set_memory_address(priv->dma, priv->dma_tx_channel, (uint32_t)CBUF_GetPopEntryPtr(priv->tx_buffer));
      
      if(priv->den_pin) {
        gpio_set(priv->den_port, priv->den_pin);

      }

      dma_enable_channel(priv->dma, priv->dma_tx_channel);
    }
  } else {
    asm("nop");
  }

}

void usart_poll(usart_drv* priv) {
  //_rx_irq_process(priv);
}

int usart_rx_avail(struct usart_drv_s* priv) {
  return CBUF_Len(priv->rx_buffer);
}

int usart_tx_avail(struct usart_drv_s* priv) {
  return CBUF_Space(priv->tx_buffer);
}

int usart_write(struct usart_drv_s* priv, const uint8_t* buffer, size_t n) {

  if(CBUF_Space(priv->tx_buffer) < n) {
    CM_ATOMIC_BLOCK() {
      enqueue_tx_dma(priv);
    }
    return 0;
  }

  for(int i = 0; i < n; i++) {
    CBUF_Push(priv->tx_buffer, buffer[i]);
  }
  CM_ATOMIC_BLOCK() {
    enqueue_tx_dma(priv);
  }
  return n;

}

int usart_read(struct usart_drv_s* priv, uint8_t* buffer, size_t n) {
  n = MIN(n, CBUF_Len(priv->rx_buffer));
  for(int i = 0; i < n; i++) {
    buffer[i] = CBUF_Pop(priv->rx_buffer);
  }
  return n;
}

int usart_getch(struct usart_drv_s* priv) {
  if(!CBUF_IsEmpty(priv->rx_buffer)) {
    return CBUF_Pop(priv->rx_buffer);
  }
  return -1;
}

int usart_putch(struct usart_drv_s* priv, char c) {
  return usart_write(priv, &c, 1);
}

void usart_set_rs485_den_pin(usart_drv* priv, uint32_t port, uint32_t pin) {
    priv->den_port = port;
    priv->den_pin = pin;
    gpio_clear(port, pin);
}

static void usart_irq_handler(struct usart_drv_s* priv) {
  uint8_t c;

  /*CM_ATOMIC_BLOCK()*/ {

    uint32_t sr = USART_SR(priv->usart);
    //uint32_t cr1 = USART_CR1(priv->usart);

    /* Check if we were called because of TXC. */
    if (sr & USART_SR_TC) {
      if(!priv->tx_dma_pending) {
        if(priv->den_pin) {
            gpio_clear(priv->den_port, priv->den_pin);
        }
        //gpio_clear(GPIOB, GPIO15);
        /* Disable the TXC interrupt, it's no longer needed. */
        USART_CR1(priv->usart) &= ~USART_CR1_TCIE;

        if(USART_CR3(priv->usart) & USART_CR3_HDSEL) {
          USART_CR1(priv->usart) |= USART_CR1_RE; //reenable receiver
        }
      }
#ifdef STM32F1
      USART_SR(priv->usart) &= ~USART_SR_TC; //clear the TC bit
#endif
#if defined(STM32L4) || defined(GD32F1X0)
      USART_ICR(priv->usart) |= USART_ICR_TCCF;
#endif
    }
#ifdef STM32F1
    if (sr & (USART_SR_FE|USART_SR_NE|USART_SR_ORE)) {
      (void)USART_DR(priv->usart);
      priv->rx_errors++;
      _rx_irq_process(priv);
    }
#else
    if (sr & (USART_ISR_FE|USART_ISR_NF|USART_ISR_ORE)) {
      USART_ICR(priv->usart) = USART_ICR_FECF|USART_ICR_NCF|USART_ICR_ORECF;
      priv->rx_errors++;
      _rx_irq_process(priv);
    }
#endif

    if (sr & USART_SR_IDLE) {
#ifdef STM32F1
    (void)USART_DR(priv->usart);
#endif
      _rx_irq_process(priv);

  #if defined(STM32L4) || defined(GD32F1X0)
      USART_ICR(priv->usart) |= USART_ICR_IDLECF;
  #endif

    }
  }
}






//int _write(int file, char *ptr, int len) {
//	int ret;
//
//	if (file == 1) {
//		ret = ring_write(&output_ring, (uint8_t *) ptr, len);
//		if (ret < 0)
//			ret = -ret;
//		USART_CR1(USART2) |= USART_CR1_TXEIE;
//		return ret;
//	}
//	errno = EIO;
//	return -1;
//}
//
//uint8_t uart_read_blocking() {
//	int32_t ret;
//	uint8_t c;
//	do {
//		dma_rx_poll();
//		ret = ring_read_ch(&input_ring, &c);
//	} while(ret == -1);
//	return c;
//}

