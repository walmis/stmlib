#pragma once

#include <stddef.h>
#include <stdint.h>
#include <libopencm3/stm32/dma.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef USART_BUFFER_SIZE
#define USART_BUFFER_SIZE 256
#endif

#ifndef USART_DMA_BUFFER_SIZE
#define USART_DMA_BUFFER_SIZE 64
#endif

struct usart_buffer
{
       volatile uint16_t    m_get_idx;
       volatile uint16_t    m_put_idx;
                uint8_t     m_entry[ USART_BUFFER_SIZE ];

};

struct usart_drv_s {
    uint32_t usart;
    uint32_t dma;
    uint32_t dma_tx_channel;
    uint32_t dma_rx_channel;
    struct usart_buffer tx_buffer;
    struct usart_buffer rx_buffer;
    uint32_t den_port;
    uint32_t den_pin;
    uint8_t dma_rx_buf[USART_DMA_BUFFER_SIZE];
    volatile uint8_t dma_last_idx;
    volatile uint16_t tx_dma_pending;
    volatile uint8_t idle_flag;
    uint32_t rx_overruns;
    /* called in ISR context */
    void (*rx_complete_event)(struct usart_drv_s* priv);
};

#define USE_USART1

struct usart_drv_s;
typedef struct usart_drv_s usart_drv;

usart_drv* usart_setup(struct usart_drv_s* priv, uint32_t usart, uint32_t baudrate);

void usart_set_rs485_den_pin(usart_drv* priv, uint32_t port, uint32_t pin);
int usart_write(usart_drv* priv, const uint8_t* buffer, size_t n);
int usart_read(usart_drv* priv, uint8_t* buffer, size_t n);
int usart_getch(usart_drv* priv);
int usart_putch(usart_drv* priv, char c);
void usart_poll(usart_drv* priv);
bool usart_idle(usart_drv* priv);
int usart_rx_avail(usart_drv* priv);
int usart_tx_avail(usart_drv* priv);


#ifdef __cplusplus
}
#endif

