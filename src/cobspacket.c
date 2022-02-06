/*
    Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */
#include <string.h>
#include <stdbool.h>
#include "packet.h"
#include "crc.h"

#if PACKET_USE_COBS

/**
 * The latest update aims at achieving optimal re-synchronization in the
 * case if lost data, at the cost of some performance.
 */

// Defines
#define BUFFER_LEN (PACKET_MAX_PL_LEN + 8)

/*

This file implements a COBS receiver similar to that described here:

IEEE/ACM TRANSACTIONS ON NETWORKING, VOL.7, NO. 2, APRIL 1999
Consistent Overhead Byte Stuffing, Stuart Cheshire and Mary Baker
http://www.stuartcheshire.org/papers/cobsforton.pdf


In this implementation the frame format is:

 <------ COB frame ------->
|                          |
+-+-------------+--------+-+
|0|   message   |   crc  |0|
+-+-------------+--------+-+
  |             |        |
   <- message -> <footer>

COBS ensures the message and footer do not contain any 0x00 bytes, which are used for frame delimiting.

The receiver data path is:

UART --> DMA --> circular buffer (rx_dma_circ_buf) --> COBS decoder --> message buffer (msgrx_buf)

Once a message has been received the crc is verified before the caller is notified.
Failed packets are dropped silently. The COBS decoder is halted while received message is being
processed, a larger circular receive buffer might help when receiving next frame and processing the
last one simultaneously.

Example usage:

  msgrx_init(&huart1);
    packet_seek_next();

  while (1)
  {
        if(packet_validate())
        {
            msg_len = msgrx_get_len();
          // process the message in msgrx_buf
            packet_seek_next();
        }
  }

*/

#define MSG_FOOTER_LEN 2

/* COBS decoder state */
enum state
{
    COBS_RX_SCAN,
    COBS_RX_STOPPED,
    COBS_RX_START_BLOCK,
    COBS_RX_DECODE_BLOCK
};


// Private types
typedef struct
{
    volatile unsigned short rx_timeout;
    bool (*send_func)(unsigned char *data, unsigned int len);
    void (*process_func)(unsigned char *data, unsigned int len);

    unsigned char rx_buffer[BUFFER_LEN];
    unsigned char tx_buffer[BUFFER_LEN];
    uint16_t packet_key;

    enum state cobs_state; /* current state */
    uint32_t frame_length; /* framed characters received */
    uint8_t i, code;

} PACKET_STATE_t;

// Private variables
static PACKET_STATE_t m_handler_states[PACKET_HANDLERS];

static void packet_seek_next(PACKET_STATE_t* packet)
{
    packet->cobs_state = COBS_RX_SCAN;
    packet->frame_length = 0;
}

static uint32_t msgrx_get_frame_len(PACKET_STATE_t* packet)
{
    if (packet->cobs_state == COBS_RX_STOPPED)
        return packet->frame_length;
    else
        return 0;
}

static uint32_t msgrx_get_len(PACKET_STATE_t* packet)
{
    uint32_t len = msgrx_get_frame_len(packet);
    if (len > MSG_FOOTER_LEN)
        return len - MSG_FOOTER_LEN;
    return 0;
}

// void printbuf(uint8_t* data, int len) {
//     for(int i = 0; i < len; i++) {
//         printf("%02X ", data[i]);
//     }
//     printf("\n");
// }

static bool packet_validate(PACKET_STATE_t* packet)
{

    /* Validate the unstuffed message */
    uint32_t frm_len = msgrx_get_len(packet);
    //printf("validate %d\n", frm_len);
    //printbuf(packet->rx_buffer, frm_len+2);
    /* Frame should have footer plus at least 1 byte in message */
    if (!frm_len)
    {
        return false;
    }
    uint16_t crc = *(uint16_t*)(&packet->rx_buffer[frm_len]) ^ packet->packet_key;
    if (crc16(packet->rx_buffer, frm_len) != crc)
    {
        //printf("bad csum %04X %04X\n", crc, crc16(packet->rx_buffer, frm_len)+packet->packet_key);
        return false;
    }
    if(packet->process_func) {
        packet->process_func(packet->rx_buffer, frm_len);
    }
    return true;
}

/*
 * Perform COBS data unstuffing as per Cheshire & Baker 1999
 */

/*
For reference, the decoder given in that paper is:

void UnStuffData(const unsigned char *ptr, unsigned long length, unsigned char *dst)
{
  const unsigned char *end = ptr + length;

  while (ptr < end)
  {
    int i, code = *ptr++;
    for (i=1; i<code; i++) *dst++ = *ptr++;
    if (code < 0xFF) *dst++ = 0;
  }
}

*/

#define OutputByte(X)                        \
    do                                       \
    {                                        \
        if (packet->frame_length < sizeof(packet->rx_buffer))    \
        {                                    \
            packet->rx_buffer[packet->frame_length++] = (X); \
        }                                    \
    } while (0);

void packet_process_buffer(uint8_t* data, size_t len, int handler_num) {
  PACKET_STATE_t *packet = &m_handler_states[handler_num];
  packet->rx_timeout = PACKET_RX_TIMEOUT;

  for(int i = 0; i < len; i++) {
    uint8_t rx_data = data[i];

    if (packet->cobs_state == COBS_RX_STOPPED) {
        packet_seek_next(packet);
    }
    if (rx_data == 0)
    {
        if (packet->frame_length > 0)
        {
            /* packet in buffer */
            packet->frame_length--; /* eliminate the trailing phantom zero that COBS produces */
            packet->cobs_state = COBS_RX_STOPPED;
            packet_validate(packet);
            packet_seek_next(packet);
            continue;
        }
        packet->cobs_state = COBS_RX_START_BLOCK;
        continue;
    }

    /* ignore data if in delimiter scan state */
    if(packet->cobs_state == COBS_RX_SCAN) {
        continue;
    }

    /* Non zero char received */
    if (packet->cobs_state == COBS_RX_START_BLOCK)
    {
        /* Start a new COBS block because previous char was 0 (new frame) or
            *  we just finished a block in current frame
            */
        packet->i = 1;
        packet->code = rx_data;
        packet->cobs_state = COBS_RX_DECODE_BLOCK;

        /* Handle special case where code outputs one zero byte only */
        if (packet->code == 1)
        {
            OutputByte(0);
            packet->cobs_state = COBS_RX_START_BLOCK;
        }
    }
    else
    {
        /* cobs_state == COBS_RX_DECODE_BLOCK */
        if (packet->i++ < packet->code)
        {
            OutputByte(rx_data);
        }
        if (packet->i == packet->code)
        {
            if (packet->code < 0xFF)
            {
                OutputByte(0);
            }
            /* End of a block so start a new block */
            packet->cobs_state = COBS_RX_START_BLOCK;
        }
    }
  }
}
void packet_process_byte(uint8_t rx_data, int handler_num)
{
  packet_process_buffer(&rx_data, 1, handler_num);
}


void packet_init(bool (*s_func)(unsigned char *data, unsigned int len),
                 void (*p_func)(unsigned char *data, unsigned int len), int handler_num)
{
    memset(&m_handler_states[handler_num], 0, sizeof(PACKET_STATE_t));
    m_handler_states[handler_num].send_func = s_func;
    m_handler_states[handler_num].process_func = p_func;
}

void packet_set_key(uint16_t key, int handler_num)
{
    m_handler_states[handler_num].packet_key = key;
}

void packet_reset(int handler_num)
{
    //packet_seek_next(&m_handler_states[handler_num]);
}

#define FinishBlock(X)    \
    do                    \
    {                     \
        *code_ptr = (X);  \
        code_ptr = dst++; \
        code = 0x01;      \
    } while (0);

#define AddByte(X)             \
    if (0 == (X))              \
    {                          \
        FinishBlock(code);     \
    }                          \
    else                       \
    {                          \
        *dst++ = (X);          \
        if(dst >= dst_end) { \
            goto error; \
        } \
        code++;                \
        if (0xFF == code)      \
            FinishBlock(code); \
    }

bool packet_send_packet(unsigned char *data, unsigned int len, int handler_num)
{
    PACKET_STATE_t *packet = &m_handler_states[handler_num];

    unsigned char *dst = packet->tx_buffer;
    uint8_t* dst_end = packet->tx_buffer + sizeof(packet->tx_buffer);
    unsigned char *dst_start = packet->tx_buffer;
    *dst++ = 0x00; /* Prepend COBS frame delimiter */
    unsigned char *code_ptr = dst++;
    unsigned char code = 0x01;

    for(int i = 0; i < len; i++)
    {
        AddByte(data[i]);
    }
    uint16_t checksum = crc16(data, len) ^ packet->packet_key;

    AddByte(checksum&0xFF);
    AddByte(checksum>>8);

    FinishBlock(code);

    dst--;
    *dst++ = 0x00; /* Append COBS frame delimiter character 0x00 */
    int32_t frame_len = dst - dst_start;
    if(packet->send_func) {
        return packet->send_func(dst_start, frame_len);
    }

error:
  return false;
}

/**
 * Call this function every millisecond. This is not strictly necessary
 * if the timeout is unimportant.
 */
void packet_timerfunc(void)
{
    for (int i = 0; i < PACKET_HANDLERS; i++)
    {
        if (m_handler_states[i].rx_timeout)
        {
            m_handler_states[i].rx_timeout--;
        }
        else
        {
            packet_reset(i);
        }
    }
}
#endif
