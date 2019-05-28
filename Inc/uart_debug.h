/*
 * uart_debug.h
 *
 *  Created on: Jul 24, 2018
 *      Author: Andrew
 */

#ifndef UART_DEBUG_H_
#define UART_DEBUG_H_

#include <stdlib.h>
#include <string.h> /*memcpy*/
#include "stm32f7xx_hal.h"

#define UART_DEBUG_HEXDUMP_BLOCK_SIZE 16 /*bytes per dump line */
//#define UART_DEBUG_OSX 1 /*lets compiler know if debug should terminate with \r\n for UART_DEBUG_OSX formating */

int8_t uart_debug_init(UART_HandleTypeDef*);
void uart_debug_deinit(void);
uint32_t uart_debug_add_to_buffer(uint8_t*, uint32_t);
void uart_debug_hexdump(uint8_t*, uint32_t);
void uart_debug_hexprint8(uint8_t val);
void uart_debug_hexprint16(uint16_t);
void uart_debug_hexprint32(uint32_t);
void uart_debug_hexprint64(uint64_t val);
void uart_debug_print_uint8(uint8_t);
void uart_debug_print_uint32(uint32_t);
void uart_debug_print_uint64(uint64_t);
void uart_debug_putchar(const unsigned char);
void uart_debug_callback(void);
void uart_debug_newline(void);
void uart_debug_dump_byte(uint8_t);
void uart_debug_send_line(char*);
void uart_debug_send_string(char* str);
void uart_debug_print_bool(uint32_t);
void uart_debug_mem_dump(uint8_t*, size_t);

void uart_debug_tx_complete_callback(DMA_HandleTypeDef *_hdma);


#endif /* UART_DEBUG_H_ */

