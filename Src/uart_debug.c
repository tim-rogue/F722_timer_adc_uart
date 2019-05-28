/*
 * uart_debug.c
 *
 *  Created on: Feb 2, 2016
 *      Author: me
 *
 *  make sure to call uart_debug_init with a valid (and initialized) UART_HandleTypeDef* before calling any other function
 *  call uart_debug_deinit to null the reference and disable things.
 *  UART_Debug assumes it has exclusive control over the UART
 *
 */

#ifndef UART_DEBUG_C
#define UART_DEBUG_C

#include "uart_debug.h"

//#define ENABLE_UART_DMA_CACHE_MAINTENANCE 1

//#define DEBUG_UART_TRANSMIT_BUFFER_LEN  SRAM2_UART_DEBUG_TX_BUFF_LEN
#define DEBUG_UART_TRANSMIT_BUFFER_LEN 8192

//need 3 characters per byte (two digits plus a space/newline, plus a possible carriage return for silly systems.
#define DEBUG_UART_TEMP_BUFFER_LEN ((UART_DEBUG_HEXDUMP_BLOCK_SIZE * 3) + 1) /* 16 bytes at 2 chars + one space per byte except the last, which is a newline*/



UART_HandleTypeDef* debugUART = NULL;

/* local functions*/
static void uart_debug_transmit(void);

//uint8_t* uart_debug_buffer = (uint8_t*) SRAM2_UART_DEBUG_TX_BUFF;
//now statucally allocation a buffer because doing it the previous way
// causes a hard fault in atollic on th F722
uint8_t uart_debug_buffer[DEBUG_UART_TRANSMIT_BUFFER_LEN];

uint8_t uart_debug_buffer_temp[DEBUG_UART_TEMP_BUFFER_LEN];
volatile uint32_t uart_debug_buffer_head = 0;
volatile uint32_t uart_debug_buffer_tail = 0;
volatile uint32_t uart_debug_buffer_transmit_head = 0;

volatile uint32_t uart_debug_buffer_remaining = DEBUG_UART_TRANSMIT_BUFFER_LEN;
//volatile uint32_t uart_debug_buffer_remaining = DEBUG_UART_TEMP_BUFFER_LEN;

volatile int8_t uart_debug_busy_flag = -1;

int8_t uart_debug_init(UART_HandleTypeDef* uart) {
    if(NULL == uart) {
        /* why are we being called with an null pointer? */
        return -1;
    }
    debugUART = uart;
    uart_debug_busy_flag = 0;
    return 0;
}

void uart_debug_deinit(void) {
    debugUART = NULL;
    uart_debug_busy_flag = -1;
    uart_debug_buffer_head = 0;
    uart_debug_buffer_tail = 0;
    uart_debug_buffer_transmit_head = 0;
    uart_debug_buffer_remaining = DEBUG_UART_TRANSMIT_BUFFER_LEN;
}

void uart_debug_print_uint8(uint8_t val) {
    uart_debug_buffer_temp[0] = (val / 100) + '0';
    val %= 100;
    uart_debug_buffer_temp[1] = (val / 10) + '0';
    uart_debug_buffer_temp[2] = (val % 10) + '0';
    uart_debug_add_to_buffer(uart_debug_buffer_temp, 3);
}

void uart_debug_print_uint32(uint32_t val) {
    uint32_t pos = DEBUG_UART_TEMP_BUFFER_LEN;
    do {
        uart_debug_buffer_temp[--pos] = (val % 10) + '0';
        val /= 10;
    } while(val);
    uart_debug_add_to_buffer(uart_debug_buffer_temp + pos, DEBUG_UART_TEMP_BUFFER_LEN - pos);
}

void uart_debug_print_uint64(uint64_t val) {
    uint64_t pos = DEBUG_UART_TEMP_BUFFER_LEN;
    do {
        uart_debug_buffer_temp[--pos] = (val % 10) + '0';
        val /= 10;
    } while(val);
    uart_debug_add_to_buffer(uart_debug_buffer_temp + pos, DEBUG_UART_TEMP_BUFFER_LEN - pos);
}

void uart_debug_hexprint8(uint8_t val) {
    uart_debug_buffer_temp[0] = '0';
    uart_debug_buffer_temp[1] = 'x';
    for(int8_t i = 3; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            uart_debug_buffer_temp[i] = (c - 10) + 'A';
        } else {
            uart_debug_buffer_temp[i] = c + '0';
        }
        val >>= 4;
    }
    uart_debug_add_to_buffer(uart_debug_buffer_temp, 4);
}

void uart_debug_hexprint16(uint16_t val) {
    uart_debug_buffer_temp[0] = '0';
    uart_debug_buffer_temp[1] = 'x';
    for(int8_t i = 5; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            uart_debug_buffer_temp[i] = (c - 10) + 'A';
        } else {
            uart_debug_buffer_temp[i] = c + '0';
        }
        val >>= 4;
    }
    uart_debug_add_to_buffer(uart_debug_buffer_temp, 6);
}

void uart_debug_hexprint32(uint32_t val) {
    uart_debug_buffer_temp[0] = '0';
    uart_debug_buffer_temp[1] = 'x';
    for(int8_t i = 9; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            uart_debug_buffer_temp[i] = (c - 10) + 'A';
        } else {
            uart_debug_buffer_temp[i] = c + '0';
        }
        val >>= 4;
    }
    uart_debug_add_to_buffer(uart_debug_buffer_temp, 10);
}

void uart_debug_hexprint64(uint64_t val) {
    uart_debug_buffer_temp[0] = '0';
    uart_debug_buffer_temp[1] = 'x';
    for(int8_t i = 17; i > 1; --i) {
        uint8_t c = (0xF & val);
        if(10 <= c) {
            uart_debug_buffer_temp[i] = (c - 10) + 'A';
        } else {
            uart_debug_buffer_temp[i] = c + '0';
        }
        val >>= 4;
    }
    uart_debug_add_to_buffer(uart_debug_buffer_temp, 18);
}

void uart_debug_print_bool(uint32_t b) {
    if(0 == b) {
        uart_debug_send_string("False");
    } else {
        uart_debug_send_string("True");
    }
}

void uart_debug_hexdump(uint8_t* buff, uint32_t len) {
    if(len > (DEBUG_UART_TRANSMIT_BUFFER_LEN / 3)) {
        len = DEBUG_UART_TRANSMIT_BUFFER_LEN / 3;
    }
    uint32_t blockSize = UART_DEBUG_HEXDUMP_BLOCK_SIZE; // for spaces between bytes
    uart_debug_buffer_temp[(UART_DEBUG_HEXDUMP_BLOCK_SIZE * 3) - 1] = '\n'; //preload this
    uint32_t numBlocks = len / blockSize;
    uint32_t lastBlockSize = len % blockSize;
    for(uint32_t k = 0; k < (blockSize - 1); ++k) { //for our inter-byte spaces
        uart_debug_buffer_temp[(k * 3) + 2] = ' '; //preload these as well as they won't change
    }
    if(lastBlockSize > 0) {
        numBlocks++;
    } else {
        lastBlockSize = blockSize;
    }
    for(uint32_t i = 0; i < numBlocks; i++) {
        uint32_t blockStart = i * blockSize;
        if(i == (numBlocks - 1)) {
            //last block
            blockSize = lastBlockSize;
        }
        for(uint32_t j = 0; j < blockSize; ++j) {
            uint8_t c1 = (0xF0 & buff[blockStart + j]) >> 4;
            uint8_t c2 = (0x0F & buff[blockStart + j]);
            if(10 <= c1) {
                uart_debug_buffer_temp[j * 3] = (c1 - 10) + 'A';
            } else {
                uart_debug_buffer_temp[j * 3] = c1 + '0';
            }
            if(10 <= c2) {
                uart_debug_buffer_temp[(j * 3) + 1] = (c2 - 10) + 'A';
            } else {
                uart_debug_buffer_temp[(j * 3) + 1] = c2 + '0';
            }
        }
        if(i != (numBlocks - 1)) {//if we're not on the last block
            uart_debug_add_to_buffer(uart_debug_buffer_temp, (blockSize * 3));
        }
        else { //on the last block, ignore the trailing space/newline
            uart_debug_add_to_buffer(uart_debug_buffer_temp, (blockSize * 3) - 1);
        }
    }
    uart_debug_newline(); //add final newline
}

/* puts a newline out cause i'm lazy*/
void uart_debug_newline(void) {
#ifdef UART_DEBUG_OSX //UART_DEBUG_OSX defined in uart_debug.h
	uint8_t carriageChar = '\r';
	uart_debug_add_to_buffer(&carriageChar, 1);
#endif
    uint8_t newlineChar = '\n';
    uart_debug_add_to_buffer(&newlineChar, 1);
}



/* puts a newline-terminated character string on the wire (limited to the transmit buffer length for sanity) */
void uart_debug_send_line(char* str) {
    uint32_t len = 0;
    for(; len < DEBUG_UART_TRANSMIT_BUFFER_LEN; ++len) {
        if(str[len] == '\n') {
            break;
        }
    }
    uart_debug_add_to_buffer((uint8_t*) str, len + 1); /* to account for the newline char*/

#ifdef UART_DEBUG_OSX //UART_DEBUG_OSX defined in uart_debug.h
    uart_debug_newline();
#endif




}

/* puts a null-terminated character string on the wire (limited to the transmit buffer length for sanity) */
void uart_debug_send_string(char* str) {
    uint32_t len = 0;
    for(; len < DEBUG_UART_TRANSMIT_BUFFER_LEN; ++len) {
        if(str[len] == '\0') {
            break;
        }
    }
    //HAL_UART_Transmit_IT(debugUART, (uint8_t*) str, len);
    uart_debug_add_to_buffer((uint8_t*) str, len);
}


/* adds stuff to the debug buffer to be sent out.
 * returns the number of bytes added to the buffer
 * if the amount of data is too big, does not add the bit that won't fit in the buffer
 */
uint32_t uart_debug_add_to_buffer(uint8_t* in, uint32_t len) {
    uint8_t a = 0;
    a += 1;
    if(len > uart_debug_buffer_remaining) {
        len = uart_debug_buffer_remaining; /*toss bit that won't fit*/
    }
    uint32_t remainingLen = (len + uart_debug_buffer_head);
    if(DEBUG_UART_TRANSMIT_BUFFER_LEN > remainingLen) {
        /* not overflowing the end of the buffer*/
        memcpy(uart_debug_buffer + uart_debug_buffer_head, in, len);
        uart_debug_buffer_head += len;
    } else {
        /*overflowing end of buffer, wrap around*/
        uint32_t endLen = DEBUG_UART_TRANSMIT_BUFFER_LEN - uart_debug_buffer_head;
        memcpy(uart_debug_buffer + uart_debug_buffer_head, in, endLen);
        uint32_t startLen = len - endLen; /*what goes at the start of the buffer*/
        memcpy(uart_debug_buffer, in + endLen, startLen);
        uart_debug_buffer_head = startLen;
    }
    uart_debug_buffer_remaining -= len; /* update this*/
    if(!uart_debug_busy_flag) {
        uart_debug_transmit(); /* start things*/
    }
    return len;
}

void uart_debug_putchar(unsigned char in) {
    uart_debug_add_to_buffer(&in, 1);
}

/* internal helper function to handle the actual HAL call*/
void uart_debug_transmit(void) {
	if(debugUART != NULL)
	{
		uint32_t head = uart_debug_buffer_head;
		if(head < uart_debug_buffer_tail) {
			/* we've wrapped around
			 * head is at the end and we'll have to do another HAL call later
			 */
			head = DEBUG_UART_TRANSMIT_BUFFER_LEN;
		}
		uint32_t len = head - uart_debug_buffer_tail;
#if (ENABLE_UART_DMA_CACHE_MAINTENANCE == 1)
            /*
               the SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
               adjust the address and the D-Cache size to invalidate accordingly.
             */
            uint32_t alignedAddr;
            alignedAddr = (uint32_t)(uart_debug_buffer + uart_debug_buffer_tail) & ~0x1F;
            SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, len);
#endif


		HAL_UART_Transmit_DMA(debugUART, uart_debug_buffer + uart_debug_buffer_tail, len);
		//HAL_UART_Transmit(debugUART, uart_debug_buffer + uart_debug_buffer_tail, len, 1000); //try blocking IO for now
		//HAL_UART_Transmit_IT(debugUART, uart_debug_buffer + uart_debug_buffer_tail, len);
		uart_debug_busy_flag = 1;
		/*update the transmit head*/
		uart_debug_buffer_transmit_head = head;
	}
}


/*void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
 * moved the callback to main, that callback then calls this function
 * */
void uart_debug_callback(void) {
    uart_debug_buffer_remaining += (uart_debug_buffer_transmit_head - uart_debug_buffer_tail);
    if(DEBUG_UART_TRANSMIT_BUFFER_LEN <= uart_debug_buffer_transmit_head) {
        uart_debug_buffer_transmit_head = 0; //wrap this
    }
    uart_debug_buffer_tail = uart_debug_buffer_transmit_head; /*update tail*/
    if(uart_debug_buffer_head == uart_debug_buffer_transmit_head) {
        /*nothing new has been added, so nothing to do!*/
        uart_debug_busy_flag = 0;
    } else {
        uart_debug_transmit();
    }
}
/* dumps a byte to the serial in binary format
 * handy for things like the status or access registers
 */
void uart_debug_dump_byte(uint8_t b) {
    uint32_t len = 11;
    uart_debug_buffer_temp[0] = '0';
    uart_debug_buffer_temp[1] = 'b';
    uint8_t mask = 0x80;
    for(uint32_t i = 2; i < 10; ++i) {
        if(mask & b) {
            uart_debug_buffer_temp[i] = '1';
        } else {
            uart_debug_buffer_temp[i] = '0';
        }
        mask >>= 1;
    }
    uart_debug_buffer_temp[10] = '\n';
    uart_debug_add_to_buffer(uart_debug_buffer_temp, len);
}

/* dumps memory to the debug UART*/
//moved here from main.c because it was never called
//but might prove useful. Its entirely possible this function
//can be deleted entirely
void uart_debug_mem_dump(uint8_t* start, size_t end) {
    size_t pos = 0;
    size_t chunk = 256; //how many bytes to dump at once before delaying so the UART can catch up
    uint32_t delay = 60; /* roughly how long it should take to dump a chunk*/
    while(pos < end) {
        if((end - pos) < chunk) {
            chunk = end - pos; //last chunk
        }
        uart_debug_send_string("MemDump: Pos ");
        uart_debug_hexprint32((uint32_t)start + pos);
        uart_debug_send_string(" Len ");
        uart_debug_hexprint32(chunk);
        uart_debug_newline();
        uart_debug_hexdump(start + pos, chunk);
        pos += chunk;
        HAL_Delay(delay);
    }
}

void uart_debug_tx_complete_callback(DMA_HandleTypeDef *_hdma) {
	uart_debug_callback();
}

#endif /* UART_DEBUG_C */


