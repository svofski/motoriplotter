#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include "usrat.h"

static FILE* input;
static int input_eof = 0;

int getchar()
{
    return uart_getchar();
}

void usart_init(uint16_t baudrate)
{
    input = fopen("test.hpgl", "r");
    if (input == NULL) {
        perror("test.hpgl");
        exit(1);
    }

}

int uart_putchar(char data, FILE* stream)
{
    return putchar(data);
}

int uart_getchar()
{
    while (!uart_available());
    return (int) uart_getc();
}

int avail_cycle = 0;
const int input_pace = 10;

uint8_t uart_available(void) 
{
    //usleep(10);
    if (++avail_cycle == input_pace) {
        avail_cycle = 0;
        //printf("available: %d\n", !input_eof);
        return !input_eof;
    }
    return 0;
}

uint8_t uart_getc()
{
    int nextchar = fgetc(input);
    if (nextchar == EOF) {
        printf("EOF");
        input_eof = 1;
    }
    return nextchar;
}


