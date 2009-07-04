#include <stdio.h>
#include <avr/io.h>

#include "usrat.h"

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_buffer_in;
static volatile uint8_t rx_buffer_out;

//! \brief Initialize USART, perform fdevopen with uart_putchar.
//! \param baudval (F_CPU/(16*baudrate))-1
//! \sa uart_putchar()
void usart_init(uint16_t baudval) {
	// Set baud rate
	UBRR0H = (uint8_t)(baudval>>8);
	UBRR0L = (uint8_t)baudval;

	rx_buffer_in = rx_buffer_out = 0;

	// Set frame format: 8 data, 1 stop bit
	UCSR0C = (uint8_t)((00<<UMSEL00) | (0<<USBS0) | (3<<UCSZ00));
	
	// Enable receiver and transmitter, enable RX complete interrupt
	UCSR0B = (uint8_t)((1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0));

	(void)fdevopen(uart_putchar, uart_getchar/*, 0*/);
}

static void cts(uint8_t pause) {
	if (pause) PORTD |= _BV(5);
	else 	    PORTD &= ~_BV(5);
}


//! \brief putchar() for USART.
//! \param data character to print.
int uart_putchar(char data) {
	//while (!(UCSR0A & (1<<UDRE0))) {};
	if (data == '\n') {
		(void)uart_putchar('\r');
	}

	while (!(UCSR0A & (1<<UDRE0))) {};

	UDR0 = (uint8_t)data;

	return 0;
}

//! \brief getchar() for USART. Wait for data if not available.
//! \return value read.
//! \sa uart_available()
int uart_getchar() {
	while (!uart_available());
	
	return (int) uart_getc();
}

//! \brief Check data availability in USART buffer.
//! \return 1 if buffer is not empty.
uint8_t uart_available() {
	return rx_buffer_in != rx_buffer_out;
}

//! \brief Nonblocking, nonchecking getchar for USART. Use with care.
uint8_t uart_getc() {
	uint8_t result = rx_buffer[rx_buffer_out];
	rx_buffer_out = (rx_buffer_out + 1) % RX_BUFFER_SIZE;
	if (!uart_available()) cts(0);
	
	return result;
}

void SIG_USART_RECV( void ) __attribute__ ( ( signal ) );  
void SIG_USART_RECV( void ) {
	rx_buffer[rx_buffer_in] = (uint8_t)UDR0;
	rx_buffer_in = (rx_buffer_in + 1) % RX_BUFFER_SIZE;
	cts(1);
	
}

// $Id: usrat.c,v 1.3 2006/05/30 09:03:52 svo Exp $
