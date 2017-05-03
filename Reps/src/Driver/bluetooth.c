/*
 * mcu6050.c
 *
 * Created: 26/04/2017 19:49:34
 *  Author: Rafael Corsi, Gabriela Almeida e Gabriel Rios
 */ 

/**
 * inclui o head do bluetooth
 */
#include "bluetooth.h"
#include "asf.h"

/**
 * \brief Configure UART console.
 */
 void USART0_init(void){
  
	/* Configura USART0 Pinos */
	sysclk_enable_peripheral_clock(ID_PIOB);
	pio_set_peripheral(PIOB, PIO_PERIPH_C, PIO_PB0);
	pio_set_peripheral(PIOB, PIO_PERIPH_C, PIO_PB1);
  
	/* Configura opcoes USART */
	const sam_usart_opt_t usart_settings = {
		.baudrate     = 9600,
		.char_length  = US_MR_CHRL_8_BIT,
		.parity_type  = US_MR_PAR_NO,
		.stop_bits    = US_MR_NBSTOP_1_BIT,
		.channel_mode = US_MR_CHMODE_NORMAL
	};

	/* Ativa Clock periferico USART0 */
	sysclk_enable_peripheral_clock(ID_USART0);
  
	/* Configura USART para operar em modo RS232 */
	usart_init_rs232(USART0, &usart_settings, sysclk_get_peripheral_hz());
  
	/* Enable the receiver and transmitter. */
		usart_enable_tx(USART0);
		usart_enable_rx(USART0);
 }
 

 /**
 *  Envia para o UART uma string
 */
uint32_t usart_putString(uint8_t *pstring){
	uint32_t i = 0 ;

	while(*(pstring + i)){
		usart_serial_putchar(USART0, *(pstring+i++));
		while(!uart_is_tx_empty(USART0)){};
	}  
     
	return(i);
}


/*
 * Busca do UART uma string
 */
uint32_t usart_getString(uint8_t *pstring){
	uint32_t i = 0 ;
	
	usart_serial_getchar(USART0, (pstring+i));
	while(*(pstring+i) != '\n'){
		usart_serial_getchar(USART0, (pstring+(++i)));
	}
	*(pstring+i+1)= 0x00;
	return(i);
}