/************************************************************************
* Rafael Corsi Gabriela Almeida e Gabriel Rios   - Insper
*
* Computação Embarcada
*
* REPS
************************************************************************/

 
#include "asf.h"
#include "Driver/mcu6050.h"
#include "Driver/bluetooth.h"
#include "arm_math.h"

/**
 * LEDs
 */
#define LED_PIO_ID		  ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		      8
#define LED_PIN_MASK    (1<<LED_PIN)

/**
 * Botão
 */
#define BUT_PIO_ID            ID_PIOA
#define BUT_PIO               PIOA
#define BUT_PIN		            11
#define BUT_PIN_MASK          (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

/** 
 * 
 */
#define TWIHS_MCU6050_ID    ID_TWIHS0 
#define TWIHS_MCU6050       TWIHS0  

/** 
 *  USART
 */
#define USART_COM     USART1
#define USART_COM_ID  ID_USART1

#define FREQ		250


/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

#define SIZE 46

float rangePerDigit ; // 2G
//const float rangePerDigit = 9.80665f ; // 2G

volatile uint8_t flag_led0 = 1;

int16_t  accX, accY, accZ;
float new_accX, new_accY, new_accZ;
volatile uint8_t  accXHigh, accYHigh, accZHigh;
volatile uint8_t  accXLow,  accYLow,  accZLow;

float new_mesuarement[SIZE] = { 0 };
	
int vel = 0;
int dist = 0;

float soma = 0.0;

uint8_t bufferRX[100];
uint8_t bufferTX[100];
uint8_t rtn;
/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
void TC1_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask)
{
	pin_toggle(PIOD, (1<<28));
	pin_toggle(LED_PIO, LED_PIN_MASK);
}

void USART1_Handler(void){
	uint32_t ret = usart_get_status(USART_COM);

	// Verifica por qual motivo entrou na interrupçcao
	if(ret & US_IER_RXRDY){                     // Dado disponível para leitura
		usart_getString(bufferRX);
		} else if(ret & US_IER_TXRDY){              // Transmissão finalizada
	}
}

void TC1_Handler(void){
	// Le valor do acc X High e Low
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &accXHigh, 1);
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &accXLow,  1);
		
	// Le valor do acc y High e  Low
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &accYHigh, 1);
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &accYLow,  1);
		
	// Le valor do acc z HIGH e Low
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &accZHigh, 1);
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &accZLow,  1);
		
	// Dados são do tipo complemento de dois
	accX = (accXHigh << 8) | (accXLow << 0);
	accY = (accYHigh << 8) | (accYLow << 0);
	accZ = (accZHigh << 8) | (accZLow << 0);
	

	new_accX = (((float) (accX * accX)) * 0.000061f);
	new_accY = (((float) (accY*accY)) * 0.000061f);
	new_accZ = (((float) (accZ*accZ)) * 0.000061f);
	/*
	printf("x/y/z : %dg / %dg / %dg \n",(int)(new_accX*1000.0), 
										(int)(new_accY*1000.0), 
										(int)(new_accZ*1000.0)
										);
	*/
	float modulo;
	modulo = (sqrt(new_accX+new_accY+new_accZ));
	int new_modulo = (int)(modulo);
	
	//printf("%d; \n", new_modulo -30);
	
	const float cnst[SIZE] = {
  0.0002604725812757,0.0004681737368045,0.0008605774774129, 0.001438286092877,
  0.002244876580681,  0.00332292005687, 0.004710989705012, 0.006439733217404,
  0.008528449813216,  0.01098198045156,  0.01378780352849,  0.01691415619799,
  0.02030950732621,   0.0239028608029,  0.02760533847088,  0.03131350605626,
  0.03491387590254,  0.03828797759156,  0.04131821734586,   0.0438943134756,
  0.04591942085708,  0.04731545493717,  0.04802759584449,  0.04802759584449,
  0.04731545493717,  0.04591942085708,   0.0438943134756,  0.04131821734586,
  0.03828797759156,  0.03491387590254,  0.03131350605626,  0.02760533847088,
  0.0239028608029,  0.02030950732621,  0.01691415619799,  0.01378780352849,
  0.01098198045156, 0.008528449813216, 0.006439733217404, 0.004710989705012,
  0.00332292005687, 0.002244876580681, 0.001438286092877,0.0008605774774129,
  0.0004681737368045,0.0002604725812757
	};
	
	
	for(int j = SIZE - 1; j >= 1; j--){
		
		new_mesuarement[j] = new_mesuarement[j-1];
		
	}
	
	new_mesuarement[0] = new_modulo;
	
	
	for(int i = 0; i < SIZE; i++){
			
		soma+= cnst[i]*new_mesuarement[i];
		
	}
	
	int modulo_filtered = (int)(soma*1000.0);
	//printf("%d \n", modulo_filtered);
	soma = 0.0;
	//printf("%d;\n", modulo_filtered);

	if(modulo_filtered >= 124500){
		
		
		printf("%d;\n", modulo_filtered);
		
	}else{

		printf("%d\n", 0);
		modulo_filtered = 0;
		
	}	
	
	pin_toggle(LED_PIO, LED_PIN_MASK);
	//delay_ms(100);
		
	sprintf(bufferTX, "%d \n", modulo_filtered);
	
	usart_putString(bufferTX);

}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/** 
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
		pio_clear(pio, mask);
	else
		pio_set(pio,mask);
}

/**
 * @Brief Inicializa o pino do BUT
 */
void BUT_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
    
	/* habilita interrupçcão do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 1);
};

/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

void TC1_init(void){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC1);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(FREQ, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC0, 1, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC0, 1, (ul_sysclk / ul_div) / FREQ);

	/* Configura e ativa interrupçcão no TC canal 0 */
	NVIC_EnableIRQ((IRQn_Type) ID_TC1);
	tc_enable_interrupt(TC0, 1, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC0, 1);
}

/**
 * \brief Configure UART console.
 * BaudRate : 115200
 * 8 bits
 * 1 stop bit
 * sem paridade
 */
static void configure_console(void)
{

	/* Configura USART1 Pinos */
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOA);
	pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
 	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
 
	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}


/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
  
	/* buffer para recebimento de dados */

  
	

	/* Initialize the SAM system */
	sysclk_init();
	board_init();
   
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
  
	/* Inicializa com serial com PC*/
	configure_console();
  
	/* Configura Leds */
	LED_init(1);
  
	/* Configura os botões */
	BUT_init();  
  
	/* Configura USART0 para comunicacao com o HM-10 */
	USART0_init();
  
	/* Inicializa funcao de delay */
	delay_init( sysclk_get_cpu_hz());
   
  
	/************************************************************************/
	/* MPU                                                                  */
	/************************************************************************/
  
	sprintf(bufferTX, "%s", "AT+NAMERios");
	usart_putString(bufferTX); 
  
  
	/* Inicializa i2c */
	printf("Inicializando bus i2c \n");
	mcu6050_i2c_bus_init();
  
	// Verifica MPU
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, bufferRX, 1);
	if(rtn != TWIHS_SUCCESS){
		printf("[ERRO] [i2c] [read] \n");
	}
  
	// Por algum motivo a primeira leitura é errada.
	if(bufferRX[0] != 0x68){
		printf("[ERRO] [mcu] [Wrong device] [0x%2X] \n", bufferRX[0]);
	}
 
	// Set Clock source
	bufferTX[0] = MPU6050_CLOCK_PLL_XGYRO;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
		printf("[ERRO] [i2c] [write] \n");

	// Configura range acelerometro para operar com 2G
	bufferTX[0] = 0x00; // 2G
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, bufferTX, 1);
	rangePerDigit = 0.000061f ; // 2G 
	//uint32_t g = rangePerDigit/2;  
	
	TC1_init();    
 
	while (1) {

	}
}
