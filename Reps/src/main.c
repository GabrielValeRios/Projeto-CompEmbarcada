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

#define SIZE 318

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
	
	//printf("%d \n", new_modulo -30);
	
	const float cnst[SIZE] = {
  0.0001062008514067,3.992636122779e-05,4.718849174738e-05,5.515910370238e-05,
  6.382300158706e-05, 7.32341169778e-05, 8.33730123784e-05,9.427241224763e-05,
  0.0001058678278493, 0.000118176814793,0.0001311577479105,0.0001448788028538,
  0.0001592470995807, 0.000174220493185,0.0001896535048419,0.0002057384132422,
  0.0002221658311149,0.0002389220614867,0.0002560355266974,0.0002732619738786,
  0.0002906188657044,0.0003078999117747,0.0003250512080742,0.0003418969263823,
  0.0003583295308822,0.0003741531658336,0.0003892448360943,0.0004034260838489,
  0.0004165335983111,0.0004283442030157,0.0004387199259275,0.0004474557329946,
  0.0004543101752137,0.0004591373996124,0.0004616828623483,0.0004617632360603,
  0.0004591559398831,0.0004536439720694,0.0004450291177658,0.0004330987675811,
  0.0004176538596948,0.0003984842232723,0.0003754183566277,0.0003482660390281,
  0.0003168528734637,0.0002810107748758, 0.000240618418852,0.0001955064949335,
  0.0001455851411031,9.074100431604e-05,3.089605325655e-05,-3.399859018973e-05,
  -0.0001039923788134,-0.0001790870656125,-0.0002592717465512,-0.0003445002851284,
  -0.0004347129781129,-0.0005298065425439,-0.000629650591173,-0.0007340890216803,
  -0.0008429364610517,-0.0009559585373712,-0.001072904732339,-0.001193485613143,
  -0.001317366132764,-0.001444202002538,-0.001573580918922,-0.001705088032135,
  -0.001838255681052, -0.00197259212252,-0.002107566041572,-0.002242623465546,
  -0.002377177903473,-0.002510609096438,-0.002642273880026,-0.002771506165538,
  -0.002897610889397,-0.003019870713044,-0.003137558109403,-0.003249912202378,
  -0.003356177694961,-0.003455570904465,-0.003547305389479,-0.003630594822987,
  -0.003704640734098,-0.003768653574368,-0.003821839494625,-0.003863424370261,
  -0.003892633177803,-0.003908712996633,-0.003910926067167,-0.003898561269516,
  -0.003870925260186,-0.003827364974015,-0.003767249056817,-0.003689991402717,
  -0.00359504571731,-0.003481900121085,-0.003350103158782, -0.00319924264499,
  -0.003028967118997,-0.002838974778039,-0.002629028611665,-0.002398951337034,
  -0.002148627860711,-0.001878012036404,-0.001587124795475,-0.001276056997882,
  -0.0009449705974711,-0.0005941015691578,-0.0002237528294484,0.0001656868969115,
  0.0005737700606036,0.0009999635818317, 0.001443666947856, 0.001904204818575,
  0.002380830236623, 0.002872728143495, 0.003379009662608, 0.003898724467134,
  0.004430853654708, 0.004974319028346, 0.005527980843378, 0.006090646240616,
  0.006661065772716, 0.007237948244217, 0.007819949693594, 0.008405690894036,
  0.008993757506997, 0.009582698311206,  0.01017104318427,   0.0107572923642,
  0.01133993835669,  0.01191745526829,  0.01248831647179,   0.0130509926592,
  0.01360396166481,  0.01414571081654,  0.01467474536812,  0.01518959078751,
  0.01568880222475,  0.01617096781002,  0.01663470966701,  0.01707870192637,
  0.01750165808024,  0.01790235362352,  0.01827961629512,  0.01863234064561,
  0.01895948848913,  0.01926009052178,  0.01953325630065,  0.01977817120904,
  0.01999410537107,  0.02018041109179,  0.02033653088247,  0.02046199347945,
  0.02055642475495,  0.02061953593085,  0.02065114002835,  0.02065114002835,
  0.02061953593085,  0.02055642475495,  0.02046199347945,  0.02033653088247,
  0.02018041109179,  0.01999410537107,  0.01977817120904,  0.01953325630065,
  0.01926009052178,  0.01895948848913,  0.01863234064561,  0.01827961629512,
  0.01790235362352,  0.01750165808024,  0.01707870192637,  0.01663470966701,
  0.01617096781002,  0.01568880222475,  0.01518959078751,  0.01467474536812,
  0.01414571081654,  0.01360396166481,   0.0130509926592,  0.01248831647179,
  0.01191745526829,  0.01133993835669,   0.0107572923642,  0.01017104318427,
  0.009582698311206, 0.008993757506997, 0.008405690894036, 0.007819949693594,
  0.007237948244217, 0.006661065772716, 0.006090646240616, 0.005527980843378,
  0.004974319028346, 0.004430853654708, 0.003898724467134, 0.003379009662608,
  0.002872728143495, 0.002380830236623, 0.001904204818575, 0.001443666947856,
  0.0009999635818317,0.0005737700606036,0.0001656868969115,-0.0002237528294484,
  -0.0005941015691578,-0.0009449705974711,-0.001276056997882,-0.001587124795475,
  -0.001878012036404,-0.002148627860711,-0.002398951337034,-0.002629028611665,
  -0.002838974778039,-0.003028967118997, -0.00319924264499,-0.003350103158782,
  -0.003481900121085, -0.00359504571731,-0.003689991402717,-0.003767249056817,
  -0.003827364974015,-0.003870925260186,-0.003898561269516,-0.003910926067167,
  -0.003908712996633,-0.003892633177803,-0.003863424370261,-0.003821839494625,
  -0.003768653574368,-0.003704640734098,-0.003630594822987,-0.003547305389479,
  -0.003455570904465,-0.003356177694961,-0.003249912202378,-0.003137558109403,
  -0.003019870713044,-0.002897610889397,-0.002771506165538,-0.002642273880026,
  -0.002510609096438,-0.002377177903473,-0.002242623465546,-0.002107566041572,
  -0.00197259212252,-0.001838255681052,-0.001705088032135,-0.001573580918922,
  -0.001444202002538,-0.001317366132764,-0.001193485613143,-0.001072904732339,
  -0.0009559585373712,-0.0008429364610517,-0.0007340890216803,-0.000629650591173,
  -0.0005298065425439,-0.0004347129781129,-0.0003445002851284,-0.0002592717465512,
  -0.0001790870656125,-0.0001039923788134,-3.399859018973e-05,3.089605325655e-05,
  9.074100431604e-05,0.0001455851411031,0.0001955064949335, 0.000240618418852,
  0.0002810107748758,0.0003168528734637,0.0003482660390281,0.0003754183566277,
  0.0003984842232723,0.0004176538596948,0.0004330987675811,0.0004450291177658,
  0.0004536439720694,0.0004591559398831,0.0004617632360603,0.0004616828623483,
  0.0004591373996124,0.0004543101752137,0.0004474557329946,0.0004387199259275,
  0.0004283442030157,0.0004165335983111,0.0004034260838489,0.0003892448360943,
  0.0003741531658336,0.0003583295308822,0.0003418969263823,0.0003250512080742,
  0.0003078999117747,0.0002906188657044,0.0002732619738786,0.0002560355266974,
  0.0002389220614867,0.0002221658311149,0.0002057384132422,0.0001896535048419,
  0.000174220493185,0.0001592470995807,0.0001448788028538,0.0001311577479105,
  0.000118176814793,0.0001058678278493,9.427241224763e-05, 8.33730123784e-05,
  7.32341169778e-05,6.382300158706e-05,5.515910370238e-05,4.718849174738e-05,
  3.992636122779e-05,0.0001062008514067
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

	if(modulo_filtered >= 116430 & modulo_filtered <= 129820){
		
		
		printf("%d;\n", modulo_filtered);
		
	}else{

		printf("%d\n", 0);
		
	}	

	pin_toggle(LED_PIO, LED_PIN_MASK);
	//delay_ms(100);
		
	sprintf(bufferTX, "%d \n", dist);
	
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
