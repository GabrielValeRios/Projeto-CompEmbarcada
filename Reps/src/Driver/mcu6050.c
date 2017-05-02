/*
 * mcu6050.c
 *
 * Created: 26/04/2017 20:14:41
 * Author: Rafael Corsi, Gabriela Almeida e Gabriel Rios
 */ 

/**
 *  Computação Embarcada
 *  Insper
 *  Rafael Corsi - rafael.corsi@insper.edu.br
 *  Código exemplo uso I2C
 *  Abril - 2017
 *
 *  Bug conhecido : spi read deve ser executado duas vezes ?
 *      - funcao afetada : mcu6050_i2c_bus_read()
 *
 *  Conectar :
 *            MCU  |  SAME70-XPLD | 
 *           ----------------------
 *            SDA  |   EXT2-11    |  
 *            SCL  |   EXT2-12    |
 *            GND  |   EXT2-19    |
 *            VCC  |   EXT2-20    | 
 * 
 * ref [1] http://playground.arduino.cc/Main/MPU-6050#short
 * ref [2] https://github.com/jarzebski/Arduino-MPU6050/blob/master/MPU6050.cpp
 */

/**
 * inclui o head do mcu6050
 */
#include "mcu6050.h"
#include "asf.h"
/*	
 *  \Brief: The function is used as I2C bus init
 */
void mcu6050_i2c_bus_init(void)
{
	twihs_options_t mcu6050_option;
	pmc_enable_periph_clk(TWIHS_MCU6050_ID);

	/* Configure the options of TWI driver */
	mcu6050_option.master_clk = sysclk_get_cpu_hz();
	mcu6050_option.speed      = 40000;
	twihs_master_init(TWIHS_MCU6050, &mcu6050_option);
}

/*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t ierror = 0x00;

	twihs_packet_t p_packet;
	p_packet.chip         = dev_addr;
	p_packet.addr[0]      = reg_addr;
	p_packet.addr_length  = 1;
	p_packet.buffer       = reg_data;
	p_packet.length       = cnt;
	
	ierror = twihs_master_write(TWIHS_MCU6050, &p_packet);

	return (int8_t)ierror;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t ierror = 0x00;
	
	twihs_packet_t p_packet;
	p_packet.chip         = dev_addr;
	p_packet.addr[0]      = reg_addr;
	p_packet.addr_length  = 1;
	p_packet.buffer       = reg_data;
	p_packet.length       = cnt;
	
  // TODO: Algum problema no SPI faz com que devemos ler duas vezes o registrador para
  //       conseguirmos pegar o valor correto.
	ierror = twihs_master_read(TWIHS_MCU6050, &p_packet);
	ierror = twihs_master_read(TWIHS_MCU6050, &p_packet);

	return (int8_t)ierror;
}