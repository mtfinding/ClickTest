/* ========================================================================== *
 * FileName: IO_control.c
 * Description: Library for IO interactions with GPIO-Expander: TI tca9555
 * Author: Matthias Timo Finding
 * Creation Date: Oct 6, 2023
 * Modified Date: Feb 10, 2024
 * Version: 1
 * ========================================================================== */

#include <main.h>
#include <stdbool.h>
#include <stdio.h>
#include <IO_control.h>

/**
  * @brief: convert from node-bitpattern to one-hot-code
  * for I/O expander registers (refer IO_control.h: 69)
  * @note: selecting single pin on port expander requires one-hot-code
  * for upper and lower register respectively
  * @param: targeted node
  * @retval: one-hot-code for port-expander write operation
  */
uint8_t get_one_hot(uint8_t node_address) {

	// get expander pin number from last 3(2-0) bits -> n
    uint8_t expander_node_id = node_address & 0b00000111;
    return 0x01 << expander_node_id;
}

/**
  * @brief: get I2C device address from node-bitpattern
  * @note: for write_register() and read_register() functions
  * @param: targeted node
  * @retval: I2C device address
  */
uint8_t get_device_address(uint8_t node_address){

	// shift device addr so A0 = LSB and remove rest
	uint8_t device_address = (node_address >> 4) & 0b00000111;

	// bring to expected format for tca9555
	return (0b00100000 | device_address) << 1;
}

/**
  * @brief: write port expander register
  * auto-selects required I2C port depending on address
  * @param: addr: address(any node of targeted device)
  * @param: reg: register to write
  * @param: data: data to write
  * @retval: I2C write operation success
  */
_Bool write_register(uint8_t node_address, uint8_t reg, uint8_t data){

	uint8_t device_address = get_device_address(node_address);
	uint8_t packet[2] = {reg, data};

	// select correct I2C instance depending on MSB of pin-bit-pattern
	I2C_HandleTypeDef *I2C_PORT = (node_address & 0x80) ? &hi2c3 : &hi2c1;

	if(HAL_I2C_Master_Transmit(	I2C_PORT,
								device_address,
								packet,
								2,
								I2C_DEF_TIMEOUT) == HAL_OK){
		return true;
	} else {
		return false;
	}
}

/**
  * @brief: read port expander register
  * @param: address(any node of targeted device) and register
  * @retval: register value
  */
uint8_t read_register(uint8_t node_address, uint8_t reg){

	uint8_t device_address = get_device_address(node_address);
	uint8_t return_data;

	// select correct I2C instance depending on MSB of pin-bit-pattern
	I2C_HandleTypeDef *I2C_PORT = (node_address & 0x80) ? &hi2c3 : &hi2c1;

	HAL_I2C_Master_Transmit(I2C_PORT, device_address, &reg, 1, I2C_DEF_TIMEOUT);
	HAL_I2C_Master_Receive(I2C_PORT, device_address, &return_data, 1, I2C_DEF_TIMEOUT);

	return return_data;
}

/**
  * @brief: Write function for addressing individual nodes
  * @param: target node and states: IO_LOW or IO_HIGH
  * @retval: none
  * @note: targets output-register of port expander
  */
_Bool write_node(node_t node, uint8_t state){

	_Bool retVal;

	// read current register contents
	uint8_t old_reg_value[2] = {read_register(node.addr, REG_OUTPUT0),
		read_register(node.addr, REG_OUTPUT1)};

	// generate bitmask of tagrgeted pin
	uint8_t bitmask = get_one_hot(node.addr);

	// select upper or lower register (bit n=3 of node.addr)
	if (node.addr & 0x08){
		uint8_t new_reg_value = (old_reg_value[1] & ~bitmask) | (state & bitmask);
		retVal = write_register(node.addr, REG_OUTPUT1, new_reg_value);
	} else {
		uint8_t new_reg_value = (old_reg_value[0] & ~bitmask) | (state & bitmask);
		retVal = write_register(node.addr, REG_OUTPUT0, new_reg_value);
	}

	return retVal;
}

/**
  * @brief: Read function for reading individual nodes
  * @param: node to read
  * @retval: IO_HIGH or IO_LOW
  */
uint8_t read_node(node_t node){

	// select upper or lower register (bit n=3 of node.addr)
	uint8_t register_address = (node.addr & 0x08) ? REG_INPUT1 : REG_INPUT0;

	// read IO status and compare with bitmask
	if(read_register(node.addr, register_address) & get_one_hot(node.addr)){
		return IO_HIGH;
	} else {
		return IO_LOW;
	}
}

/**
  * @brief: Configuration function for addressing individual nodes
  * @param: target node and states: IO_HIZ or IO_OE
  * @retval: operation success
  * @note: targets configuration-register of port expander
  */
_Bool configure_node(node_t node, uint8_t state){

	_Bool retVal;

	// read current register contents
	uint8_t old_reg_value[2] = {read_register(node.addr, REG_CONFIG0),
		read_register(node.addr, REG_CONFIG1)};

	// generate bitmask of tagrgeted pin
	uint8_t bitmask = get_one_hot(node.addr);

	// select upper or lower register (bit n=3 of node.addr)
	if (node.addr & 0x08){
		uint8_t new_reg_value = (old_reg_value[1] & ~bitmask) | (state & bitmask);
		retVal = write_register(node.addr, REG_CONFIG1, new_reg_value);
	} else {
		uint8_t new_reg_value = (old_reg_value[0] & ~bitmask) | (state & bitmask);
		retVal = write_register(node.addr, REG_CONFIG0, new_reg_value);
	}

	return retVal;
}

/**
  * @brief: Set given node to output-enable and low-state
  * @param: target node
  * @retval: none
  */
void node_output_low(node_t node){
	write_node(node, IO_LOW);
	configure_node(node, IO_OE);
}

/**
  * @brief: Set given node to output-enable and high-state
  * @param: target node
  * @retval: none
  */
void node_output_high(node_t node){
	write_node(node, IO_HIGH);
	configure_node(node, IO_OE);
}

/**
  * @brief: Set given node high-impedance and high (reset state)
  * @param: target node
  * @retval: none
  */
void node_release(node_t node){
	configure_node(node, IO_HIZ);
	write_node(node, IO_HIGH);
}

/**
  * @brief: Set busy-LED output
  * @param: state: LED_ON or LED_OFF
  * @retval: none
  */
void set_busy_led(int state){
	switch(state){
		case LED_ON:
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);
			break;
		case LED_OFF:
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
			break;
		default:
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
			break;
	}
}

/**
  * @brief: Set error-LED output
  * @param: state: LED_ON or LED_OFF
  * @retval: none
  */
void set_error_led(int state){
	switch(state){
		case LED_ON:
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, SET);
			break;
		case LED_OFF:
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, RESET);
			break;
		default:
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, RESET);
			break;
	}
}

/**
  * @brief: resets all registers to reset-values
  * @param: none
  * @retval: none
  */
void reset_io(){

	// loop through address-space of GPIO Expanders(0b00000000 to 0b11110000)
	for(uint8_t i = 0; i < I2C_ADDR_CNT; i++){

		// address I2C bus I and II and all contained devices (refer IO_control.h: 69)
		uint8_t target_expander = (0b00000000 + (i << 4));

		// set all register values to reset-value
		write_register(target_expander, REG_OUTPUT0, RST_ALL_ONE);
		write_register(target_expander, REG_OUTPUT1, RST_ALL_ONE);
		write_register(target_expander, REG_POLINV0, RST_ALL_ZERO);
		write_register(target_expander, REG_POLINV1, RST_ALL_ZERO);
		write_register(target_expander, REG_CONFIG0, RST_ALL_ONE);
		write_register(target_expander, REG_CONFIG1, RST_ALL_ONE);
	}
}

/**
  * @brief: resets all registers to reset-values and restore Nucleo enable signal
  * @param: none
  * @retval: none
  * @note: created due to common use-case
  */
void reset_io_selective(){
	reset_io();
	node_output_low(MB_DUT_ENA);
}
