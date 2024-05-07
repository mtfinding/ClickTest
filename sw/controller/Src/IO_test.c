/* ========================================================================== *
 * FileName: IO_test.c
 * Description: IO tests performed via IO_control library for NUCLEO Board as
 * well as Click adapter board.
 * Author: Matthias Timo Finding
 * Creation Date: Nov 16, 2023
 * Modified Date: Feb 9, 2024
 * Version: 1
 * ========================================================================== */

#include <main.h>
#include <stdbool.h>
#include <stdio.h>
#include <IO_control.h>
#include <IO_test.h>

/**
  * @brief: algorithm for clickboard net-test
  * @note: each node on net has weak pull-up, net is tested by
  * pulling first node to LOW and checking other nodes
  * @param: pointer to net-under-test
  * @retval: boolean if successful
  */
_Bool test_net_cb(const net_t *net){

	printf("testing net: %s...", net->name);

	switch(net->nodecount){

		// handle invalid net lengths
		case 0:
			printf("invalid net of length 0\r\n");
			return false;
		break;

		case 1:
			printf("invalid net length\r\n");
			return false;
		break;

		// handle two-point-nets
		case 2:
			// pull first node low
			node_output_low(*net->node[0]);

			// check second node
			if(read_node(*net->node[1]) == IO_HIGH){
				printf("fail at node: %s or %s\r\n",
				net->node[0]->name,
				net->node[1]->name);
				node_release(*net->node[0]);
				return false;
			}

			node_release(*net->node[0]);
			printf("pass.\r\n");
			return true;
		break;

		// handle nets > 2
		default:
			// pull first node low
			node_output_low(*net->node[0]);

			for(uint8_t i = 0; i < net->nodecount; i++){

				// test if node is HIGH indicating electrical discontinuity
				if(read_node(*net->node[i]) == IO_HIGH){

					// handle break-at-injection-node exception
					if(i == 1){

						// release old injection node
						node_release(*net->node[0]);

						// set node 1 to new injection node
						node_output_low(*net->node[1]);

						// re-test with new injection node
						for(uint8_t i = 0; i < net->nodecount; i++){
							if(i == 1) continue;
							if(read_node(*net->node[i]) == IO_LOW){
								printf("fail at node: %s\r\n",
									net->node[0]->name);
								show_fail_on_cb(net->node[0]);
								return false;
							}
						}

						// reset injection point to node 0
						node_release(*net->node[1]);
						node_output_low(*net->node[0]);
					}

					node_release(*net->node[0]);
					printf("fail at node: %s\r\n", net->node[i]->name);
					show_fail_on_cb(net->node[i]);
					return false;
				}
			}

			node_release(*net->node[0]);
			printf("pass.\r\n");
			return true;

		break;
	}

}

/**
  * @brief: tests all nets of click expander board
  * @param: none
  * @retval: boolean if successful
  */
_Bool cb_net_test(){

	printf("\r\n------------------------------------------------\r\n");
	printf("IO Test:\r\n");

	const net_t *cb_nets[CB_NET_CNT] = {
			&A0, &A1, &A2, &A3,
			&A4, &A5, &D0, &D1,
			&D3, &D4, &D5, &D6,
			&D7, &D8, &D11, &D12,
			&D13, &V3, &V5, &GND
	};

	reset_io_selective();

	for(uint8_t i = 0; i < CB_NET_CNT; i++){

		if(!test_net_cb(cb_nets[i])){
			set_error_led(LED_ON);
			reset_io_selective();
			printf("IO Test NOK\r\n");
			return false;
		}
		// delay for better terminal-readability
		HAL_Delay(50);
	}

	reset_io_selective();
	printf("IO Test OK\r\n");
	return true;
}

/**
  * @brief: LED test routine
  * @param: none
  * @retval: boolean (always true: user-verified test)
  */
_Bool cb_led_test(){

	printf("\r\n------------------------------------------------\r\n");
	printf("LED Test:\r\n");
	HAL_Delay(500);

	node_t led_cathodes[3] = {CB_C_A5, CB_C_D9, CB_C_A3};
	char led_channels[3][6] = {"red", "green", "blue"};

	// set 3V3 high for common-anode rgb-LED
	node_output_high(CB_C_3V3);

	for(uint8_t i = 0; i < 3; i++){
		printf("testing LED - %s channel\r\n", led_channels[i]);
		node_output_low(led_cathodes[i]);
		HAL_Delay(1000); // time for user to verify LED visually
		node_release(led_cathodes[i]);
	}

	reset_io_selective();
	return true;
}

/**
  * @brief: Pushbutton test routine
  * @param: none
  * @retval: boolean if successful
  */
_Bool cb_pushbutton_test(){

	printf("\r\n------------------------------------------------\r\n");
	printf("Pushbutton Test:\r\n");
	printf("please press and hold pushbutton\r\n");

	// pull floating GND of click-expander-board to system GND
	node_output_low(CB_C_GND_1);

	// 5s timeframe for user to push button
	for(uint8_t i = 0; i < 10; i++){

		if(read_node(CB_C_A2) == IO_LOW){
			node_release(CB_C_GND_1);
			printf("Pushbutton Test OK\r\n");
			return true;
		}

		HAL_Delay(500);
	}

	set_error_led(LED_ON);
	node_release(CB_C_GND_1);
	printf("Pushbutton Test NOK\r\n");
	return false;

}

/**
  * @brief: Potentiometer test routine
  * @param: none
  * @retval: boolean if successful
  */
_Bool cb_potentiometer_test(){

	printf("\r\n------------------------------------------------\r\n");
	printf("Potentiometer Test:\r\n");
	HAL_Delay(500);
	printf("please rotate potentiometer through entire range\r\n");

	uint16_t adc_samples[ADC_TEST_SAMPLES];
	uint16_t max_val = 2048;
	uint16_t min_val = 2048;

	// supply click-board GND and 3V3 nets via port-expanders
	node_output_low(CB_C_GND_1);
	node_output_high(CB_C_3V3);

	// collect ADC samples while user rotates potentiometer
	for(uint8_t i = 0; i < ADC_TEST_SAMPLES; i++){

		// read ADC
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		adc_samples[i] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		// get min/max values
		if(adc_samples[i] < min_val) min_val = adc_samples[i];
		if(adc_samples[i] > max_val) max_val = adc_samples[i];

		// set sample-rate
		HAL_Delay(ADC_TEST_DURATION / ADC_TEST_SAMPLES);
	}

	reset_io_selective();

	// evaluate potentiometer range
	if(max_val - min_val > ADC_TEST_THRESHHOLD){
		printf("Potentiometer Test OK\r\n");
		return true;

	} else{
		set_error_led(LED_ON);
		printf("Potentiometer Test NOK\r\n");
		return false;
	}
}

/**
  * @brief: master test scheduling function for click-board
  * @param: none
  * @retval: boolean if successful
  */
_Bool cb_master_test(){

	// test function pointers
	_Bool (*test_functions[CB_TEST_CNT])(void) = {
			cb_net_test, cb_led_test, cb_pushbutton_test, cb_potentiometer_test
	};

	set_busy_led(LED_ON);

	// execute all tests
	for(uint8_t i = 0; i < CB_TEST_CNT; i++){

		if(!test_functions[i]()){
			set_busy_led(LED_OFF);
			return false;
		}
	}

	set_busy_led(LED_OFF);
	return true;

}

/**
  * @brief: master test scheduling function for Nucleo-Board
  * @param: none
  * @retval: boolean if successful
  */
_Bool nuc_master_test(){

	set_busy_led(LED_ON);

	printf("\r\n------------------------------------------------\r\n");
	printf("NUCLEO DUT Test:\r\n");

	node_t dut_nodes[DUT_NODE_CNT] = {
			MB_DUT_D0, MB_DUT_D1, MB_DUT_D2, MB_DUT_D3,
			MB_DUT_D4, MB_DUT_D5, MB_DUT_D6, MB_DUT_D9,
			MB_DUT_D10, MB_DUT_D11, MB_DUT_D12, MB_DUT_D13,
			MB_DUT_A1, MB_DUT_A2, MB_DUT_A3, MB_DUT_A4,
			MB_DUT_A5, MB_DUT_A6};


	// enable DUT power & wait for boot
	node_output_high(MB_DUT_ENA);
	HAL_Delay(1000);

	// check if all DUT pins are pulled LOW
	for(uint8_t i = 0; i < DUT_NODE_CNT; i++){
		
		if(read_node(dut_nodes[i]) == IO_LOW){
			printf("node: '%s' OK\r\n", dut_nodes[i].name);
		} else {
			printf("FAIL at node: '%s'\r\n", dut_nodes[i].name);
			set_error_led(LED_ON);
			node_output_low(MB_DUT_ENA);
			return false;
		}
		// delay for better terminal-radability
		HAL_Delay(50);
	}

	node_output_low(MB_DUT_ENA);
	printf("NUCLEO DUT Test OK\r\n");
	set_busy_led(LED_OFF);
	return true;

}

/**
  * @brief: master test scheduling function for all tests
  * @param: none
  * @retval: boolean if successful
  */
_Bool test_all(){

	// test function pointer array
	_Bool (*test_functions[2])(void) = {
			cb_master_test, nuc_master_test
	};

	// execute all tests
	for(uint8_t i = 0; i < 2; i++){

		if(!test_functions[i]()){
			return false;
		}
	}

	return true;
}
