/* ========================================================================== *
 * FileName: IO_test.h
 * Description: Header file for IO_test.c
 * Author: Matthias Timo Finding
 * Creation Date: Nov 16, 2023
 * Modified Date: Feb 9, 2024
 * Version: 1
 * ========================================================================== */

#ifndef INC_IO_TEST_H_
#define INC_IO_TEST_H_

/* ---------------------------- HAL handle import --------------------------- */
extern ADC_HandleTypeDef hadc1;

/* --------------------------------- Defines -------------------------------- */
#define CB_NET_CNT 			    20		// # nets on click expander board
#define CB_TEST_CNT 			4		// # test functions for click-board
#define DUT_NODE_CNT 			18		// # nets on Nucleo board
#define ADC_TEST_SAMPLES 		50		// # samples to verify ADC/POT range
#define ADC_TEST_DURATION 		5000	// ADC test duration in milliseconds
#define ADC_TEST_THRESHHOLD 	3500	// minimum succesfull ADC range (4096 max)

/* -------------------------- function prototypes --------------------------- */
_Bool test_net_cb(const net_t *net);
_Bool cb_net_test();
_Bool cb_led_test();
_Bool cb_pushbutton_test();
_Bool cb_potentiometer_test();
_Bool cb_master_test();
_Bool nuc_master_test();
_Bool test_all();

#endif /* INC_IO_TEST_H_ */
