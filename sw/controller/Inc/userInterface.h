/* ========================================================================== *
 * FileName: userInterface.h
 * Description: Functions for user interaction with test-programs through terminal
 * Author: Matthias Timo Finding
 * Creation Date: Jan 26, 2024
 * Modified Date: Jan 26, 2024
 * Version: 1
 * ========================================================================== */

#ifndef INC_USERINTERFACE_H_
#define INC_USERINTERFACE_H_

/* --------------------------------- Defines -------------------------------- */
#define UI_NO_OP		0xFF    // for future use
#define UI_EXEC_NUC		0x01    // execute nucleo test
#define UI_EXEC_CB		0x02    // execute click-board test
#define UI_EXEC_ALL		0x03    // execute all tests
#define UART_RX_LEN     20      // UART rx buf length

/* -------------------------- function prototypes --------------------------- */
uint8_t button_handler(void);
void print_header(void);
uint8_t parse_input(char *user_input);
void show_fail_on_cb(node_t *failed_node);

// sorted nodes for show_fail_on_cb
// sorted array of nodes corresponding to terminal display
static const node_t UNUSED_NODE = {"UNUSED_NODE", 0b00000000};

static const node_t sorted_nodes_cb[] = {	
	MB_T_GND1, MB_T_D13, MB_T_D12, MB_T_D11,
	MB_T_A4, MB_T_3V3_1, MB_T_5V_1,

	MB_T_D0, MB_T_D1, MB_T_3V3_2, MB_T_GND_2,
	MB_T_5V_2, MB_T_D4, MB_T_D5,

	CB_L_A0, CB_L_D6, CB_C_D1, UNUSED_NODE, CB_R_A1, CB_R_A4,
	CB_L_D7, CB_L_A2, CB_C_D0, CB_C_GND_1, CB_R_D8, CB_R_A3,
	CB_L_D3, CB_L_D0, UNUSED_NODE, UNUSED_NODE, CB_R_A5, CB_R_D0,
	CB_L_D13, CB_L_D1, CB_C_GND_2, UNUSED_NODE, CB_R_D13, CB_R_D1,
	CB_L_D12, CB_L_D5, CB_C_D2, CB_C_A7, CB_R_D12, CB_R_D5,
	CB_L_D11, CB_L_D4, CB_C_D3, CB_C_A6, CB_R_D11, CB_R_D4,
	CB_L_3V3, CB_L_5V, CB_C_D4, CB_C_A5, CB_R_3V3, CB_R_5V,
	CB_L_GND_1, CB_L_GND_2, CB_C_D5, CB_C_A4, CB_R_GND_1, CB_R_GND_2,

	CB_C_D6, CB_C_A3, CB_C_D7, CB_C_A2, CB_C_D8, CB_C_A1,
	CB_C_D9, CB_C_A0, CB_C_D10, UNUSED_NODE, CB_C_D11, CB_C_3V3,
	CB_C_D12, CB_C_D13
	};
			

#endif /* INC_USERINTERFACE_H_ */
