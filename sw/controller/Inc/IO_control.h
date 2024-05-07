/* ========================================================================== *
 * FileName: IO_control.h
 * Description: Header file for IO_control.c
 * Author: Matthias Timo Finding
 * Creation Date: Oct 6, 2023
 * Modified Date: Feb 9, 2024
 * Version: 1
 * ========================================================================== */

#ifndef INC_IO_CONTROL_H_
#define INC_IO_CONTROL_H_

/* ---------------------------- type declarations --------------------------- */
typedef struct node_t {
	char name[12];
	uint8_t addr;
} node_t;

typedef struct net_t {
	char name[4];
	uint8_t nodecount;
    const node_t **node;
} net_t;

/* ---------------------------- HAL handle import --------------------------- */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;

/* ---------------------------- IO state defines ---------------------------- */
#define IO_LOW			0x00	// IO Low State, use for write_node()
#define IO_HIGH			0xFF	// IO High State, use for write_node()
#define IO_HIZ			0xFF	// IO High Impedance state, for configure_node()
#define IO_OE			0x00	// IO Output enable state, for configure_node()
#define LED_ON			0x00	// LED on-state
#define LED_OFF			0x01	// LED off-state
#define I2C_ADDR_CNT	16		// # of addressable gpio expanders
#define RST_ALL_ZERO	0x00	// reset all pins to 0
#define RST_ALL_ONE		0xFF	// reset all pins to 1

/* ---------------------------- register defines ---------------------------- */
#define REG_INPUT0		0x00	// lower input register
#define REG_INPUT1		0x01	// upper input register
#define REG_OUTPUT0		0x02	// lower output register
#define REG_OUTPUT1		0x03	// upper output register
#define REG_POLINV0		0x04	// lower polarity inversion register
#define REG_POLINV1		0x05	// upper polarity inversion register
#define REG_CONFIG0		0x06	// lower configuration register
#define REG_CONFIG1		0x07	// upper configuration register

/* ----------------------------- other defines ------------------------------ */
#define I2C_DEF_TIMEOUT 10		// 10ms I2C timeout

/* -------------------------- function prototypes --------------------------- */
uint8_t get_one_hot(uint8_t nodeAddress);
uint8_t get_device_address(uint8_t nodeAddress);
_Bool write_register(uint8_t nodeAddress, uint8_t reg, uint8_t data);
uint8_t read_register(uint8_t nodeAddress, uint8_t reg);
_Bool configure_node(node_t node, uint8_t state);
_Bool write_node(node_t node, uint8_t state);
uint8_t read_node(node_t node);
void node_output_low(node_t node);
void node_output_high(node_t node);
void node_release(node_t node);
void reset_io();
void reset_io_selective();
void set_error_led(int state);
void set_busy_led(int state);

/* ------------------------------- node defines ------------------------------ *
 * Nodes consist of Node-Name & 8-bit Address, see node_t datatype
 *
 * Naming convention:
 * CB_L/R/C: ClickBoard Left/Right/Center -> pins on Click Boards
 * MB_T: MotherBoard Top -> pins connecting to Click Expander top pins
 * MB_DUT: MotherBoard Device Under Test -> pins connecting to NUCLEO
 * NC: Not Connected
 * 
 * Address convention:
 * b7(MSB) sets I2C port: 0 -> hi2c1 | 1 -> hi2c3
 * b6..4 set I/O expander Address: 000 to 111
 * b3..0 set pin on I/O expander: 0000 to 1111 (16 pins per expander)
 *
 * MSB	0	0	1	0	1	0	0	1	LSB
 *	   BUS  | —ADDR— |	| — PIN # — |
 *	   		A2  A1  A0
 * 
 */

// Pins: ClickBoard - Left, designator on PCB: U101
static const node_t CB_L_A0 = {"CB_L_A0", 0b01100000};
static const node_t CB_L_D7 = {"CB_L_D7", 0b01100001};
static const node_t CB_L_D3 = {"CB_L_D3", 0b01100010};
static const node_t CB_L_D13 = {"CB_L_D13", 0b01100011};
static const node_t CB_L_D12 = {"CB_L_D12", 0b01100100};
static const node_t CB_L_D11 = {"CB_L_D11", 0b01100101};
static const node_t CB_L_3V3 = {"CB_L_3V3", 0b01100110};
static const node_t CB_L_GND_1 = {"CB_L_GND_1", 0b01100111};
static const node_t CB_L_GND_2 = {"CB_L_GND_2", 0b01101000};
static const node_t CB_L_5V = {"CB_L_5V", 0b01101001};
static const node_t CB_L_D4 = {"CB_L_D4", 0b01101010};
static const node_t CB_L_D5 = {"CB_L_D5", 0b01101011};
static const node_t CB_L_D1 = {"CB_L_D1", 0b01101100};
static const node_t CB_L_D0 = {"CB_L_D0", 0b01101101};
static const node_t CB_L_A2 = {"CB_L_A2", 0b01101110};
static const node_t CB_L_D6 = {"CB_L_D6", 0b01101111};

// Pins: ClickBoard -Right, designator on PCB: U201
static const node_t CB_R_A1 = {"CB_R_A1", 0b00010000};
static const node_t CB_R_D8 = {"CB_R_D8", 0b00010001};
static const node_t CB_R_A5 = {"CB_R_A5", 0b00010010};
static const node_t CB_R_D13 = {"CB_R_D13", 0b00010011};
static const node_t CB_R_D12 = {"CB_R_D12", 0b00010100};
static const node_t CB_R_D11 = {"CB_R_D11", 0b00010101};
static const node_t CB_R_3V3 = {"CB_R_3V3", 0b00010110};
static const node_t CB_R_GND_1 = {"CB_R_GND_1", 0b00010111};
static const node_t CB_R_GND_2 = {"CB_R_GND_2", 0b00011000};
static const node_t CB_R_5V = {"CB_R_5V", 0b00011001};
static const node_t CB_R_D4 = {"CB_R_D4", 0b00011010};
static const node_t CB_R_D5 = {"CB_R_D5", 0b00011011};
static const node_t CB_R_D1 = {"CB_R_D1", 0b00011100};
static const node_t CB_R_D0 = {"CB_R_D0", 0b00011101};
static const node_t CB_R_A3 = {"CB_R_A3", 0b00011110};
static const node_t CB_R_A4 = {"CB_R_A4", 0b00011111};

// Pins: ClickBoard - Center, designator on PCB: U301
static const node_t CB_C_D1 = {"CB_C_D1", 0b01010000};
static const node_t CB_C_D0 = {"CB_C_D0", 0b01010001};
static const node_t CB_C_RST_1 = {"CB_C_RST_1", 0b01010010};
static const node_t CB_C_GND_1 = {"CB_C_GND_1", 0b01010011};
static const node_t CB_C_D2 = {"CB_C_D2", 0b01010100};
static const node_t CB_C_D3 = {"CB_C_D3", 0b01010101};
static const node_t CB_C_D4 = {"CB_C_D4", 0b01010110};
static const node_t CB_C_D5 = {"CB_C_D5", 0b01010111};
static const node_t CB_C_A4 = {"CB_C_A4", 0b01011000};
static const node_t CB_C_A5 = {"CB_C_A5", 0b01011001};
static const node_t CB_C_A6 = {"CB_C_A6", 0b01011010};
static const node_t CB_C_A7 = {"CB_C_A7", 0b01011011};
static const node_t CB_C_5V = {"CB_C_5V", 0b01011100};
static const node_t CB_C_RST_2 = {"CB_C_RST_2", 0b01011101};
static const node_t CB_C_GND_2 = {"CB_C_GND_2", 0b01011110};
static const node_t CB_C_VIN = {"CB_C_VIN", 0b01011111};


// designator on PCB: U302
static const node_t CB_C_D6 = {"CB_C_D6", 0b00110000};
static const node_t CB_C_D7 = {"CB_C_D7", 0b00110001};
static const node_t CB_C_D8 = {"CB_C_D8", 0b00110010};
static const node_t CB_C_D9 = {"CB_C_D9", 0b00110011};
static const node_t CB_C_D10 = {"CB_C_D10", 0b00110100};
static const node_t CB_C_D11 = {"CB_C_D11", 0b00110101};
static const node_t CB_C_D12 = {"CB_C_D12", 0b00110110};
static const node_t CB_C_NC1 = {"CB_C_NC1", 0b00110111};
static const node_t CB_C_D13 = {"CB_C_D13", 0b00111000};
static const node_t CB_C_3V3 = {"CB_C_3V3", 0b00111001};
static const node_t CB_C_AREF = {"CB_C_AREF", 0b00111010};
static const node_t CB_C_A0 = {"CB_C_A0", 0b00111011};
static const node_t CB_C_A1 = {"CB_C_A1", 0b00111100};
static const node_t CB_C_A2 = {"CB_C_A2", 0b00111101};
static const node_t CB_C_A3 = {"CB_C_A3", 0b00111110};
static const node_t CB_C_NC2 = {"CB_C_NC2", 0b00111111};

// Pins: MotherBoard - TopPins, designator on PCB: U401
static const node_t MB_T_GND1 = {"MB_T_GND1", 0b00100000};
static const node_t MB_T_D13 = {"MB_T_D13", 0b00100001};
static const node_t MB_T_D12 = {"MB_T_D12", 0b00100010};
static const node_t MB_T_D11 = {"MB_T_D11", 0b00100011};
static const node_t MB_T_A4 = {"MB_T_A4", 0b00100100};
static const node_t MB_T_3V3_1 = {"MB_T_3V3_1", 0b00100101};
static const node_t MB_T_5V_1 = {"MB_T_5V_1", 0b00100110};
static const node_t MB_T_NC1 = {"MB_T_NC1", 0b00100111};
static const node_t MB_T_D0 = {"MB_T_D0", 0b00101000};
static const node_t MB_T_D1 = {"MB_T_D1", 0b00101001};
static const node_t MB_T_3V3_2 = {"MB_T_3V3_2", 0b00101010};
static const node_t MB_T_GND_2 = {"MB_T_GND_2", 0b00101011};
static const node_t MB_T_5V_2 = {"MB_T_5V_2", 0b00101100};
static const node_t MB_T_D4 = {"MB_T_D4", 0b00101101};
static const node_t MB_T_D5 = {"MB_T_D5", 0b00101110};
static const node_t MB_T_NC2 = {"MB_T_NC2", 0b00101111};

// Pins: MotherBoard - DUT pins, designator on PCB: U601
static const node_t MB_DUT_NC1 = {"MB_DUT_NC1", 0b10000000};
static const node_t MB_DUT_NC2 = {"MB_DUT_NC2", 0b10000001};
static const node_t MB_DUT_NC3 = {"MB_DUT_NC3", 0b10000010};
static const node_t MB_DUT_NC4 = {"MB_DUT_NC4", 0b10000011};
static const node_t MB_DUT_D12 = {"MB_DUT_D12", 0b10000100};
static const node_t MB_DUT_D11 = {"MB_DUT_D11", 0b10000101};
static const node_t MB_DUT_D10 = {"MB_DUT_D10", 0b10000110};
static const node_t MB_DUT_D9 = {"MB_DUT_D9", 0b10000111};
static const node_t MB_DUT_D6 = {"MB_DUT_D6", 0b10001000};
static const node_t MB_DUT_D5 = {"MB_DUT_D5", 0b10001001};
static const node_t MB_DUT_D4 = {"MB_DUT_D4", 0b10001010};
static const node_t MB_DUT_D3 = {"MB_DUT_D3", 0b10001011};
static const node_t MB_DUT_D2 = {"MB_DUT_D2", 0b10001100};
static const node_t MB_DUT_RST = {"MB_DUT_RST", 0b10001101};
static const node_t MB_DUT_D0 = {"MB_DUT_D0", 0b10001110};
static const node_t MB_DUT_D1 = {"MB_DUT_D1", 0b10001111};

// designator on PCB: U602
static const node_t MB_DUT_A7 = {"MB_DUT_A7", 0b10110000};
static const node_t MB_DUT_A6 = {"MB_DUT_A6", 0b10110001};
static const node_t MB_DUT_A5 = {"MB_DUT_A5", 0b10110010};
static const node_t MB_DUT_A4 = {"MB_DUT_A4", 0b10110011};
static const node_t MB_DUT_A3 = {"MB_DUT_A3", 0b10110100};
static const node_t MB_DUT_A2 = {"MB_DUT_A2", 0b10110101};
static const node_t MB_DUT_A1 = {"MB_DUT_A1", 0b10110110};
static const node_t MB_DUT_A0 = {"MB_DUT_A0", 0b10110111};
static const node_t MB_DUT_D13 = {"MB_DUT_D13", 0b10111000};
static const node_t MB_DUT_NC5 = {"MB_DUT_NC5", 0b10111001};
static const node_t MB_DUT_NC6 = {"MB_DUT_NC6", 0b10111010};
static const node_t MB_DUT_NC7 = {"MB_DUT_NC7", 0b10111011};
static const node_t MB_DUT_NC8 = {"MB_DUT_NC8", 0b10111100};
static const node_t MB_DUT_NC9 = {"MB_DUT_NC9", 0b10111101};
static const node_t MB_DUT_NC10 = {"MB_DUT_NC10", 0b10111110};
static const node_t MB_DUT_ENA = {"MB_DUT_ENA", 0b10111111};


/* ------------------------------- net defines------------------------------- *
 * nets consist of: netname, number of nodes & pointer to array of nodes
 */

// data-nets
static const node_t *A0_nodes[] = {&CB_L_A0, &CB_C_A0};
static const net_t A0  = {"A0",  2, A0_nodes};

static const node_t *A1_nodes[] = {&CB_R_A1, &CB_C_A1};
static const net_t A1  = {"A1",  2, A1_nodes};

static const node_t *A2_nodes[] = {&CB_L_A2, &CB_C_A2};
static const net_t A2  = {"A2",  2, A2_nodes};

static const node_t *A3_nodes[] = {&CB_R_A3, &CB_C_A3};
static const net_t A3  = {"A3",  2, A3_nodes};

static const node_t *A4_nodes[] = {&CB_R_A4, &CB_C_A4, &MB_T_A4};
static const net_t A4  = {"A4",  3, A4_nodes};

static const node_t *A5_nodes[] = {&CB_R_A5, &CB_C_A5};
static const net_t A5  = {"A5",  2, A5_nodes};

static const node_t *D0_nodes[] = {&CB_L_D0, &CB_R_D0, &CB_C_D0, &MB_T_D0};
static const net_t D0  = {"D0",  4, D0_nodes};

static const node_t *D1_nodes[] = {&CB_L_D1, &CB_R_D1, &CB_C_D1, &MB_T_D1};
static const net_t D1  = {"D1",  4, D1_nodes};

static const node_t *D3_nodes[] = {&CB_L_D3, &CB_C_D3};
static const net_t D3  = {"D3",  2, D3_nodes};

static const node_t *D4_nodes[] = {&CB_L_D4, &CB_R_D4, &CB_C_D4, &MB_T_D4};
static const net_t D4  = {"D4",  4, D4_nodes};

static const node_t *D5_nodes[] = {&CB_L_D5, &CB_R_D5, &CB_C_D5, &MB_T_D5};
static const net_t D5  = {"D5",  4, D5_nodes};

static const node_t *D6_nodes[] = {&CB_L_D6, &CB_C_D6};
static const net_t D6  = {"D6",  2, D6_nodes};

static const node_t *D7_nodes[] = {&CB_L_D7, &CB_C_D7};
static const net_t D7  = {"D7",  2, D7_nodes};

static const node_t *D8_nodes[] = {&CB_R_D8, &CB_C_D8};
static const net_t D8  = {"D8",  2, D8_nodes};

static const node_t *D11_nodes[] = {&CB_L_D11, &CB_R_D11, &CB_C_D11, &MB_T_D11};
static const net_t D11 = {"D11", 4, D11_nodes};

static const node_t *D12_nodes[] = {&CB_L_D12, &CB_R_D12, &CB_C_D12, &MB_T_D12};
static const net_t D12 = {"D12", 4, D12_nodes};

static const node_t *D13_nodes[] = {&CB_L_D13, &CB_R_D13, &CB_C_D13, &MB_T_D13};
static const net_t D13 = {"D13", 4, D13_nodes};

// Power nets
static const node_t *V3_nodes[] = {&CB_L_3V3, &CB_R_3V3, &CB_C_3V3, &MB_T_3V3_1,
 									&MB_T_3V3_2};
static const net_t V3  = {"3V3", 5, V3_nodes};

static const node_t *V5_nodes[] = {&CB_L_5V, &CB_R_5V, &CB_C_5V, &MB_T_5V_1,
									&MB_T_5V_2};
static const net_t V5  = {"5V",  5, V5_nodes};

static const node_t *GND_nodes[] = {&CB_L_GND_1, &CB_L_GND_2, &CB_R_GND_1,
									&CB_R_GND_2, &CB_C_GND_1, &CB_C_GND_2,
									&MB_T_GND1, &MB_T_GND_2};
static const net_t GND = {"GND", 8, GND_nodes};
								
#endif
