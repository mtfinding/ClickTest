/* ========================================================================== *
 * FileName: userInterface.c
 * Description: Functions for user interaction with test-programs through terminal
 * Author: Matthias Timo Finding
 * Creation Date: Jan 26, 2024
 * Modified Date: Jan 26, 2024
 * Version: 1
 * ========================================================================== */


#include <main.h>
#include <IO_control.h>
#include <userInterface.h>
#include <string.h>
#include <stdio.h>

// global variable for mapped button-action
uint8_t UI_BUTTON_ACTION = UI_EXEC_ALL;

/**
  * @brief: returns mapped button-action
  * @param: none
  * @retval: button-action
  */
uint8_t button_handler(void){
    return UI_BUTTON_ACTION;
}

/**
  * @brief: print header for terminal
  * @param: none
  * @retval: none
  */
void print_header(void){
    printf("\r\n");
    printf("=====================================================================\r\n");
    printf("=                      Nucleo & Click-Shield                        =\r\n");
    printf("=                           --TESTER--                              =\r\n");
    printf("=                       by Matthias Finding                         =\r\n");
    printf("=====================================================================\r\n");
    printf("\r\n");
    printf("enter 'help' for available commands\r\n");
}

/**
  * @brief: terminal-input parsing function & button-action mapper
  * @param: pointer to string to parse
  * @retval: next function to execute or no-operation
  * @note: commands: help, test, map
  * @note: arguments for test / map: -a: all, -n: nucleo, -c: click-board
  */
uint8_t parse_input(char *user_input){
    
        // check if input is empty
        if(user_input[0] == '\0'){
            printf("no input detected\r\n");
            return UI_NO_OP;
        }
    
        /*-------------------------*
         * check for help command  *
         *-------------------------*/
        if(strncmp(user_input, "help", 4) == 0){

            printf("\r\navailable commands:\r\n");
            printf("test -a: execute all test functions\r\n");
            printf("test -n: execute all nucleo test functions\r\n");
            printf("test -c: execute all click-board test functions\r\n");
            printf("map -a: map USER-button to execute all test functions\r\n");
            printf("map -n: map USER-button to execute all nucleo test functions\r\n");
            printf("map -c: map USER-button to execute all click-board test functions\r\n");
            
            return UI_NO_OP;

        }

        /*-------------------------*
         * check for test command  *
         *-------------------------*/
        if(strncmp(user_input, "test", 4) == 0){
    
            // test -a
            if(strncmp(user_input, "test -a", 7) == 0){
                return UI_EXEC_ALL;
            }
    
            // test -n
            if(strncmp(user_input, "test -n", 7) == 0){
                return UI_EXEC_NUC;
            }
    
            // test -c
            if(strncmp(user_input, "test -c", 7) == 0){
                return UI_EXEC_CB;
            }
    
            // no argument
            printf("no argument given\r\n");
            return UI_NO_OP;
        }

        /*-------------------------*
         * check for map command   *
         *-------------------------*/
        if(strncmp(user_input, "map", 3) == 0){

            // map -a
            if(strncmp(user_input, "map -a", 6) == 0){
                printf("USER-button mapped to execute all tests\r\n");
                UI_BUTTON_ACTION = UI_EXEC_ALL;
                return UI_NO_OP;
            }

            // map -n
            if(strncmp(user_input, "map -n", 6) == 0){
                printf("USER-button mapped to execute Nucleo-tests\r\n");
                UI_BUTTON_ACTION = UI_EXEC_NUC;
                return UI_NO_OP;
            }

            // map -c
            if(strncmp(user_input, "map -c", 6) == 0){
                printf("USER-button mapped to execute Click-tests\r\n");
                UI_BUTTON_ACTION = UI_EXEC_CB;
                return UI_NO_OP;
            }

            // no argument
            printf("no argument given\r\n");
            return UI_NO_OP;
        }
    
        // invalid input
        printf("invalid input\r\n");
        return UI_NO_OP;
}

/**
  * @brief: print image of pin-configuration for click expander to ease search
  * @param: node to display
  * @retval: none
  */
void show_fail_on_cb(node_t *failed_node){

	printf("\r\n ");
	
    // print top-nodes (2x7 nodes)
	for(int i = 0; i < 7; i++){
        (sorted_nodes_cb[i].addr != failed_node->addr) ? printf("o") : printf("x");
	}

	printf("                                 ");
	
	for(int i = 0; i < 7; i++){
        (sorted_nodes_cb[i+7].addr != failed_node->addr) ? printf("o") : printf("x");
	}

	printf("\r\n\n");

	// print "upper block" (8x6 nodes)
	for(int i = 0; i < 8; i++){
        printf("%s       %s          %s        %s           %s       %s",
        (sorted_nodes_cb[14+(i*6)+0].addr != failed_node->addr) ? "o" : "x",
        (sorted_nodes_cb[14+(i*6)+1].addr != failed_node->addr) ? "o" : "x",
        (sorted_nodes_cb[14+(i*6)+2].addr != failed_node->addr) ? "o" : "x",
        (sorted_nodes_cb[14+(i*6)+3].addr != failed_node->addr) ? "o" : "x",
        (sorted_nodes_cb[14+(i*6)+4].addr != failed_node->addr) ? "o" : "x",
        (sorted_nodes_cb[14+(i*6)+5].addr != failed_node->addr) ? "o" : "x"
        );
		printf("\r\n");
	}

	// print "lower block" (7x2 nodes)
	for(int i = 0; i < 7; i++){
        printf("                   %s        %s",
        (sorted_nodes_cb[62+(i*2)+0].addr != failed_node->addr) ? "o" : "x",
        (sorted_nodes_cb[62+(i*2)+1].addr != failed_node->addr) ? "o" : "x"
        );
		printf("\r\n");
	}
	
	printf("\r\n");
}
