/**
 * Copyright
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file main.c
 * @author
 * @date June 22
 * @version 1.0
 *
 * @brief Controls remotely a FRDM-K64F development board.
 *
 * The code initializes several hardware peripherals of the FRDM-K64F board
 * and allows to interact with them remotely.
 * The board gains its ability to work remotely through the usage
 * of the lwIP library using its TCP/IP stack.
 *
 * @see https://www.nxp.com/support/developer-resources/evaluation-and-development-boards/freedom-development-boards/mcu-boards/freedom-development-platform-for-kinetis-k64-k63-and-k24-mcus:FRDM-K64F
 * @see https://savannah.nongnu.org/projects/lwip/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"

#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"

#include "FreeRTOS.h"
#include "message_buffer.h"

#include "lwip/opt.h"
#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "ethernetif.h"

#include "lcd.h"
#include "led.h"
#include "pwm.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

//#include "Comunicacion.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** Media access control address used for interface configuration. */
#define MAC_ADDRESS {0x02, 0x12, 0x13, 0x10, 0x15, 0x36}
#define TCP_PORT 1234U /**< Listening port. */
#define ROW_TOP 0U /**< Top row of the LCD. */
#define ROW_BOTTOM 1U /**< Bottom row of the LCD. */
#define BUFFER_LENGTH 1400 /**< Bytes of data in TCP packets. */
#define UART_COMMAND "uart"
#define TEMP_COMMAND "temp"

typedef unsigned char uint8_t;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void stack_init_thread(void * arg);
void tcp_listener_thread(void * arg);
void uart_Task(void * arg);
void temp_Task(void * arg);
void rx_command_check(char * cmd, char * nMot, char * vel);
/*******************************************************************************
 * Variables
 ******************************************************************************/

MessageBufferHandle_t uart_handler, temp_handler; /**< Buffer handler of uart_Task. */
uint8_t messageBuffer[BUFFER_LENGTH]; /**< Buffer to be used by the threads. */
TaskHandle_t tcp_listener_handle; /**< Task handle of tcp_listener_thread. */
err_t TCP_WRITE_FLAG_COPY;
ip4_addr_t ipaddr;
char buffer[BUFFER_LENGTH];
char * ip;
char * puerto;
char * temp;
char * cmd;
char * func;
char * var;

/*******************************************************************************
 * Code
 ******************************************************************************/
/**
 * @brief Initialize the hardware and create the tasks that use that HW.
 *
 * Initialize the hardware using the MCUXpresso Config Tools generated code.
 * This code allows to mux the microcontroller pins, gate the necessary clock
 * signals and configure the enabled peripherals.
 *
 * @return EXIT_FAILURE if the task scheduler doesn't start.
 */
int main(void)
{
	/* Init board hardware. */

	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	/* Disable Memory Protection Unit. */
	SYSMPU->CESR &= ~SYSMPU_CESR_VLD_MASK;

	/* LCD initialization. */
	LCD_Init(0x3F, 16, 2, LCD_5x8DOTS);

	/* Creation of the message buffers. */
	uart_handler = xMessageBufferCreate(BUFFER_LENGTH);
	temp_handler = xMessageBufferCreate(BUFFER_LENGTH);

	/* Tasks creation. */
	if (xTaskCreate(stack_init_thread, "stack_init_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, NULL) != pdPASS)
	{
		PRINTF("stack_init_thread creation failed.\n");
	}

	if (xTaskCreate(tcp_listener_thread, "tcp_listener_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, &tcp_listener_handle) != pdPASS)
	{
		PRINTF("tcp_listener_thread creation failed.\n");
	}
	vTaskSuspend(tcp_listener_handle);

	if (xTaskCreate(uart_Task, "Uart_Task",
				DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO - 1, NULL) != pdPASS)
	{
		PRINTF("Task UART task creation failed!.\r\n");
	}
	if (xTaskCreate(temp_Task, "Temp_Task",
				DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO - 1, NULL) != pdPASS)
	{
		PRINTF("Task TEMP task creation failed!.\r\n");
	}
	vTaskStartScheduler();

	return EXIT_FAILURE;
}

void temp_Task(void * arg){

	size_t xReceivedBytes;
	const TickType_t xBlockTime = pdMS_TO_TICKS(20);
	PRINTF("temp");
	for(;;){
		//temp = func;
		xReceivedBytes = xMessageBufferReceive(temp_handler, (void *) buffer,
				sizeof(buffer), xBlockTime);

		if (xReceivedBytes)
		{
			PRINTF("TEMPERATURA");
			turn_on_red();
			vTaskDelay(1000/portTICK_RATE_MS);
			turn_off_leds();
			turn_on_red();

			LCD_clear_row(ROW_TOP);
			LCD_printstr("Temp actual:");
			LCD_clear_row(ROW_BOTTOM);
			LCD_printstr((char *)temp);
			vTaskDelay(1500/portTICK_RATE_MS);
		}
	}

}


/**
 * @brief Look for commands in char buffers.
 *
 * The first characters are compared against the current available commands,
 * if the comparison is successful a message is send to the specific thread.
 *
 * @param[in] buffer pointer to the char buffer.
 * @param[in] null_terminator_pos position in the buffer of the null terminator.
 */
/*void rx_command_check(char * cmd, char * nMot, char * vel)
{

	if (strcmp(cmd, UART_COMMAND) == 0)
	{
		xMessageBufferSend(uart_handler, (void *) buffer, strlen(buffer), 0);
	}else{
		PRINTF("Invalid command.\n");
	}
}*/


void uart_Task(void *arg)  //comando-> uart/Mode/0x01/ || uart/Vel1/0xff/ || uart/GVel1/
{
	PRINTF("uart");
	char buffer[BUFFER_LENGTH];
	uint8_t sync = 0x00;
	uint8_t var31 = 0x31;
	uint8_t var32 = 0x32;
	//uint8_t var33 = 0x33; aceleracion
	uint8_t var34 = 0x34;
	uint8_t var21 = 0x21;
	uint8_t var22 = 0x22;
	size_t xReceivedBytes;
	const TickType_t xBlockTime = pdMS_TO_TICKS(20);
	uint8_t g_vel1;
	uint8_t g_vel2;

	char * config_msg;

	for(;;){
		xReceivedBytes = xMessageBufferReceive(uart_handler, (void *) buffer,
				sizeof(buffer), xBlockTime);


		if (xReceivedBytes)
		{
			turn_off_leds();
			turn_on_blue();
			PRINTF("MOTORES");
			if(strcmp(func,"Mode")==0){
				turn_on_blue();
				PRINTF("ENTRE EN MODO");
				UART_WriteBlocking(UART4, &sync,sizeof(sync));
				UART_WriteBlocking(UART4, &var34,sizeof(var34));
				UART_WriteBlocking(UART4, &var,sizeof(var));
				PRINTF("SE ENVIO EL MODO \n");

			}else if(strcmp(func,"Vel1")==0){
				turn_on_blue();
				UART_WriteBlocking(UART4, &sync,sizeof(sync));
				UART_WriteBlocking(UART4, &var31,sizeof(var31));
				UART_WriteBlocking(UART4, &var,sizeof(var));
				PRINTF("SE ENVIO LA VELOCIDAD 1\n");

			}else if(strcmp(func,"Vel2")==0){
				turn_on_blue();
				UART_WriteBlocking(UART4, &sync,sizeof(sync));
				UART_WriteBlocking(UART4, &var32,sizeof(var32));
				UART_WriteBlocking(UART4, &var,sizeof(var));
				PRINTF("SE ENVIO LA VELOCIDAD 2\n");

			}else if(strcmp(func,"GVel1")==0){
				turn_on_red();
				UART_WriteBlocking(UART4, &sync,sizeof(sync));
				UART_WriteBlocking(UART4, &var21,sizeof(var21));
				UART_ReadBlocking(UART4, &g_vel1,sizeof(g_vel1));
				PRINTF("\n SE ENVIO LA VELOCIDAD 1: %d",g_vel1);

				//WRITER
				if(g_vel1==128){
					config_msg = "g_vel/mot1/128/";
				}else if(g_vel1==0){
					config_msg = "g_vel/mot1/0/";
				}else if(g_vel1==255){
					config_msg = "g_vel/mot1/255/";
				}

				struct netconn * netconn;
				IP4_ADDR(&ipaddr, 192,168,0,24);
				netconn = netconn_new(NETCONN_TCP);
				//vTaskDelay(2500/portTICK_RATE_MS);
				netconn_connect(netconn, &ipaddr, TCP_PORT);
				netconn_write(netconn, config_msg,strlen(config_msg), TCP_WRITE_FLAG_COPY);
				netconn_delete(netconn);

			}else if(strcmp(func,"GVel2")==0){
				turn_on_red();
				UART_WriteBlocking(UART4, &sync,sizeof(sync));
				UART_WriteBlocking(UART4, &var22,sizeof(var22));
				UART_ReadBlocking(UART4, &g_vel2,sizeof(g_vel2));
				PRINTF("SE ENVIO LA VELOCIDAD 2 \n");

				//WRITER
				if(g_vel2==128){
					config_msg = "g_vel/mot2/128/";
				}else if(g_vel2==0){
					config_msg = "g_vel/mot2/0/";
				}else if(g_vel2==255){
					config_msg = "g_vel/mot2/255/";
				}
				struct netconn * netconn;
				IP4_ADDR(&ipaddr, 192,168,0,24);
				netconn = netconn_new(NETCONN_TCP);
				//vTaskDelay(2500/portTICK_RATE_MS);
				netconn_connect(netconn, &ipaddr, TCP_PORT);
				netconn_write(netconn, config_msg,strlen(config_msg), TCP_WRITE_FLAG_COPY);
				netconn_delete(netconn);
			}else{
				PRINTF("COMANDO NO VALIDO");
			}
			turn_off_leds();
		}
		/*vTaskDelay(2000/portTICK_RATE_MS);
		LCD_clear_row(ROW_TOP);
		LCD_printstr(ip);
		LCD_clear_row(ROW_BOTTOM);
		LCD_printstr(puerto);*/
	}


}


/**
 * @brief Listen for received TCP packages.
 *
 * Establish a TCP connection in a given port.
 * When a package is received,
 * the first characters are checked looking for commands.
 *
 * @param[in] arg argument to the thread.
 */
void tcp_listener_thread(void * arg)
{
	PRINTF("listener");
	struct netbuf  * netbuf;
	struct netconn * netconn;
	struct netconn * newconn;

	netconn = netconn_new(NETCONN_TCP);
	netconn_bind(netconn, NULL, TCP_PORT);
	netconn_listen(netconn);
	char port_announce[16];
	sprintf(port_announce, "TCP port %d", TCP_PORT);
	puerto = port_announce;
	LCD_clear_row(ROW_BOTTOM);
	LCD_printstr(port_announce);


	for (;;)
	{
		turn_on_green();
		netconn_accept(netconn, &newconn);
		while (ERR_OK == netconn_recv(newconn, &netbuf))
		{
			do
			{
				netbuf_copy(netbuf, buffer, sizeof(buffer));
				char delimitador[] = "/";
				char *token = strtok(buffer, delimitador);
				cmd = token;


				LCD_clear_row(ROW_TOP);
				LCD_printstr("Listening:");
				LCD_clear_row(ROW_BOTTOM);
				LCD_printstr(buffer);

				if(strcmp(cmd, UART_COMMAND) == 0){
					token = strtok(NULL, delimitador);
					func = token;
					token = strtok(NULL, delimitador);
					var = token;
					PRINTF("\n Recibi: %s, %s, %s \n",cmd,func,var);
					xMessageBufferSend(uart_handler, (void *) buffer, strlen(buffer), 0);
				}else if(strcmp(cmd, TEMP_COMMAND) == 0){
					token = strtok(NULL, delimitador);
					temp = token;
					PRINTF("\n Recibi: %s, %s \n",cmd,temp);
					xMessageBufferSend(temp_handler, (void *) buffer, strlen(buffer), 0);
				}else{
					PRINTF("\n mal");
				}

			}
			while (netbuf_next(netbuf) >= 0);

			netbuf_delete(netbuf);
		}
		netconn_delete(newconn);
		turn_off_leds();
		vTaskDelay(2000/portTICK_RATE_MS);
		LCD_clear_row(ROW_TOP);
		LCD_printstr(ip);
		LCD_clear_row(ROW_BOTTOM);
		LCD_printstr(port_announce);
	}
}


/**
 * @brief Initialize the TCP/IP stack and obtain an IP address.
 *
 * Initialize the TCP/IP stack using lwIP which is a small footprint
 * implementation of the TCP/IP protocol suite.
 * After a successful initialization of the stack, set up the network interface
 * of the board.
 * When everything is ready try to negotiate an IP address using DHCP.
 *
 * @param[in] arg argument to the thread.
 */

void stack_init_thread(void * arg)
{
	LCD_clear_row(ROW_TOP);
	LCD_printstr("Obtaining IP");
	LCD_clear_row(ROW_BOTTOM);
	LCD_printstr("Please wait...");

	struct netif netif;
	struct dhcp * dhcp;

	ip4_addr_t ip_address;
	ip4_addr_t netmask;
	ip4_addr_t gateway;

	IP4_ADDR(&ip_address, 192,168,0,23);
	IP4_ADDR(&netmask, 255,255,255,0);
	IP4_ADDR(&gateway, 192,168,0,1);


	ethernetif_config_t config =
	{
			.phyAddress = BOARD_ENET0_PHY_ADDRESS,
			.clockName = kCLOCK_CoreSysClk,
			.macAddress = MAC_ADDRESS,
	};

	// Initialization of the TCP/IP stack.
	tcpip_init(NULL, NULL);

	// Set up the network interface.
	netif_add(&netif, &ip_address, &netmask, &gateway, &config,
			ethernetif0_init, tcpip_input);
	netif_set_default(&netif);
	netif_set_up(&netif);

	// Start DHCP negotiation.
	//dhcp_start(&netif);

	for(;;)
	{
		//dhcp = netif_dhcp_data(&netif);

		//if (dhcp->state == DHCP_STATE_BOUND)
		//{

			PRINTF("IPv4 Address : %s\n", ipaddr_ntoa(&netif.ip_addr));
			PRINTF("IPv4 Netmask : %s\n", ipaddr_ntoa(&netif.netmask));
			PRINTF("IPv4 Gateway : %s\n", ipaddr_ntoa(&netif.gw));

			LCD_clear_row(ROW_TOP);
			ip = ipaddr_ntoa(&netif.ip_addr);
			LCD_printstr(ipaddr_ntoa(&netif.ip_addr));

			vTaskResume(tcp_listener_handle);
			vTaskDelete(NULL);
		//}
	}
}




