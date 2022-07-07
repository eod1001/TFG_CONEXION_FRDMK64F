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
 * @author Enrique del Olmo Dominguez
 * @date June 22
 * @version 1.0
 *
 * @brief
 * This software is the second part "Slave A" of a pilot plant consisting of 3
 * FRDM K64F boards. This Embedded System receives information through network
 * cable and TCP/IP protocols. It processes this data and communicates with the
 * LCD display via I2C communication, or with the motors via UART communication.
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

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** Media access control address used for interface configuration. */
#define MAC_ADDRESS {0x02, 0x12, 0x13, 0x10, 0x15, 0x12}
#define TCP_PORT 1234U /**< Listening port. */
#define ROW_TOP 0U /**< Top row of the LCD. */
#define ROW_BOTTOM 1U /**< Bottom row of the LCD. */
#define BUFFER_LENGTH 1400 /**< Bytes of data in TCP packets. */
#define UART_COMMAND "uart"
#define TEMP_COMMAND "temp"
#define AlertaTemp 10

typedef unsigned char uint8_t;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void init_IP(void * arg);
void tcp_listener_thread(void * arg);
void uart_Task(void * arg);
void temp_Task(void * arg);
/*******************************************************************************
 * Variables
 ******************************************************************************/
MessageBufferHandle_t uart_handler, temp_handler; /**< Buffer handler of uart_Task. */
uint8_t messageBuffer[BUFFER_LENGTH]; /**< Buffer to be used by the threads. */
TaskHandle_t tcp_listener_handle; /**< Task handle of tcp_listener_thread. */
err_t TCP_WRITE_FLAG_COPY;
ip4_addr_t ipaddr;
ip4_addr_t sendIP;

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
	if (xTaskCreate(init_IP, "init_IP",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, NULL) != pdPASS)
	{
		PRINTF("init_IP creation failed.\n");
	}

	if (xTaskCreate(tcp_listener_thread, "tcp_listener_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, &tcp_listener_handle) != pdPASS)
	{
		PRINTF("tcp_listener_thread creation failed.\n");
	}
	vTaskSuspend(tcp_listener_handle);

	if (xTaskCreate(uart_Task, "Uart_Task",
				DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO-1, NULL) != pdPASS)
	{
		PRINTF("Task UART task creation failed!.\r\n");
	}
	if (xTaskCreate(temp_Task, "Temp_Task",
				DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO-1, NULL) != pdPASS)
	{
		PRINTF("Task TEMP task creation failed!.\r\n");
	}
	vTaskStartScheduler();

	return EXIT_FAILURE;
}




/**
 * @brief Displays the temperature received from the master SE.
 *
 * It receives the command with the voltage from the master SE
 * temperature sensor. It is in charge of transforming it to degrees
 * centigrade and displaying it on the screen.
 *
 *
 * @param[in] arg argument to the thread.
 */
void temp_Task(void * arg){

	size_t xReceivedBytes;
	const TickType_t xBlockTime = pdMS_TO_TICKS(20);
	int tempC;
	int tempInt;
	for(;;){
		turn_off_leds();
		xReceivedBytes = xMessageBufferReceive(temp_handler, (void *) buffer,
				sizeof(buffer), xBlockTime);
		if (xReceivedBytes)
		{
			tempInt = strtol(temp, NULL, 10);
			tempC = 370*26/tempInt; //(((5.0 * tempInt)*100)/1024);
			int valorAux = tempC;
			int z =1;
			while(valorAux>=10){
				valorAux = valorAux/10;
				z++;
			}
			char str[z];
			int x = z;
			char valorc;
			int valor = tempC;
			while(valor!=0){
				valorc = (valor % 10) + 48;
				str[z-1] = (char)valorc;
				valor = valor/10;
				z--;
			}
			str[x] = '\0';
			char msg_temp[5] ="";
			strcat(msg_temp,str);

			LCD_clear_row(ROW_TOP);
			LCD_printstr("Temp actual:");
			LCD_clear_row(ROW_BOTTOM);

			LCD_printstr(msg_temp);
			turn_on_red();
			vTaskDelay(2000/portTICK_RATE_MS);
		}
	}
}


/**
 * @brief Communication with the motors according to the
 * information of the command received.
 *
 * It communicates with the EMG30 motors via UART. It indicates
 * the actions to be performed, according to the command received
 * by the master SE.
 *
 * @param[in] arg argument to the thread.
 */
void uart_Task(void *arg)
{
	char buffer[BUFFER_LENGTH];
	uint8_t sync = 0x00;
	uint8_t var31 = 0x31;
	uint8_t var32 = 0x32;
	uint8_t var34 = 0x34;
	uint8_t var21 = 0x21;
	uint8_t var22 = 0x22;
	uint8_t var38 = 0x38;
	size_t xReceivedBytes;
	const TickType_t xBlockTime = pdMS_TO_TICKS(20);
	uint8_t g_vel1;
	uint8_t g_vel2;
	uint8_t stop = 0x80;
	char * gvel1;
	char * gvel2;
	char * config_msg;

	for(;;){
		xReceivedBytes = xMessageBufferReceive(uart_handler, (void *) buffer,
				sizeof(buffer), xBlockTime);
		if (xReceivedBytes)
		{
			UART_WriteBlocking(UART4, &sync,sizeof(sync));
			wait_Waitms(500);
			UART_WriteBlocking(UART4, &var38,sizeof(var38));
			int var1 = strtol(var, NULL, 10);
			uint8_t var2 = (uint8_t)var1;

			if(strcmp(func,"Mode")==0){

				UART_WriteByte(UART4, sync);
				wait_Waitms(500);
				UART_WriteByte(UART4, var34);
				wait_Waitms(500);
				UART_WriteByte(UART4, var2);
				wait_Waitms(1000);

			}else if(strcmp(func,"Vel1")==0){
				LCD_clear_row(ROW_TOP);
				LCD_printstr("Fijando Vel1");
				LCD_clear_row(ROW_BOTTOM);
				LCD_printstr(var);
			    UART_WriteByte(UART4, sync);
			    wait_Waitms(500);
			    UART_WriteByte(UART4, var31);
			    wait_Waitms(500);
			    UART_WriteByte(UART4, var2);
			    wait_Waitms(1000);
				vTaskDelay(1500/portTICK_RATE_MS);

			}else if(strcmp(func,"Vel2")==0){
				LCD_clear_row(ROW_TOP);
				LCD_printstr("Fijando Vel2");
				LCD_clear_row(ROW_BOTTOM);
				LCD_printstr(var);
			    UART_WriteByte(UART4, sync);
			    wait_Waitms(500);
			    UART_WriteByte(UART4, var32);
			    wait_Waitms(500);
			    UART_WriteByte(UART4, var2);
			    wait_Waitms(1000);
				vTaskDelay(1500/portTICK_RATE_MS);

			}else if(strcmp(func,"GVel1")==0){
				UART_WriteByte(UART4, sync);
				wait_Waitms(500);
			    UART_WriteByte(UART4, var21);
				wait_Waitms(500);
				g_vel1 = UART_ReadByte(UART4);
				wait_Waitms(1000);

				//WRITER
				if(g_vel1==128){
					config_msg = "g_vel/mot1/128/";
					gvel1 = "128";
				}else if(g_vel1==0){
					config_msg = "g_vel/mot1/0/";
					gvel1 = "0";
				}else if(g_vel1==255){
					config_msg = "g_vel/mot1/255/";
					gvel1 = "255";
				}

				LCD_clear_row(ROW_TOP);
				LCD_printstr("Vel Motor 1:");
				LCD_clear_row(ROW_BOTTOM);
				LCD_printstr(gvel1);

				struct netconn * netconn;
				netconn = netconn_new(NETCONN_TCP);
				netconn_connect(netconn, &sendIP, TCP_PORT);
				netconn_write(netconn, config_msg,strlen(config_msg), TCP_WRITE_FLAG_COPY);
				netconn_delete(netconn);
				vTaskDelay(2000/portTICK_RATE_MS);

			}else if(strcmp(func,"GVel2")==0){
			    UART_WriteByte(UART4, sync);
			    wait_Waitms(500);
			    UART_WriteByte(UART4, var22);
			    wait_Waitms(500);
			    g_vel2 = UART_ReadByte(UART4);
			    wait_Waitms(1000);

				if(g_vel2==128){
					config_msg = "g_vel/mot2/128/";
					gvel2 = "128";
				}else if(g_vel2==0){
					config_msg = "g_vel/mot2/0/";
					gvel2 = "0";
				}else if(g_vel2==255){
					config_msg = "g_vel/mot2/255/";
					gvel2 = "255";
				}
				LCD_clear_row(ROW_TOP);
				LCD_printstr("Vel Motor 2:");
				LCD_clear_row(ROW_BOTTOM);
				LCD_printstr(gvel2);

				struct netconn * netconn;
				netconn = netconn_new(NETCONN_TCP);
				netconn_connect(netconn, &sendIP, TCP_PORT);
				netconn_write(netconn, config_msg,strlen(config_msg), TCP_WRITE_FLAG_COPY);
				netconn_delete(netconn);
				vTaskDelay(2000/portTICK_RATE_MS);

			}else if(strcmp(func,"stop")==0){

				LCD_clear_row(ROW_TOP);
				LCD_printstr("Parada de");
				LCD_clear_row(ROW_BOTTOM);
				LCD_printstr("Emergencia.");
				UART_WriteByte(UART4, sync);
				wait_Waitms(500);
				UART_WriteByte(UART4, var34);
				wait_Waitms(500);
				UART_WriteByte(UART4, 0);
				wait_Waitms(500);
			    UART_WriteByte(UART4, sync);
			    wait_Waitms(500);
			    UART_WriteByte(UART4, var31);
			    wait_Waitms(500);
			    UART_WriteByte(UART4, stop);
			    wait_Waitms(1000);
			    UART_WriteByte(UART4, sync);
			    wait_Waitms(500);
			    UART_WriteByte(UART4, var32);
			    wait_Waitms(500);
			    UART_WriteByte(UART4, stop);

			}else{
				PRINTF("COMANDO NO VALIDO");
			}
		}
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
		netconn_accept(netconn, &newconn);
		while (ERR_OK == netconn_recv(newconn, &netbuf))
		{
			turn_off_leds();
			turn_on_green();
			do
			{
				netbuf_copy(netbuf, buffer, sizeof(buffer));
				char delimitador[] = "/";
				char *token = strtok(buffer, delimitador);
				cmd = token;

				LCD_clear_row(ROW_TOP);
				LCD_printstr("Listening:");
				LCD_clear_row(ROW_BOTTOM);

				if(strcmp(cmd, UART_COMMAND) == 0){
					token = strtok(NULL, delimitador);
					func = token;
					token = strtok(NULL, delimitador);
					var = token;
					xMessageBufferSend(uart_handler, (void *) buffer, strlen(buffer), 0);
				}else if(strcmp(cmd, TEMP_COMMAND) == 0){
					token = strtok(NULL, delimitador);
					temp = token;
					xMessageBufferSend(temp_handler, (void *) buffer, strlen(buffer), 0);
				}else{
					PRINTF("\n Comando Erroneo");
				}
			}
			while (netbuf_next(netbuf) >= 0);

			netbuf_delete(netbuf);
		}
		netconn_delete(newconn);
		vTaskDelay(2000/portTICK_RATE_MS);
		LCD_clear_row(ROW_TOP);
		LCD_printstr(ip);
		LCD_clear_row(ROW_BOTTOM);
		LCD_printstr(port_announce);
		turn_off_leds();
		turn_on_red();
	}
}


/**
 * @brief Initialize the TCP/IP stack and set an IP address.
 *
 * Initialize the TCP/IP stack using lwIP which is a small footprint
 * implementation of the TCP/IP protocol suite.
 * After a successful initialization of the stack, set up the network interface
 * of the board.
 *
 *
 * @param[in] arg argument to the thread.
 */
void init_IP(void * arg)
{
	LCD_clear_row(ROW_TOP);
	LCD_printstr("Obtaining IP");
	LCD_clear_row(ROW_BOTTOM);
	LCD_printstr("Please wait...");

	struct netif netif;
	ip4_addr_t ip_address;
	ip4_addr_t netmask;
	ip4_addr_t gateway;

	IP4_ADDR(&ip_address, 192,168,0,12);
	IP4_ADDR(&netmask, 255,255,255,0);
	IP4_ADDR(&gateway, 192,168,0,1);
	IP4_ADDR(&sendIP, 192,168,0,10);

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

	for(;;)
	{
			PRINTF("IPv4 Address : %s\n", ipaddr_ntoa(&netif.ip_addr));
			PRINTF("IPv4 Netmask : %s\n", ipaddr_ntoa(&netif.netmask));
			PRINTF("IPv4 Gateway : %s\n", ipaddr_ntoa(&netif.gw));

			LCD_clear_row(ROW_TOP);
			ip = ipaddr_ntoa(&netif.ip_addr);
			LCD_printstr(ipaddr_ntoa(&netif.ip_addr));

			vTaskResume(tcp_listener_handle);
			vTaskDelete(NULL);
	}
}




