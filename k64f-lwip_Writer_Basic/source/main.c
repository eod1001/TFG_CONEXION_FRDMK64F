/**
 * Copyright 2019 rpc0027
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
 * @author RPC0027
 * @date December 2018
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
#include <stdio.h>
#include <stdlib.h>

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

//#include "lcd.h"
#include "led.h"
#include "pwm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** Media access control address used for interface configuration. */
#define MAC_ADDRESS {0x02, 0x12, 0x13, 0x10, 0x15, 0x35} //02:12:13:10:15:11
#define TCP_PORT 1234U /**< Listening port. */

//#define ROW_TOP 0U /**< Top row of the LCD. */
//#define ROW_BOTTOM 1U /**< Bottom row of the LCD. */
#define BUFFER_LENGTH 1400 /**< Bytes of data in TCP packets. */
#define COMMAND_SIZE 4 /**< Characters used for commands name. */
#define LED_COMMAND "led:" /**< Name of the LED command. */
#define LED_COLOR_INDEX 4 /**< Index of the color argument. */
#define MSG_COMMAND "msg:" /**< Name of the MSG command. */
#define MSG_ROW_INDEX 4 /**< Index of the row argument. */
#define MSG_TXT_INDEX 6 /**< Index of the text argument. */
#define PWM_COMMAND "pwm:" /**< Name of the PWM command. */
#define PWM_DEVICE_INDEX 4 /**< Index of the PWM device argument.*/
#define PWM_PERCENTAGE_INDEX 6 /**< Index of the percentage argument. */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void stack_init_thread(void * arg);
void tcp_writer_thread(void * arg);
/*void rx_command_check(char * buffer, uint16_t null_terminator_pos);
void led_thread(void * arg);
void msg_thread(void * arg);
void pwm_thread(void * arg);
uint8_t range_adjust(long value);*/

/*******************************************************************************
 * Variables
 ******************************************************************************/
MessageBufferHandle_t led_handler; /**< Buffer handler of led_thread. */
MessageBufferHandle_t msg_handler; /**< Buffer handler of msg_thread. */
MessageBufferHandle_t pwm_handler; /**< Buffer handler of pwm_thread. */
uint8_t messageBuffer[BUFFER_LENGTH]; /**< Buffer to be used by the threads. */
TaskHandle_t tcp_writer_handle; /**< Task handle of tcp_listener_thread. */
err_t TCP_WRITE_FLAG_COPY;
ip4_addr_t ipaddr;


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
int main(void)   //writer
{
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();


	/* Disable Memory Protection Unit. */
	SYSMPU->CESR &= ~SYSMPU_CESR_VLD_MASK;
	turn_off_leds();
	/* LCD initialization. */
	//LCD_Init(0x3F, 16, 2, LCD_5x8DOTS);

	/* Creation of the message buffers. */
	led_handler = xMessageBufferCreate(BUFFER_LENGTH);
	msg_handler = xMessageBufferCreate(BUFFER_LENGTH);
	pwm_handler = xMessageBufferCreate(BUFFER_LENGTH);

	/* Tasks creation. */
	if (xTaskCreate(stack_init_thread, "stack_init_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, NULL) != pdPASS)
	{
		PRINTF("stack_init_thread creation failed.\n");
	}

	if (xTaskCreate(tcp_writer_thread, "tcp_writer_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO-1 , &tcp_writer_handle) != pdPASS)
	{
		PRINTF("tcp_listener_thread creation failed.\n");
	}
	/* Task suspended until the TCP/IP stack is initialized. */
	//vTaskSuspend(tcp_listener_handle);

	/*if (xTaskCreate(led_thread, "led_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO - 1, NULL) != pdPASS)
	{
		PRINTF("led_thread creation failed.\n");
	}

	if (xTaskCreate(msg_thread, "msg_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO - 1, NULL) != pdPASS)
	{
		PRINTF("msg_thread creation failed.\n");
	}

	if (xTaskCreate(pwm_thread, "pwm_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO - 1, NULL) != pdPASS)
	{
		PRINTF("pwm_thread creation failed.\n");
	}
*/
	vTaskStartScheduler();

	return EXIT_FAILURE;
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

	PRINTF("Obtaining IP, pls wait..\n");

	struct netif netif;
	struct dhcp * dhcp;

	ip4_addr_t ip_address;
	ip4_addr_t netmask;
	ip4_addr_t gateway;

	ethernetif_config_t config =
	{
			.phyAddress = BOARD_ENET0_PHY_ADDRESS,
			.clockName = kCLOCK_CoreSysClk,
			.macAddress = MAC_ADDRESS,
	};

	/* Initialization of the TCP/IP stack. */
	tcpip_init(NULL, NULL);


	/* Set up the network interface. */


	netif_add(&netif, &ip_address, &netmask, &gateway, &config, ethernetif0_init, tcpip_input);



	IP4_ADDR(&ip_address, 192,168,0,36);
	IP4_ADDR(&netmask, 255,255,255,0);
	IP4_ADDR(&gateway, 192,168,0,1);
	netif_set_addr(&netif, &ip_address, &netmask, &gateway);

	netif_set_up(&netif);
	netif_set_default(&netif);
	//autoip_start();
	//dhcp_start(&netif);
		//dhcp_set_struct(&netif, dhcp);
	for(;;){
		PRINTF("IPv4 Address : %s\n", ipaddr_ntoa(&netif.ip_addr));
		PRINTF("IPv4 Netmask : %s\n", ipaddr_ntoa(&netif.netmask));
		PRINTF("IPv4 Gateway : %s\n", ipaddr_ntoa(&netif.gw));

		vTaskResume(tcp_writer_handle);
		vTaskDelete(NULL);
	}

	/* Start DHCP negotiation.
	dhcp_start(&netif);

	for(;;)
	{
		dhcp = netif_dhcp_data(&netif);

		if (dhcp->state == DHCP_STATE_BOUND)
		{
			PRINTF("IPv4 Address : %s\n", ipaddr_ntoa(&netif.ip_addr));
			PRINTF("IPv4 Netmask : %s\n", ipaddr_ntoa(&netif.netmask));
			PRINTF("IPv4 Gateway : %s\n", ipaddr_ntoa(&netif.gw));

			//LCD_clear_row(ROW_TOP);
			//LCD_printstr(ipaddr_ntoa(&netif.ip_addr));

			vTaskResume(tcp_writer_handle);
			vTaskDelete(NULL);
		}
	} */

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
void tcp_writer_thread(void * arg)
{
	printf("WRITER\n");
	char *string = "led:r";
	struct netconn * netconn;


	IP4_ADDR(&ipaddr, 192,168,0,14);


	for (;;)
	{
		netconn = netconn_new(NETCONN_TCP);
		turn_on_blue();
		vTaskDelay(2500/portTICK_RATE_MS);

		netconn_connect(netconn, &ipaddr, TCP_PORT);//bind/IP4_ADDR_ANY
		turn_on_white();
		if (1){
			turn_off_leds();
			turn_on_green();
			//printf(string);
			netconn_write(netconn, string,strlen(string), TCP_WRITE_FLAG_COPY);
		}

		netconn_delete(netconn);
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
/*void rx_command_check(char * buffer, uint16_t null_terminator_pos)
{
	if (COMMAND_SIZE > null_terminator_pos)
	{
		PRINTF("Invalid command: too short.\n");
	}
	else
	{
		if (strncmp(buffer, LED_COMMAND, COMMAND_SIZE) == 0)
		{
			xMessageBufferSend(led_handler, (void *) buffer, strlen(buffer), 0);
		}
		else if (strncmp(buffer, MSG_COMMAND, COMMAND_SIZE) == 0)
		{
			xMessageBufferSend(msg_handler, (void *) buffer, strlen(buffer) + 1, 0);
		}
		else if (strncmp(buffer, PWM_COMMAND, COMMAND_SIZE) == 0)
		{
			xMessageBufferSend(pwm_handler, (void *) buffer, strlen(buffer), 0);
		}
		else
		{
			PRINTF("Invalid command.\n");
		}
	}
}*/

/**
 * @brief Modify the color of the on board RGB LED.
 *
 * Using a letter sent as the command argument turn on one of the available
 * colors of the on board LED.
 * Also, can turn off all the LEDs.
 *
 * @param[in] arg argument to the thread.
 */
/*void led_thread(void *arg)
{
	char buffer[BUFFER_LENGTH];
	size_t xReceivedBytes;
	const TickType_t xBlockTime = pdMS_TO_TICKS(20);

	for (;;)
	{
		xReceivedBytes = xMessageBufferReceive(led_handler, (void *) buffer,
				sizeof(buffer), xBlockTime);

		if (xReceivedBytes)
		{
			switch (buffer[LED_COLOR_INDEX])
			{
			case 'r':
				turn_on_red();
				break;

			case 'g':
				turn_on_green();
				break;

			case 'b':
				turn_on_blue();
				break;

			case 'y':
				turn_on_yellow();
				break;

			case 'm':
				turn_on_magenta();
				break;

			case 'c':
				turn_on_cyan();
				break;

			case 'w':
				turn_on_white();
				break;

			case 'o':
				turn_off_leds();
				break;

			default:
				PRINTF("Invalid LED color.\n");
			}
		}
	}
}

/**
 * @brief Show a message in the LCD using the I2C bus.
 *
 * Show in one of the rows of the display a message of 16 characters.
 * The row must be selected using the adequate argument.
 *
 * @param[in] arg argument to the thread.
 */
/*void msg_thread(void *arg)
{
	char buffer[BUFFER_LENGTH];
	size_t xReceivedBytes;
	const TickType_t xBlockTime = pdMS_TO_TICKS(20);

	for (;;)
	{
		xReceivedBytes = xMessageBufferReceive(msg_handler, (void *) buffer,
				sizeof(buffer), xBlockTime);

		if (xReceivedBytes)
		{
			if (buffer[MSG_ROW_INDEX] == '0')
			{
				//LCD_clear_row(ROW_TOP);
				//LCD_printstr(buffer + MSG_TXT_INDEX);
				printf(buffer + MSG_TXT_INDEX);
			}
			else if (buffer[MSG_ROW_INDEX == '1'])
			{
				//LCD_clear_row(ROW_BOTTOM);
				//LCD_printstr(buffer + MSG_TXT_INDEX);
				printf(buffer + MSG_TXT_INDEX);
			}
			else
			{
				PRINTF("Invalid LCD line.\n");
			}
		}
	}
}

/**
 * @brief Modify the intensity of the Basic I/O Shield LEDs using PWM.
 *
 * Using a letter sent as the command argument the user can select one
 * of the LEDs on the Basic I/O.
 * The other argument must indicate the desired intensity between 0% and 100%.
 *
 * @param[in] arg argument to the thread.
 */
/*void pwm_thread(void *arg)
{
	char buffer[BUFFER_LENGTH];
	size_t xReceivedBytes;
	const TickType_t xBlockTime = pdMS_TO_TICKS(20);
	uint8_t percentage;

	for (;;)
	{
		xReceivedBytes = xMessageBufferReceive(pwm_handler, (void *) buffer,
				sizeof(buffer), xBlockTime);

		if (xReceivedBytes)
		{
			percentage = range_adjust(
					strtol(buffer + PWM_PERCENTAGE_INDEX, NULL, 10));

			switch (buffer[PWM_DEVICE_INDEX])
			{
			case 'w':
				update_pwm_dutycyle(WHITE_PWM, WHITE_CHANNEL, percentage);
				break;

			case 'g':
				update_pwm_dutycyle(GREEN_PWM, GREEN_CHANNEL,  percentage);
				break;

			case 'y':
				update_pwm_dutycyle(YELLOW_PWM, YELLOW_CHANNEL, percentage);
				break;

			case 'r':
				update_pwm_dutycyle(RED_PWM, RED_CHANNEL, percentage);
				break;

			default:
				PRINTF("Invalid PWM device.\n");
			}
		}
	}
}

/**
 * @brief Truncate values lower than 0 and higher than 100.
 *
 * A negative value becomes 0 and a positive value greater than 100 becomes 100.
 *
 * @param[in] value a number given by the user.
 * @return a value within range.
 */
/*uint8_t range_adjust(long value)
{
	uint8_t corrected_value;

	if (value < 0)
	{
		corrected_value = 0U;
	}
	else if (value > 100)
	{
		corrected_value = 100U;
	}
	else
	{
		corrected_value = (uint8_t) value;
	}

	return corrected_value;
}

/*** end of file ***/
