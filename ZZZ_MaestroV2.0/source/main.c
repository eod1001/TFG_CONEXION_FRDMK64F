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
 * @date
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

#include "lwip/opt.h"
#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "ethernetif.h"


#include "led.h"



/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "fsl_adc16.h"
#include "fsl_gpio.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"



/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** Media access control address used for interface configuration. */
#define MAC_ADDRESS {0x02, 0x12, 0x13, 0x10, 0x15, 0x10}
#define TCP_PORT 1234U /**< Listening port. */
#define ch1ADC 12U
#define ch2ADC 13U
#define ChannelGroup 0U

#define TempChannelGroup 0U
#define Tempch1ADC 11U



#define BUFFER_LENGTH 1400 /**< Bytes of data in TCP packets. */
#define MOTOR_COMMAND "g_vel"
#define NUM_MOTOR1_COMMAND "mot1"
#define NUM_MOTOR2_COMMAND "mot2"

#define BOARD_D12_GPIO        GPIOD
#define BOARD_D12_PORT        PORTD
#define BOARD_D12_GPIO_PIN    3U
#define D12_Button        	 "D12"

#define BOARD_D8_GPIO        GPIOC
#define BOARD_D8_PORT        PORTC
#define BOARD_D8_GPIO_PIN    12U
#define D8_Button        	 "D8"

#define BOARD_D7_GPIO        GPIOC
#define BOARD_D7_PORT        PORTC
#define BOARD_D7_GPIO_PIN    3U
#define D7_Button        	 "D7"

#define BOARD_D4_GPIO        GPIOB
#define BOARD_D4_PORT        PORTB
#define BOARD_D4_GPIO_PIN    23U
#define D4_Button        	 "D4"

#define BOARD_SW3_GPIO        GPIOA
#define BOARD_SW3_PORT        PORTA
#define BOARD_SW3_GPIO_PIN    4U
#define BOARD_SW3_NAME        "SW3"

#define BOARD_SW2_GPIO        GPIOC
#define BOARD_SW2_PORT        PORTC
#define BOARD_SW2_GPIO_PIN    6U
#define BOARD_SW2_NAME        "SW2"

#define MAXTEMP 8//30

typedef unsigned char uint8_t;
typedef void * button_handle_t;
typedef void * button_config_t;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void stack_init_thread(void * arg);
void tcp_listener_thread(void * arg);
void ADC_Writer_thread(void * arg);
void temp_thread(void * arg);
void turn_off_expansion_leds();
void turn_on_expansion_leds();
void rx_command_check(char * cmd, char * nMot, char * vel);
/*******************************************************************************
 * Variables
 ******************************************************************************/
TaskHandle_t tcp_listener_handle;
TaskHandle_t ADC_Writer_handle;
TaskHandle_t temp_handle;
uint32_t TempValue;
err_t TCP_WRITE_FLAG_COPY;
ip4_addr_t ipaddr;
ip4_addr_t sendIPA;
ip4_addr_t sendIPB;
bool g_Adc16ConversionDoneFlag = false;
uint32_t g_Adc16ConversionValue;


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



	gpio_pin_config_t P_config = {
	        kGPIO_DigitalInput,
	        1,
	    };
	gpio_pin_config_t E_config = {
		        kGPIO_DigitalInput,
		        0,
		    };

	GPIO_PinInit(BOARD_D12_GPIO, BOARD_D12_GPIO_PIN, &E_config);
	GPIO_PinInit(BOARD_D8_GPIO, BOARD_D8_GPIO_PIN, &E_config);
	GPIO_PinInit(BOARD_D7_GPIO, BOARD_D7_GPIO_PIN, &E_config);
	GPIO_PinInit(BOARD_D4_GPIO, BOARD_D4_GPIO_PIN, &E_config);
	GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &P_config);
	GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &P_config);

	/* Tasks creation. */
	if (xTaskCreate(stack_init_thread, "stack_init_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, NULL) != pdPASS)
	{
		PRINTF("stack_init_thread creation failed.\n");
	}

	if (xTaskCreate(ADC_Writer_thread, "ADC_Writer_thread",
				DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, &ADC_Writer_handle) != pdPASS)
	{
		PRINTF("adc_task creation failed.\n");
	}

	vTaskSuspend(ADC_Writer_handle);

	if (xTaskCreate(tcp_listener_thread, "tcp_listener_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, &tcp_listener_handle) != pdPASS)
	{
		PRINTF("tcp_listener_thread creation failed.\n");
	}
	vTaskSuspend(tcp_listener_handle);

	if (xTaskCreate(temp_thread, "temp_thread",
			DEFAULT_THREAD_STACKSIZE, NULL, DEFAULT_THREAD_PRIO, &temp_handle) != pdPASS)
	{
		PRINTF("tcp_listener_thread creation failed.\n");
	}
	vTaskSuspend(temp_handle);
	turn_off_expansion_leds();
	vTaskStartScheduler();
	return EXIT_FAILURE;
}

void temp_thread(void * arg)
{
	adc16_config_t tempConfigStruct;
	adc16_channel_config_t tempChannelConfigStruct;
	ADC16_GetDefaultConfig(&tempConfigStruct);

	#ifdef BOARD_ADC_USE_ALT_VREF
	tempConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
	#endif

	ADC16_Init(ADC1, &tempConfigStruct);
	ADC16_EnableHardwareTrigger(ADC1, false); /* Make sure the software trigger is used. */
	tempChannelConfigStruct.enableInterruptOnConversionCompleted = false; /* Enable the interrupt. */

	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    tempChannelConfigStruct.enableDifferentialConversion = false;
	#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

	#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */


	for(;;){
		if(GPIO_PinRead(GPIOB, BOARD_D4_GPIO_PIN)==1){
			turn_on_expansion_leds();
			tempChannelConfigStruct.channelNumber = Tempch1ADC;
			ADC16_SetChannelConfig(ADC1, TempChannelGroup, &tempChannelConfigStruct);
			TempValue = ADC16_GetChannelConversionValue(ADC1, TempChannelGroup);
			vTaskDelay(1500/portTICK_RATE_MS);
			char msg_temp[10] ="";
			if(TempValue!=0){
			char valorc;
			int valor = (int)TempValue;
			int valorAux = valor;
			int z = 0;
			PRINTF("\n VALOR TempValue: %d",valor);
			while(valorAux>=10){
				valorAux = valorAux/10;
				z++;
			}
			char str[z];
			    int x = z;
			    while(valor!=0){
			        valorc = (valor % 10) + 48;
					str[z] = (char)valorc;
					valor = valor/10;
			        z--;
				}
			    str[x+1] = '\0';


			    strcat(msg_temp,"temp/");
			    strcat(msg_temp,str);
			    strcat(msg_temp,"/");
			}else{
				strcat(msg_temp,"temp/0/");
			}
				printf("\n mensaje: %s",msg_temp);

				if(TempValue > MAXTEMP){

					struct netconn * netconn;
					netconn = netconn_new(NETCONN_TCP);
					vTaskDelay(5000/portTICK_RATE_MS);
					netconn_connect(netconn, &sendIPA, TCP_PORT);
					netconn_write(netconn, msg_temp,strlen(msg_temp), TCP_WRITE_FLAG_COPY);
					netconn_delete(netconn);
					PRINTF("msg_temp: %s",msg_temp);
					PRINTF("\n VALOR: %d \n",TempValue);
					turn_off_expansion_leds();
				}
			}
		}
}

void turn_on_expansion_leds(){
	GPIO_PinWrite(LA_GPIO, LA_PIN, 1U);
	GPIO_PinWrite(LV_GPIO, LV_PIN, 1U);
	GPIO_PinWrite(LB_GPIO, LB_PIN, 1U);
	GPIO_PinWrite(LR_GPIO, LR_PIN, 1U);

}
void turn_off_expansion_leds(){
	GPIO_PinWrite(LA_GPIO, LA_PIN, 0U);
	GPIO_PinWrite(LV_GPIO, LV_PIN, 0U);
	GPIO_PinWrite(LB_GPIO, LB_PIN, 0U);
	GPIO_PinWrite(LR_GPIO, LR_PIN, 0U);

}


void ADC_Writer_thread(void * arg)
{

	char * config_msg;
	char * Writer_String;
	adc16_config_t adc16ConfigStruct;
	adc16_channel_config_t adc16ChannelConfigStruct;
	ADC16_GetDefaultConfig(&adc16ConfigStruct);

	#ifdef BOARD_ADC_USE_ALT_VREF
	adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
	#endif

	ADC16_Init(ADC0, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false; /* Enable the interrupt. */

	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
	#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

	#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

	for(;;){
			if(GPIO_PinRead(GPIOD, BOARD_D12_GPIO_PIN)==1){
				GPIO_PinWrite(LB_GPIO, LB_PIN, 1U);
				adc16ChannelConfigStruct.channelNumber = ch1ADC;
			    ADC16_SetChannelConfig(ADC0, ChannelGroup, &adc16ChannelConfigStruct);
			    while (0U == (kADC16_ChannelConversionDoneFlag &
			                          ADC16_GetChannelStatusFlags(ADC0, ChannelGroup)))
			            {
			            }
		        g_Adc16ConversionValue = ADC16_GetChannelConversionValue(ADC0, ChannelGroup);
		        PRINTF("\n value: %d",g_Adc16ConversionValue);
		        if(g_Adc16ConversionValue>2900){
		        	Writer_String = "uart/Vel1/255/";
		        	PRINTF("-255");
		        }else if (g_Adc16ConversionValue<1100){
		        	Writer_String = "uart/Vel1/0/";
		        	PRINTF("-0");
		        }else{
		        	Writer_String = "uart/Vel1/128/";
		        	PRINTF("-128");
		        }

		        config_msg = "uart/Mode/0/";
		        GPIO_PinWrite(LV_GPIO, LV_PIN, 1U);
				PRINTF("\n %s",Writer_String);
				struct netconn * netconn;
				netconn = netconn_new(NETCONN_TCP);
				netconn_connect(netconn, &sendIPA, TCP_PORT);
				netconn_write(netconn, config_msg,strlen(config_msg), TCP_WRITE_FLAG_COPY);
				GPIO_PinWrite(LA_GPIO, LA_PIN, 1U);
				vTaskDelay(1500/portTICK_RATE_MS);
				netconn_write(netconn, Writer_String,strlen(Writer_String), TCP_WRITE_FLAG_COPY);
				netconn_delete(netconn);
				GPIO_PinWrite(LR_GPIO, LR_PIN, 1U);

			}
			if(GPIO_PinRead(GPIOC, BOARD_D8_GPIO_PIN)==1){
				GPIO_PinWrite(LB_GPIO, LB_PIN, 1U);
				adc16ChannelConfigStruct.channelNumber = ch2ADC;
				ADC16_SetChannelConfig(ADC0, ChannelGroup, &adc16ChannelConfigStruct);
				while (0U == (kADC16_ChannelConversionDoneFlag &
							 ADC16_GetChannelStatusFlags(ADC0, ChannelGroup)))
					{
					}
				g_Adc16ConversionValue = ADC16_GetChannelConversionValue(ADC0, ChannelGroup);
				PRINTF("\n value: %d",g_Adc16ConversionValue);
		        if(g_Adc16ConversionValue>2900){
		        	Writer_String = "uart/Vel2/255/";
		        	PRINTF("-255");
		        }else if (g_Adc16ConversionValue<1100){
		        	Writer_String = "uart/Vel2/0/";
		        	PRINTF("-0");
		        }else{
		        	Writer_String = "uart/Vel2/128/";
		        	PRINTF("-128");
		        }

		        config_msg = "uart/Mode/0/";
		        GPIO_PinWrite(LV_GPIO, LV_PIN, 1U);

		        PRINTF("\n %s",Writer_String);
				struct netconn * netconn;

				netconn = netconn_new(NETCONN_TCP);
				netconn_connect(netconn, &sendIPB, TCP_PORT);
				netconn_write(netconn, config_msg,strlen(config_msg), TCP_WRITE_FLAG_COPY);
				GPIO_PinWrite(LA_GPIO, LA_PIN, 1U);
				vTaskDelay(1500/portTICK_RATE_MS);
				netconn_write(netconn, Writer_String,strlen(Writer_String), TCP_WRITE_FLAG_COPY);
				GPIO_PinWrite(LR_GPIO, LR_PIN, 1U);
				netconn_delete(netconn);

			}
			if(GPIO_PinRead(GPIOA, BOARD_SW3_GPIO_PIN)==0){
				turn_on_expansion_leds();
				config_msg = "uart/GVel2/";
				struct netconn * netconn;
				netconn = netconn_new(NETCONN_TCP);
				vTaskDelay(1000/portTICK_RATE_MS);
				netconn_connect(netconn, &sendIPA, TCP_PORT);
				netconn_write(netconn, config_msg,strlen(config_msg), TCP_WRITE_FLAG_COPY);
				netconn_delete(netconn);

			}
			if(GPIO_PinRead(GPIOC, BOARD_SW2_GPIO_PIN)==0){
				turn_on_expansion_leds();
				config_msg = "uart/GVel1/";
				struct netconn * netconn;
				netconn = netconn_new(NETCONN_TCP);
				vTaskDelay(1000/portTICK_RATE_MS);
				netconn_connect(netconn, &sendIPA, TCP_PORT);
				netconn_write(netconn, config_msg,strlen(config_msg), TCP_WRITE_FLAG_COPY);
				netconn_delete(netconn);

			}
			if(GPIO_PinRead(GPIOC, BOARD_D7_GPIO_PIN)==1){
				turn_on_expansion_leds();
				config_msg = "uart/stop/";
				struct netconn * netconn;
				netconn = netconn_new(NETCONN_TCP);
				vTaskDelay(1500/portTICK_RATE_MS);
				netconn_connect(netconn, &sendIPB, TCP_PORT);
				netconn_write(netconn, config_msg,strlen(config_msg), TCP_WRITE_FLAG_COPY);
				netconn_delete(netconn);

				netconn = netconn_new(NETCONN_TCP);
				vTaskDelay(1500/portTICK_RATE_MS);
				netconn_connect(netconn, &sendIPB, TCP_PORT);
				netconn_write(netconn, config_msg,strlen(config_msg), TCP_WRITE_FLAG_COPY);
				netconn_delete(netconn);

			}
		vTaskDelay(1500/portTICK_RATE_MS);
		turn_off_leds();
		turn_off_expansion_leds();
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
	turn_off_expansion_leds();
	struct netbuf  * netbuf;
	struct netconn * netconn;
	struct netconn * newconn;
	char buffer[BUFFER_LENGTH];

	netconn = netconn_new(NETCONN_TCP);
	netconn_bind(netconn, NULL, TCP_PORT);
	netconn_listen(netconn);
	char * vel;
	char * nMot;
	char * cmd;
	char port_announce[16];
	sprintf(port_announce, "TCP port %d", TCP_PORT);


	for (;;)
	{
		PRINTF("LISTENER");
		netconn_accept(netconn, &newconn);

		while (ERR_OK == netconn_recv(newconn, &netbuf))
		{
			do
			{
				netbuf_copy(netbuf, buffer, sizeof(buffer));
				char delimitador[] = "/";
				char *token = strtok(buffer, delimitador);
				cmd = token;
				token = strtok(NULL, delimitador);
				nMot = token;
				token = strtok(NULL, delimitador);
				vel = token;
				PRINTF("\n %s, %s, %s ",cmd,nMot,vel);
				rx_command_check(cmd, nMot, vel);
			}
			while (netbuf_next(netbuf) >= 0);

			netbuf_delete(netbuf);
		}
		netconn_delete(newconn);

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
void rx_command_check(char * cmd, char * nMot, char * vel)
{
	char * nMotor;
	if (strcmp(cmd, MOTOR_COMMAND) == 0)
	{
		if(strcmp(nMot, NUM_MOTOR1_COMMAND) == 0){
			nMotor = "motor 1";
		}else if(strcmp(nMot, NUM_MOTOR2_COMMAND) == 0){
			nMotor = "motor 2";
		}else{
			PRINTF("INVALID COMMAND NUM_MOTOR");
		}
		PRINTF("Numero de motor: %s, velocidad: %s",nMotor,vel);
	}else{
		PRINTF("Invalid command.\n");
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
	PRINTF("\n Obteniendo IP \n");

	struct netif netif;

	ip4_addr_t ip_address;
	ip4_addr_t netmask;
	ip4_addr_t gateway;

	IP4_ADDR(&ip_address, 192,168,0,10);
	IP4_ADDR(&netmask, 255,255,255,0);
	IP4_ADDR(&gateway, 192,168,0,1);
	IP4_ADDR(&sendIPA, 192,168,0,11);
	IP4_ADDR(&sendIPB, 192,168,0,12);

	ethernetif_config_t config =
	{
			.phyAddress = BOARD_ENET0_PHY_ADDRESS,
			.clockName = kCLOCK_CoreSysClk,
			.macAddress = MAC_ADDRESS,
	};

	/* Initialization of the TCP/IP stack. */
	tcpip_init(NULL, NULL);

	/* Set up the network interface. */
	netif_add(&netif, &ip_address, &netmask, &gateway, &config,
			ethernetif0_init, tcpip_input);
	netif_set_default(&netif);
	netif_set_up(&netif);

	for(;;)
	{

			PRINTF("IPv4 Address : %s\n", ipaddr_ntoa(&netif.ip_addr));
			PRINTF("IPv4 Netmask : %s\n", ipaddr_ntoa(&netif.netmask));
			PRINTF("IPv4 Gateway : %s\n", ipaddr_ntoa(&netif.gw));

			vTaskResume(tcp_listener_handle);
			vTaskResume(ADC_Writer_handle);
			vTaskResume(temp_handle);
			vTaskDelete(NULL);
	}
}

