/**
 * Clase que controla la comunicacion tanto con los motores como con el modulo bluetooth.
 * @file modelo/Comunicacion.c
 * @author Jonatan Santos Barrios
 * @date 17/11/2013
 * @version 1.0
 */

/*#include "MOTORES.h"
#include "BLUETOOTH.h"
#include "modelo/Comunicacion.h"
#include "stdlib.h"
*/
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_uart.h"
//#include "uart4.h"
#include "stdint.h"
//#include "wait.h"
#include "fsl_uart_freertos.h"
#include "fsl_uart.h"


/*******************************************************************************
 * Defines
 ******************************************************************************/

typedef unsigned char uint8_t;



//typedef unsigned char BYTE;
/*******************************************************************************
 * Variables
 ******************************************************************************/



uart_handle_t g_uartHandle;



/**
 * @brief Envio de del modo de trabajo a los motores.
 *
 * Envia el valor del modo de trabajo de los motores a la controladora de motores.\n
 * Modos de trabajo: \n
 * &nbsp;&nbsp;0 - establecerVelocidad1 para motor1 y establecerVelocidad2 para motor2; valores entre 0 y 255.\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 - Maxima velocidad hacia atras\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;128 - Parado\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;255 - Maxima velocidad hacia adelante.\n
 * &nbsp;&nbsp;1 - establecerVelocidad1 para motor1 y establecerVelocidad2 para motor2.\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;-128 - Maxima velocidad hacia atras\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 - Parado\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;127 - Maxima velocidad hacia adelante.\n
 * &nbsp;&nbsp;2 - establecerVelocidad1 controla la velocidad de los dos motores y establecerVelocidad2 controla la velocidad angula o de giro.\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 - Maxima velocidad hacia atras\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;128 - Parado\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;255 - Maxima velocidad hacia adelante.\n
 * &nbsp;&nbsp;3 - establecerVelocidad1 controla la velocidad de los dos motores y establecerVelocidad2 controla la velocidad angula o de giro.\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;-128 - Maxima velocidad hacia atras\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 - Parado\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;127 - Maxima velocidad hacia adelante.\n
 * @param modo Un valor 0 y 3.
 */
void establecerModo(uint8_t modo, uart_rtos_handle_t  handleUART){
	// Mandamos los bytes del mensaje
	printf("ENVIA MODo ini");
	uint8_t var = 0x00;
	uint8_t var34 = 0x34;
	UART_RTOS_Send(&handleUART, &var, sizeof(var));
	printf("MODO 1");
	UART_RTOS_Send(&handleUART, &var34, sizeof(var34));
	printf("MODO 2");
	UART_RTOS_Send(&handleUART, &modo, sizeof(modo));
	//UART_WriteBlocking(UART4, &var,sizeof(var)); //(uint8_t *)0x00
	//UART_WriteBlocking(UART4, &var34,sizeof(var34));
	//UART_WriteBlocking(UART4, &modo,sizeof(modo));
	printf("ENVIA MODO fin");
}



/**
 * @brief Envio de la velocidad a los motores
 * 
 * Envia el valor de velocidad a la controladora de motores, segun el modo de trabajo
 * correspondera a velocidad de un motor o de ambos.
 * @param velocidad Segun el modo de trabajo el valor de velocidad ira entre 0 y 255 o -128 y 127.
 */
void establecerVelocidad1(uint8_t vel, uart_rtos_handle_t handleUART){
	// Mandamos los bytes del mensaje
	printf("ENVIA VEL ini");
	uint8_t var = 0x00;
	uint8_t var31 = 0x31;
	//UART_WriteBlocking(UART4, &var,sizeof(var)); //(uint8_t *)0x00nt
	UART_RTOS_Send(&handleUART, &var, sizeof(var));
	printf("1");
	//UART_WriteBlocking(UART4, &var31,sizeof(var31));
	UART_RTOS_Send(&handleUART, &var31, sizeof(var31));
	printf("2");
	//UART_WriteBlocking(UART4, &vel,sizeof(vel));
	UART_RTOS_Send(&handleUART, &vel, sizeof(vel));
	printf("ENVIA VEL fin");
}

/**
 * @brief Envio de la velocidad a los motores
 * 
 * Envia el valor de velocidad a la controladora de motores, segun el modo de trabajo
 * correspondera a velocidad de un motor o la velocidad angular.
 * @param velocidad Segun el modo de trabajo el valor de velocidad ira entre 0 y 255 o -128 y 127.
 */
void establecerVelocidad2(uint8_t vel){
	// Mandamos los bytes del mensaje
	uint8_t var = 0x00;
	uint8_t var32 = 0x32;
	UART_WriteBlocking(UART4, &var,sizeof(var)); //(uint8_t *)0x00
	UART_WriteBlocking(UART4, &var32,sizeof(var32));
	UART_WriteBlocking(UART4, &vel,sizeof(vel));

}

/**
 * @brief Envio de la aceleracion a los motores
 * 
 * Envia el valor de aceleracion de los motores a la controladora de motores.
 * @param aceleracion Un valor de aceleracion entre 1 y 10.
 */
void establecerAceleracion(uint8_t ace){
	// Mandamos los bytes del mensaje
	uint8_t var = 0x00;
	uint8_t var33 = 0x33;
	UART_WriteBlocking(UART4, &var,sizeof(var)); //(uint8_t *)0x00
	UART_WriteBlocking(UART4, &var33,sizeof(var33));
	UART_WriteBlocking(UART4, &ace,sizeof(ace));

}


/**
 * @brief Obtener el modo de trabajo de los motores
 * 
 * Pregunta a la controla de motores el modo de trabajo de los mismos.
 * Modos de trabajo: \n
 * &nbsp;&nbsp;0 - establecerVelocidad1 para motor1 y establecerVelocidad2 para motor2; valores entre 0 y 255.\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 - Maxima velocidad hacia atras\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;128 - Parado\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;255 - Maxima velocidad hacia adelante.\n
 * &nbsp;&nbsp;1 - establecerVelocidad1 para motor1 y establecerVelocidad2 para motor2.\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;-128 - Maxima velocidad hacia atras\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 - Parado\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;127 - Maxima velocidad hacia adelante.\n
 * &nbsp;&nbsp;2 - establecerVelocidad1 controla la velocidad de los dos motores y establecerVelocidad2 controla la velocidad angula o de giro.\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 - Maxima velocidad hacia atras\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;128 - Parado\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;255 - Maxima velocidad hacia adelante.\n
 * &nbsp;&nbsp;3 - establecerVelocidad1 controla la velocidad de los dos motores y establecerVelocidad2 controla la velocidad angula o de giro.\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;-128 - Maxima velocidad hacia atras\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;0 - Parado\n
 * &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;127 - Maxima velocidad hacia adelante.\n
 * @return El modo de trabajo, un valor entre 0 y 3.
 *
BYTE obtenerModo(){
	// Variable del dato a recibir
	BYTE dato;
	
	// Mandamos los bytes del mensaje
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x00) != 0) {};
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x2B) != 0) {};
	// Recibimos el dato
	while(UART_TransferReceiveNonBlocking(UART4, &g_uartHandle, &dato, NULL) != 0) {};
	
	// Devolvemos el dato
	return dato;
}

/**
 * @brief Obtener la velocidad de los motores
 * 
 * Pregunta a la controladora la velocidad de los motores, segun el modo de trabajo
 * correspondera a velocidad de un motor o de ambos.
 * @return Segun el modo de trabajo el valor de la velocidad ira entre 0 y 255 o -128 y 127.
 *
BYTE obtenerVelocidad1(){
	// Variable del dato a recibir
	BYTE dato;
	
	// Mandamos los bytes del mensaje
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x00) != 0) {};
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x21) != 0) {};
	// Recibimos el dato
	while(UART_TransferReceiveNonBlocking(UART4, &g_uartHandle, &dato, NULL) != 0) {};
	
	// Devolvemos el dato
	return dato;
}

/**
 * @brief Obtener la velocidad de los motores
 * 
 * Pregunta a la controladora la velocidad de los motores, segun el modo de trabajo
 * correspondera a velocidad de un motor o la velocidad angular.
 * @return Segun el modo de trabajo el valor de la velocidad ira entre 0 y 255 o -128 y 127.
 *
BYTE obtenerVelocidad2(){
	// Variable del dato a recibir
	BYTE dato;
	
	// Mandamos los bytes del mensaje
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x00) != 0) {};
	while(UART_TransferReceiveNonBlocking(UART4, &g_uartHandle, 0x22, NULL) != 0) {};
	// Recibimos el dato
	while(UART_TransferReceiveNonBlocking(UART4, &g_uartHandle, &dato, NULL) != 0) {};
	
	// Devolvemos el dato
	return dato;
}

/**
 * @brief Obtener el valor del encoder del motor1
 * 
 * Pregunta a la controladora el valor del encoder1.
 * @return Valor del encoder1 devuelto por la controladora.
 *
long obtenerEncoder1(){
	// Contador
	int i;
	// Variable del resultado
	long result;
	// Variable del dato a recibir
	BYTE dato;
	
	// Mandamos los bytes del mensaje
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x00) != 0) {};
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x23) != 0) {};
	// Bucle para recibir los 4 bytes de la respuesta
	for(i = 0; i < 4; i++){
		// Recibimos el dato
		while(UART_TransferReceiveNonBlocking(UART4, &g_uartHandle, &dato, NULL) != 0) {};
		// Segun el n� de dato tiene un valor u otro.
		switch (i){
			case 0:
				result = dato << 24;
				break;
			case 1:
				result += dato << 16;
				break;
			case 2:
				result += dato << 8;
				break;
			case 3:
				result += dato;
		}
	}
	// Devolvemos el resultado
	return result;
}

/**
 * @brief Obtener el valor del encoder del motor2
 * 
 * Pregunta a la controladora el valor del encoder2.
 * @return Valor del encoder2 devuelto por la controladora.
 *
long obtenerEncoder2(){
	// Contador
	int i;
	// Variable del resultado
	long result;
	// Variable del dato a recibir
	BYTE dato;
	
	// Mandamos los bytes del mensaje
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x00) != 0) {};
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x24) != 0) {};
	// Bucle para recibir los 4 bytes de la respuesta
	for(i = 0; i < 4; i++){
		// Recibimos el dato
		while(UART_TransferReceiveNonBlocking(UART4, &g_uartHandle, &dato, NULL) != 0) {};
		// Segun el n� de dato tiene un valor u otro.
		switch (i){
			case 0:
				result = dato << 24;
				break;
			case 1:
				result += dato << 16;
				break;
			case 2:
				result += dato << 8;
				break;
			case 3:
				result += dato;
		}
	}
	// Devolvemos el resultado
	return result;
}

/**
 * @brief Resetea los encoder
 * 
 * Manda a la controladora la orden de poner a 0 los encodres.
 *
void resetearEncoders(){
	// Mandamos los bytes del mensaje
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x00) != 0) {};
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x35) != 0) {};

}

/**
 * @brief Deshabilita el timeout.
 * 
 * Manda a la controladora la orden de deshabilitar el timeout.
 * Si el timeout esta habilitado el movimiento finaliza a los dos segundos si no se recibe ninguno orden nueva de movimiento, 
 * estando habilitado el moviento se mantiene hasta recibir una nueva orden.
 *
void deshabilitarTimeOut(){
	// Mandamos los bytes del mensaje
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x00) != 0) {};
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x38) != 0) {};
}

/**
 * @brief Deshabilita el timeout.
 * 
 * Manda a la controladora la orden de deshabilitar el timeout.
 * Si el timeout esta habilitado el movimiento finaliza a los dos segundos si no se recibe ninguno orden nueva de movimiento, 
 * estando habilitado el moviento se mantiene hasta recibir una nueva orden.
 *
void habilitarTimeOut(){
	// Mandamos los bytes del mensaje
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x00) != 0) {};
	while(UART_TransferSendNonBlocking(UART4, &g_uartHandle, 0x39) != 0) {};
}

/**
 * @brief Envia las coordenadas.
 * 
 * Envia las coordenadas de la posicion al modulo bluetooth.
 * @param x Valor de la coordenada X
 * @param y Valor de la coordenada Y
 */
/*void enviarPosicion(int x, int y){
	// Enviamos una X
	while(BLUETOOTH_SendChar('X') != 0) {};
	// Comprobamos si el valor es positivo o negativo
	if (x < 0){
		// Enviamos el signo negativo
		while(BLUETOOTH_SendChar('-') != 0) {};
	} else {
		// Enviamos un 0
		while(BLUETOOTH_SendChar('0') != 0) {};
	}
	// Enviamos digito a digito el numero
	while(BLUETOOTH_SendChar((char)(48+((int)abs(x)/10000)))!= 0) {};
	while(BLUETOOTH_SendChar((char)(48+((int)abs(x)%10000)/1000))!= 0) {};
	while(BLUETOOTH_SendChar((char)(48+((int)abs(x)%1000)/100))!= 0) {};
	while(BLUETOOTH_SendChar((char)(48+((int)abs(x)%100)/10))!= 0) {};
	while(BLUETOOTH_SendChar((char)(48+((int)abs(x)%10)))!= 0) {};
	// Enviamos una Y
	while(BLUETOOTH_SendChar('Y') != 0) {};
	// Comprobamos si el valor es positivo o negativo
	if (y < 0){
		// Enviamos el signo negativo
		while(BLUETOOTH_SendChar('-') != 0) {};
	} else {
		// Enviamos un 0
		while(BLUETOOTH_SendChar('0') != 0) {};
	}
	// Enviamos digito a digito el numero
	while(BLUETOOTH_SendChar((char)(48+((int)abs(y)/10000)))!= 0) {};
	while(BLUETOOTH_SendChar((char)(48+((int)abs(y)%10000)/1000))!= 0) {};
	while(BLUETOOTH_SendChar((char)(48+((int)abs(y)%1000)/100))!= 0) {};
	while(BLUETOOTH_SendChar((char)(48+((int)abs(y)%100)/10))!= 0) {};
	while(BLUETOOTH_SendChar((char)(48+((int)abs(y)%10)))!= 0) {};
}*/
