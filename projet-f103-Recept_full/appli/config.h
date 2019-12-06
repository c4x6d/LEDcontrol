/*
 * config.h
 *
 *  Created on: 31 mars 2016
 *      Author: Nirgal
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stm32f1xx_hal.h"

//Choix de la cible utilisee.
#define NUCLEO	1
#define BLUEPILL 0

//_______________________________________________________
//Configuration des broches utilisees...
//Ces macros permettent d'utiliser dans le code des noms explicites (LED_GREEN, ...)
#if NUCLEO
	#define LED_GREEN_GPIO		GPIOA
	#define LED_GREEN_PIN		GPIO_PIN_5
	#define BLUE_BUTTON_GPIO	GPIOC
	#define BLUE_BUTTON_PIN 	GPIO_PIN_13
	#if BLUEPILL
		#error "Vous ne pouvez pas definir a la fois NUCLEO et BLUEPILL !"
	#endif
#endif
#if BLUEPILL
	#define LED_GREEN_GPIO		GPIOC
	#define LED_GREEN_PIN		GPIO_PIN_13
	#define BLUE_BUTTON_GPIO	GPIOA
	#define BLUE_BUTTON_PIN 	GPIO_PIN_15		//Pas de bouton sur la bluepill, choisissez le port souhaite si vous en reliez un.
#endif

//Choisir les broches pour l'UART1, parmi ces deux possibilites :
#define UART1_ON_PB6_PB7
//#define UART1_ON_PA9_PA10

//Choisir les broches pour l'UART2, parmi ces deux possibilites :
#define UART2_ON_PA2_PA3
//#define UART2_ON_PD5_PD6

//Choisir les broches pour l'UART3, parmi ces deux possibilites :
#define UART3_ON_PB10_PB11
//#define UART3_ON_PD8_PD9

//_______________________________________________________


//Ci dessous, les defines permettant d'activer les modules de la librairie.
//Attention, s'ils sont tous actives, le code compile est trop volumineux pour rentrer dans la memoire de 64kio du STM32F103C8T6 la Bluepill.
//Le STM32F103RBT6 de la carte Nucleo dispose quant a lui d'une memoire de 128kio de flash.


#define USE_BSP_TIMER			1 //Utilisation de stm32f1_timer.c/h


#define USE_ADC					0
	//Ces configurations permettent d'activer les entrees analogiques souhaitees.
	//16 entrees analogiques peuvent etre activees maximum.
	//2 entrees analogiques doivent etre activees minimum. (Vref est un choix possible si vous utilisez une seule entree)
	#define USE_AN0			1	//Broche correspondante : PA0
	#define USE_AN1			1	//Broche correspondante : PA1
	#define USE_AN2			0	//Broche correspondante : PA2	//Sur la Nucleo, cette broche n'est pas cablee !
	#define USE_AN3			0	//Broche correspondante : PA3	//Sur la Nucleo, cette broche n'est pas cablee !
	#define USE_AN4			0	//Broche correspondante : PA4
	#define USE_AN5			0	//Broche correspondante : PA5
	#define USE_AN6			0	//Broche correspondante : PA6
	#define USE_AN7			0	//Broche correspondante : PA7
	#define USE_AN8			0	//Broche correspondante : PB0
	#define USE_AN9			0	//Broche correspondante : PB1
	#define USE_AN10		0	//Broche correspondante : PC0	//Sur la Bluepill, cette broche n'est pas cablee !
	#define USE_AN11		0	//Broche correspondante : PC1	//Sur la Bluepill, cette broche n'est pas cablee !
	#define USE_AN12		0	//Broche correspondante : PC2	//Sur la Bluepill, cette broche n'est pas cablee !
	#define USE_AN13		0	//Broche correspondante : PC3	//Sur la Bluepill, cette broche n'est pas cablee !
	#define USE_AN14		0	//Broche correspondante : PC4	//Sur la Bluepill, cette broche n'est pas cablee !
	#define USE_AN15		0	//Broche correspondante : PC5	//Sur la Bluepill, cette broche n'est pas cablee !
	#define USE_AN16		0	//Capteur de temperature interne
	#define USE_AN17		1	//Vref


#define USE_SCREEN_TFT_ILI9341		0	//Ecran TFT 2.4
	#if USE_SCREEN_TFT_ILI9341
		#define USE_XPT2046			1	//Tactile

		#define USE_FONT11x18		0
		#define USE_FONT7x10		1
		#define USE_FONT16x26		0
	#endif


#define USE_SCREEN_LCD2X16			0	//Ecran LCD 2x16

#define USE_TFT_PCD8544				0	//Ecran TFT Nokia

//Carte capteur IKS01A1 :
#define USE_SENSOR_HTS221		0//Humidite et temperature
#define USE_SENSOR_LPS25HB		0//Pression (et temperature)
#define USE_SENSOR_LSM6DS0		0//Accelerometre et Gyroscope
#define USE_SENSOR_LIS3MDL		0//Magnetometre

//Autres capteurs :
#define USE_SENSOR_LSM6DS3		0//Accelerometre et Gyroscope
#define USE_SENSOR_LPS22HB		0//Pression (et temperature)
#define USE_MLX90614			0	//Capteur de temperature sans contact
#define USE_MPU6050				0//Accelerometre et Gyroscope

#define USE_MATRIX_KEYBOARD		0
#define USE_MATRIX_LED			1
#define USE_CAPACITIVE_KEYBOARD	0
#define USE_MATRIX_LED_32_32	1
#define USE_FINGERPRINT			0

#define USE_MCP23S17			0 //Port expander I/O en SPI
#define USE_MCP23017			0 //Port expander I/O en I2C
#define USE_APDS9960			0	//Capteur de gestes, de couleur, de luminosite ambiante
#define USE_BH1750FVI			0	//Capteur de luminosite ambiante
#define USE_BMP180				0	//Capteur de pression
#define USE_MOTOR_DC			0
#define USE_RTC					0
#define USE_PWM					0
#define USE_ESP8266				0//Module Wifi
#define USE_NFC03A1				0
#define USE_EPAPER				0

#if USE_EPAPER
	#define RST_Pin 			GPIO_PIN_1
	#define RST_GPIO_Port 		GPIOA
	#define DC_Pin 				GPIO_PIN_1
	#define DC_GPIO_Port 		GPIOC
	#define BUSY_Pin 			GPIO_PIN_0
	#define BUSY_GPIO_Port 		GPIOB
	#define SPI_CS_Pin 			GPIO_PIN_4
	#define SPI_CS_GPIO_Port 	GPIOA
	#define EPAPER_SPI			SPI1
#endif

#define USE_SD_CARD				0	//Utilisation d'une carte SD pour lire/ecrire des fichiers


//Liste des modules utilisant le peripherique I2C
#if USE_MLX90614 || USE_MPU6050	|| USE_APDS9960	 || USE_BH1750FVI || USE_BMP180 || USE_MCP23017
	#define USE_I2C				1
#endif
#define I2C_TIMEOUT				5	//ms

#define I2C1_ON_PB6_PB7			0

//Liste des modules utilisant le peripherique SPI
#if USE_SCREEN_TFT_ILI9341	|| USE_SD_CARD
	#define USE_SPI				1
#endif

#endif /* CONFIG_H_ */
