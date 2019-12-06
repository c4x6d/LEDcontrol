/*
 * test.c
 *
 *  Created on: 3 nov. 2016
 *      Author: Nirgal
 */


/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "config.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_nucleo.h"
#include "stm32f1_uart.h"
#include "stm32f1_sys.h"
#include "stm32f1_pwm.h"
#include "macro_types.h"
#include "stm32f1_timer.h"
#include "MPU6050/stm32f1_mpu6050.h"
#include "stm32f1_gpio.h"
#include "stm32f1_rtc.h"
#include "stm32f1_motorDC.h"
#include "CapacitiveKeyboard/CapacitiveKeyboard.h"
#include "MatrixKeyboard/matrix_keyboard.h"
#include "lcd2x16.h"
#include "stm32f1_adc.h"
#include "stm32f1_ili9341.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "x_nucleo_iks01a1_humidity.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_temperature.h"
#include "stm32f1_xpt2046.h"
#include "WS2812S.h"
#include "MLX90614.h"
#include <math.h>


#if USE_SENSOR_LSM6DS0 | USE_SENSOR_LSM6DS3
	static DrvContextTypeDef *ACCELERO_handle = NULL;
	static DrvContextTypeDef *GYRO_handle = NULL;
	static SensorAxes_t ACC_Value;
	static SensorAxes_t GYR_Value;                  /*!< Gyroscope Value */
#endif

#if USE_SENSOR_LIS3MDL
	static DrvContextTypeDef *MAGNETO_handle = NULL;
	static SensorAxes_t MAG_Value;                  /*!< Magnetometer Value */
#endif
#if USE_SENSOR_HTS221
	static DrvContextTypeDef *HUMIDITY_handle = NULL;
	static DrvContextTypeDef *TEMPERATURE_handle = NULL;
	static float HUMIDITY_Value;           /*!< Humidity Value */
	static float TEMPERATURE_Value;        /*!< Temperature Value */
#endif
#if USE_SENSOR_LPS22HB | USE_SENSOR_LPS25HB
	static float PRESSURE_Value;           /*!< Pressure Value */
	static DrvContextTypeDef *PRESSURE_handle = NULL;
#endif




void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);


//Fonction de test (BLOCANTE !)
void test(void)
{
	//Initialisation du timer 3 a une periode de 1000µs (1ms)
	#if USE_BSP_TIMER
		TIMER_run_us(TIMER3_ID,1000, TRUE);
	#endif
	#if USE_PWM
		PWM_test();
	#endif

	#if USE_SCREEN_TFT_ILI9341
		ILI9341_Init();
	//	ILI9341_DisplayOff();
	//	ILI9341_DisplayOn();
		ILI9341_Fill(ILI9341_COLOR_WHITE);
		ILI9341_DrawCircle(20,20,5,ILI9341_COLOR_BLUE);
		ILI9341_DrawLine(20,20,100,20,ILI9341_COLOR_RED);
		ILI9341_DrawLine(20,20,20,100,ILI9341_COLOR_RED);

		#if USE_FONT7x10
			ILI9341_Putc(110,11,'x',&Font_7x10,ILI9341_COLOR_BLUE,ILI9341_COLOR_WHITE);
			ILI9341_Putc(15,110,'y',&Font_7x10,ILI9341_COLOR_BLUE,ILI9341_COLOR_WHITE);
			ILI9341_Puts(25,200, "chaine 7x10", &Font_7x10, ILI9341_COLOR_BROWN, ILI9341_COLOR_WHITE);
		#endif
		#if USE_FONT11x18
			ILI9341_Puts(25,225, "chaine 11x18", &Font_11x18, ILI9341_COLOR_BROWN, ILI9341_COLOR_WHITE);
		#endif
		#if USE_FONT16x26
			ILI9341_Puts(25,250, "chaine 16x26", &Font_16x26, ILI9341_COLOR_BROWN, ILI9341_COLOR_WHITE);
		#endif

		XPT2046_init();
	#endif
	#if USE_SCREEN_LCD2X16
		LCD2X16_init();
		LCD2X16_printf("essai");
	#endif

	#if USE_MATRIX_LED
		LED_MATRIX_test();
	#endif

	#if USE_MPU6050
		MPU6050_test();
	#endif

	#if USE_SENSOR_LSM6DS0 | USE_SENSOR_LSM6DS3		//Accelerometre et Gyroscope
		// Try to use LSM6DS3 DIL24 if present, otherwise use LSM6DS0 on board
		BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, (void**)&ACCELERO_handle );
		BSP_GYRO_Init( GYRO_SENSORS_AUTO, (void**)&GYRO_handle );
		BSP_ACCELERO_Sensor_Enable( ACCELERO_handle );
		BSP_GYRO_Sensor_Enable( GYRO_handle );
	#endif
	#if USE_SENSOR_LIS3MDL
		// Force to use LIS3MDL
		BSP_MAGNETO_Init( LIS3MDL_0, (void**)&MAGNETO_handle );
		BSP_MAGNETO_Sensor_Enable( MAGNETO_handle );
	#endif
	#if USE_SENSOR_HTS221
		// Force to use HTS221
		BSP_HUMIDITY_Init( HTS221_H_0, (void**)&HUMIDITY_handle );
		BSP_HUMIDITY_Sensor_Enable( HUMIDITY_handle );
		// Force to use HTS221
		BSP_TEMPERATURE_Init( HTS221_T_0, (void**)&TEMPERATURE_handle );
		BSP_TEMPERATURE_Sensor_Enable( TEMPERATURE_handle );
	#endif
	#if USE_SENSOR_LPS22HB | USE_SENSOR_LPS25HB
		// Try to use LPS25HB DIL24 if present, otherwise use LPS25HB on board
		BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, (void**)&PRESSURE_handle );
		BSP_PRESSURE_Sensor_Enable( PRESSURE_handle );
	#endif
	#if USE_BH1750FVI
		BH1750FVI_demo();		//Fonction blocante !
	#endif
	#if USE_APDS9960
		APDS9960_demo_RGB();	//Fonction blocante !
	#endif
	#if USE_ESP8266
		ESP8266_demo();
	#endif

	uint8_t c;
	uint8_t status;
	#if USE_SENSOR_LPS22HB | USE_SENSOR_LPS25HB | USE_SENSOR_HTS221
		int32_t d1, d2;
	#endif
	while(1)
	{




		#if USE_SCREEN_TFT_ILI9341
			#if USE_XPT2046
				static int16_t static_x,static_y;
				int16_t x, y;

				if(XPT2046_getAverageCoordinates(&x, &y, 10, XPT2046_COORDINATE_SCREEN_RELATIVE))
				{
					x = ((int16_t)(240)) - x;
					ILI9341_DrawCircle(static_x,static_y,15,ILI9341_COLOR_WHITE);
					ILI9341_DrawCircle(x,y,15,ILI9341_COLOR_BLUE);
					static_x = x;
					static_y = y;
				}

			#endif
		#endif


		#if USE_SD_CARD
			DEMO_sd_state_machine(FALSE);
		#endif
		#if USE_SENSOR_LSM6DS0 |  USE_SENSOR_LSM6DS3		//Accelerometre et Gyroscope
			if(BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
			{

				BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
				printf("ACC_X: %d, ACC_Y: %d, ACC_Z: %d\n",(int)ACC_Value.AXIS_X, (int)ACC_Value.AXIS_Y, (int)ACC_Value.AXIS_Z);
			}
			if(BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
			{
				BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);
				printf("GYR_X: %d, GYR_Y: %d, GYR_Z: %d\n", (int)GYR_Value.AXIS_X, (int)GYR_Value.AXIS_Y, (int)GYR_Value.AXIS_Z);
			}
		#endif
		#if USE_SENSOR_LIS3MDL
			if(BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
			{
				BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);
				printf("MAG_X: %d, MAG_Y: %d, MAG_Z: %d\n", (int)MAG_Value.AXIS_X, (int)MAG_Value.AXIS_Y, (int)MAG_Value.AXIS_Z);
			}
		#endif
		#if USE_SENSOR_HTS221
			if(BSP_HUMIDITY_IsInitialized(HUMIDITY_handle, &status) == COMPONENT_OK && status == 1)
			{
				BSP_HUMIDITY_Get_Hum(HUMIDITY_handle, &HUMIDITY_Value);
				floatToInt(HUMIDITY_Value, &d1, &d2, 2);
				printf("HUM: %d.%02d\n", (int)d1, (int)d2);
			}
			if(BSP_TEMPERATURE_IsInitialized(TEMPERATURE_handle, &status) == COMPONENT_OK && status == 1)
			{
				BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &TEMPERATURE_Value);
				floatToInt(TEMPERATURE_Value, &d1, &d2, 2);
				printf("TEMP: %d.%02d\n", (int)d1, (int)d2);
			}
		#endif
		#if USE_SENSOR_LPS22HB | USE_SENSOR_LPS25HB
			if(BSP_PRESSURE_IsInitialized(PRESSURE_handle, &status) == COMPONENT_OK && status == 1)
			{
				BSP_PRESSURE_Get_Press(PRESSURE_handle, &PRESSURE_Value);
				floatToInt(PRESSURE_Value, &d1, &d2, 2);
				printf("PRESS: %d.%02d\n", (int)d1, (int)d2);
			}
		#endif

		c = 0;
		if(UART_data_ready(UART1_ID))
		{
			c = UART_get_next_byte(UART1_ID);
			UART_putc(UART1_ID,(uint8_t)(c+1));	//echo de la lettre suivante !
		}

		#if USE_MOTOR_DC
			DEMO_MOTOR_statemachine(FALSE,c);
		#endif
		#if USE_ADC
			DEMO_adc_statemachine();
		#endif
		#if USE_MLX90614
			DEMO_MLX90614_statemachine(FALSE);
		#endif
		#if USE_MATRIX_KEYBOARD
			DEMO_matrix_keyboard_process_main(FALSE);
		#endif
		#if USE_CAPACITIVE_KEYBOARD
			DEMO_CapacitiveKeyboard(FALSE);
		#endif
		#if USE_RTC
			DEMO_RTC_process_main(FALSE);
		#endif
		#if USE_SCREEN_LCD2X16
			LCD2X16_printf("essai");
		#endif

		//routine de surveillance du stack overflow
		/*extern uint8_t _estack;
		extern uint8_t _ebss;
		uint32_t * a;
		for(a = (uint32_t *)(&_ebss+4); a<(uint32_t *)(&_estack); a++)
		{
			if(*a != 0x55555555)
			{
				printf("stack overflow ?! Max stack occupation : %d\n",(uint32_t *)&_estack - a);
				while(1);
			}
		}
		*/

	}
}



/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_int the pointer to the integer part as output
 * @param  out_dec the pointer to the decimal part as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec)
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}


/*
static void Accelero_Sensor_Handler(TMsg *Msg)
{
  int32_t data[6];
  uint8_t status = 0;

  if(BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
  {
    BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);

    if ( DataLoggerActive )
    {
      if(Sensors_Enabled & ACCELEROMETER_SENSOR)
      {
        Serialize_s32(&Msg->Data[15], ACC_Value.AXIS_X, 4);
        Serialize_s32(&Msg->Data[19], ACC_Value.AXIS_Y, 4);
        Serialize_s32(&Msg->Data[23], ACC_Value.AXIS_Z, 4);
      }
    }

    else if ( AutoInit )
    {
      data[0] = ACC_Value.AXIS_X;
      data[1] = ACC_Value.AXIS_Y;
      data[2] = ACC_Value.AXIS_Z;

      sprintf(dataOut, "ACC_X: %d, ACC_Y: %d, ACC_Z: %d\n", (int)data[0], (int)data[1], (int)data[2]);
      HAL_UART_Transmit(&UartHandle, (uint8_t*)dataOut, strlen(dataOut), 5000);
    }
  }
}
*/




