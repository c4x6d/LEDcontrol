/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_nucleo.h"
#include "stm32f1_uart.h"
#include "stm32f1_sys.h"
#include "macro_types.h"
#include "stm32f1_gpio.h"
#include "stm32f1_timer.h"
#include "WS2812S.h"
#include "systick.h"

void writeLED(bool_e b)
{
	HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, b);
}

bool_e readButton(void)
{
	return HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO, BLUE_BUTTON_PIN);
}

#define TSOP_GPIO			GPIOA
#define TSOP_PIN 			GPIO_PIN_8
#define NB_BITS				8
#define NB_LEDS				11
#define ADD_QTY				20
#define INT_QTY				2

#define BLACK				0x000000
#define RED					0x00FF00
#define GREEN				0x0000FF
#define BLUE				0xFF0000
#define YELLOW				0x00FFFF
#define CYAN				0xFF00FF
#define MAGENTA				0xFFFF00
#define RED_BLUE			0x80FF00
#define RED_GREEN			0x00FF80
#define GREEN_RED			0x0080FF
#define GREEN_BLUE			0x8000FF
#define BLUE_GREEN			0xFF0080
#define BLUE_RED			0xFF8000

typedef enum
{
	CODE_NONE = 0x00,
	POWER_ON_OFF = 0x55,
	RGB_RED = 0x56,
	RGB_GREEN = 0x59,
	RGB_BLUE = 0x5A,
	RGB_RED_UP = 0x65,
	RGB_RED_DOWN = 0x66,
	RGB_GREEN_UP = 0x69,
	RGB_GREEN_DOWN = 0x6A,
	RGB_BLUE_UP = 0x95,
	RGB_BLUE_DOWN = 0x96,
	RGB_INT_UP = 0x99,
	RGB_INT_DOWN = 0x9A,
	RUNNING = 0xA5,
}code_e;

static volatile uint8_t received_full_code = 0;
static volatile uint32_t t = 0;

static uint32_t pixels[64];
static int32_t color = 0;
static double color_red;
static double color_green;
static double color_blue;

static bool_e light_off = TRUE;
static bool_e running_active = FALSE;
static int effect_index = -1;

void process_ms(void)
{
	if(t)
	{
		t--;
	}
}

void LEDS_OFF(void)
{
	for(uint8_t i=0; i<NB_LEDS; i++)
	{
		pixels[i] = BLACK;
	}
	LED_MATRIX_display(pixels, NB_LEDS);
}


void LEDS_INT_UP()
{
	if(color_red < 255 && color_green < 255 && color_blue < 255)
	{
		color_red = MIN(255, color_red * INT_QTY);
		color_green = MIN(255, color_green * INT_QTY);
		color_blue = MIN(255, color_blue * INT_QTY);
	}
}
void LEDS_INT_DOWN()
{
	if(color_red > 0.5 || color_green > 0.5 || color_blue > 0.5)
	{
		color_red = MAX(0, color_red / INT_QTY);
		color_green = MAX(0, color_green / INT_QTY);
		color_blue = MAX(0, color_blue / INT_QTY);
	}
}

void LEDS_RUNNING(void)
{
	static uint32_t effect_led[12] = {RED, RED_GREEN, YELLOW, GREEN_RED, GREEN, GREEN_BLUE, CYAN, BLUE_GREEN, BLUE, BLUE_RED, MAGENTA, RED_BLUE};
	uint32_t tempo;
	static bool_e sens;
	static bool_e running_int_up;
	static double red_running;
	static double green_running;
	static double blue_running;
	static bool_e initialize = FALSE;
	static int comp = 0;
	if(!light_off && running_active)
	{
		switch(effect_index)
		{
			case 0 :
				tempo = effect_led[0];
				for(uint8_t i=0; i<NB_LEDS; i++)
				{
					effect_led[i] = effect_led[i+1];
					pixels[i] = effect_led[0];
				}
				effect_led[11] = tempo;
				break;
			case 1 :
				tempo = effect_led[0];
				for(uint8_t i=0; i<NB_LEDS; i++)
				{
					effect_led[i] = effect_led[i+1];
					pixels[i] = effect_led[0];
				}
				effect_led[11] = tempo;
				running_int_up = FALSE;
				HAL_Delay(500);
				break;
			case 2 :
				if(!initialize)
				{
					tempo = effect_led[0];
					for(uint8_t i=0; i<NB_LEDS; i++)
					{
						effect_led[i] = effect_led[i+1];
					}
					effect_led[11] = tempo;
					red_running = (double)(effect_led[0] >> 8 & 255);
					green_running = (double)(effect_led[0] >> 16);
					blue_running = (double)(effect_led[0] & 255);
					while(red_running > 0.5 || green_running > 0.5 || blue_running > 0.5)
					{
						red_running /= INT_QTY;
						green_running /= INT_QTY;
						blue_running /= INT_QTY;
					}
					initialize = TRUE;
				}
				if(MIN(255, (uint8_t)red_running) == 255 || MIN(255, (uint8_t)green_running) == 255 || MIN(255, (uint8_t)blue_running) == 255)
				{
					running_int_up = FALSE;
				}
				else if(MAX(0, (uint8_t)red_running) <= 0.5 && MAX(0, (uint8_t)green_running) <= 0.5 && MAX(0, (uint8_t)blue_running) <= 0.5)
				{
					comp = (comp + 1) % 3;
					if(!comp)
					{
						initialize = FALSE;
					}
					else
					{
						running_int_up = TRUE;
					}
				}
				if(running_int_up)
				{
					red_running *= INT_QTY;
					green_running *= INT_QTY;
					blue_running *= INT_QTY;
				}
				else
				{
					red_running /= INT_QTY;
					green_running /= INT_QTY;
					blue_running /= INT_QTY;
				}
				for(uint8_t i=0;i<NB_LEDS;i++)
				{
					pixels[i] = (uint32_t)((MAX(0, MIN(255, (uint8_t)green_running)) << 16 ) + (MAX(0, MIN(255, (uint8_t)red_running)) << 8) + MAX(0, MIN(255, (uint8_t)blue_running)));
				}
				break;
			case 3 :
				tempo = effect_led[0];
				for(uint8_t i=0; i<NB_LEDS; i++)
				{
					effect_led[i] = effect_led[i+1];
					pixels[i] = effect_led[i+1];
				}
				effect_led[11] = tempo;
				break;
			case 4 :
				tempo = effect_led[0];
				for(uint8_t i=0; i<NB_LEDS; i++)
				{
					effect_led[i] = effect_led[i+1];
				}
				effect_led[11] = tempo;
				for(uint8_t i=0; i<NB_LEDS/2+1; i++)
				{
					pixels[i] = effect_led[i];
					pixels[NB_LEDS-i-1] = effect_led[i];
				}
				break;
			case 5 :
				tempo = effect_led[11];
				for(uint8_t i=0; i<NB_LEDS; i++)
				{
					effect_led[NB_LEDS-i] = effect_led[NB_LEDS-i-1];
				}
				effect_led[0] = tempo;
				for(uint8_t i=0; i<NB_LEDS/2+1; i++)
				{
					pixels[i] = effect_led[i];
					pixels[NB_LEDS-i-1] = effect_led[i];
				}
				sens = TRUE;
				break;
			case 6 :
				if(pixels[0] == RED)
				{
					sens = FALSE;
				}
				else if(pixels[5] == RED)
				{
					sens = TRUE;
				}
				if(sens)
				{
					tempo = effect_led[0];
					for(uint8_t i=0; i<NB_LEDS; i++)
					{
						effect_led[i] = effect_led[i+1];
					}
					effect_led[11] = tempo;
				}
				else
				{
					tempo = effect_led[11];
					for(uint8_t i=0; i<NB_LEDS; i++)
					{
						effect_led[NB_LEDS-i] = effect_led[NB_LEDS-i-1];
					}
					effect_led[0] = tempo;
				}
				for(uint8_t i=0; i<NB_LEDS/2+1; i++)
				{
					pixels[i] = effect_led[i];
					pixels[NB_LEDS-i-1] = effect_led[i];
				}
				break;
			default :
				break;
		}
		LED_MATRIX_display(pixels, NB_LEDS);
		HAL_Delay(100);
	}
}

void FULL_GRADIENT(uint64_t received_full_code)
{
	static double color_red_temp = 255;
	static double color_green_temp = 255;
	static double color_blue_temp = 255;
	running_active = FALSE;
	bool_e order_accepted = TRUE;
	switch (received_full_code)
	{
		case CODE_NONE :
			break;
		case POWER_ON_OFF:
			if(!light_off)
			{
				color_red_temp = color_red;
				color_green_temp = color_green;
				color_blue_temp = color_blue;
				light_off = TRUE;
			}
			else
			{
				color_red = color_red_temp;
				color_green = color_green_temp;
				color_blue = color_blue_temp;
				light_off = FALSE;
			}
			break;
		case RGB_RED_UP :
			color_red = MIN(255, color_red + ADD_QTY);
			break;
		case RGB_RED_DOWN :
			color_red = MAX(0, color_red - ADD_QTY);
			break;
		case RGB_GREEN_UP :
			color_green = MIN(255, color_green + ADD_QTY);
			break;
		case RGB_GREEN_DOWN :
			color_green = MAX(0, color_green - ADD_QTY);
			break;
		case RGB_BLUE_UP :
			color_blue = MIN(255, color_blue + ADD_QTY);
			break;
		case RGB_BLUE_DOWN:
			color_blue = MAX(0, color_blue - ADD_QTY);
			break;
		case RUNNING :
			running_active = TRUE;
			order_accepted = FALSE;
			effect_index = (effect_index + 1) % 7;
			break;
		case RGB_RED :
			color_red = 255;
			color_green = 0;
			color_blue = 0;
			break;
		case RGB_GREEN :
			color_red = 0;
			color_green = 255;
			color_blue = 0;
			break;
		case RGB_BLUE :
			color_red = 0;
			color_green = 0;
			color_blue = 255;
			break;
		case RGB_INT_UP :
			LEDS_INT_UP();
			break;
		case RGB_INT_DOWN :
			LEDS_INT_DOWN();
			break;
		default :
			order_accepted = FALSE;
			break;
	}
	if(order_accepted)
	{
		if(light_off)
		{
			LEDS_OFF();
		}
		else
		{
			color = (MAX(0, MIN(255, (uint8_t)color_green)) << 16 ) + (MAX(0, MIN(255, (uint8_t)color_red)) << 8) + MAX(0, MIN(255, (uint8_t)color_blue));
			if (order_accepted)
			{
				for(uint8_t i=0;i<NB_LEDS;i++)
				{
					pixels[i] = (uint32_t)color;
				}
			}
		}
		LED_MATRIX_display(pixels, NB_LEDS);
	}
}

static volatile int8_t index = -1;

static volatile uint8_t full_code = 0;

void EXTI9_5_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(TSOP_PIN) != RESET){
		__HAL_GPIO_EXTI_CLEAR_IT(TSOP_PIN);
		//On acquitte l'it



		NVIC_DisableIRQ(EXTI9_5_IRQn);
		//TODO allumer le timer !
		index = -1;
		full_code = 0;

		TIMER_run_us(TIMER1_ID, 1500, TRUE);
		clear_it_status(TIMER1_ID);

	}
}

void TIMER1_user_handler_it(void)
{
	if(index==-1)
	{
				TIMER_set_period(TIMER1_ID, 48000);
	}
	else if(index==0)
	{
		TIMER_set_period(TIMER1_ID, 32000);
	}
	//en fonction de l'index ;
	//si index = 0 -> on lance le timer pour la bonne duree
	//si index = le dernier -> on stoppe le timer, on leve le flag pour traiter la trame
			//et on reactive les it. (apres avoir acquite le flag avec __HAL_GPIO_EXTI_CLEAR_IT(TSOP_PIN);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	uint8_t pin;
	pin = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	full_code |= (uint8_t)(pin<<(index));
	index++;
	if(index == NB_BITS)
	{
		received_full_code = full_code;
		TIMER_stop(TIMER1_ID);
		__HAL_GPIO_EXTI_CLEAR_IT(TSOP_PIN);
		NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
}

int main(void)
{
	//Initialisation de la couche logicielle HAL (Hardware Abstraction Layer)
	//Cette ligne doit rester la premiere etape de la fonction main().
	HAL_Init();
	//Initialisation de l'UART2 a la vitesse de 115200 bauds/secondes (92kbits/s) PA2 : Tx  | PA3 : Rx.
		//Attention, les pins PA2 et PA3 ne sont pas reliees jusqu'au connecteur de la Nucleo.
		//Ces broches sont redirigees vers la sonde de debogage, la liaison UART etant ensuite encapsulee sur l'USB vers le PC de developpement.
	UART_init(UART2_ID,115200);
	//"Indique que les printf sortent vers le peripherique UART2."
	SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);
	//Initialisation du port de la led Verte (carte Nucleo)
	BSP_GPIO_PinCfg(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	//Initialisation du port du bouton bleu (carte Nucleo)
	BSP_GPIO_PinCfg(BLUE_BUTTON_GPIO, BLUE_BUTTON_PIN, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	//Broche capteur en entree
	BSP_GPIO_PinCfg(TSOP_GPIO, TSOP_PIN, GPIO_MODE_IT_FALLING,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	Systick_add_callback_function(&process_ms);
	LED_MATRIX_init();
	LEDS_OFF();
	while(1)
	{
		if (received_full_code)
		{
			FULL_GRADIENT(received_full_code);
			received_full_code = 0;
		}
		LEDS_RUNNING();
	}
}
