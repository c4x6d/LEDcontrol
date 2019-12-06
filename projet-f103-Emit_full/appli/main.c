/**
  ******************************************************************************
  * @file    main.c
  * @author  Nirgal
  * @date    03-July-2019
  * @brief   Default main function.
  ******************************************************************************
*/
#include "stm32f1xx_hal.h"
#include "stm32f1_uart.h"
#include "stm32f1_sys.h"
#include "stm32f1_gpio.h"
#include "macro_types.h"
#include "systick.h"
#include "stm32f1_pwm.h"


#define pwm_off() PWM_set_duty(TIMER1_ID, TIM_CHANNEL_1, 0)
#define pwm_on() PWM_set_duty(TIMER1_ID, TIM_CHANNEL_1, 50)
#define BLUE_UP	GPIOA, GPIO_PIN_9
#define BLUE_DOWN	GPIOA, GPIO_PIN_10
#define GREEN_UP	GPIOA, GPIO_PIN_11
#define GREEN_DOWN	GPIOB, GPIO_PIN_6
#define RED_UP	GPIOB, GPIO_PIN_7
#define RED_DOWN	GPIOB, GPIO_PIN_8
#define ON_OFF	GPIOB, GPIO_PIN_9
#define RED	GPIOB, GPIO_PIN_10
#define GREEN	GPIOB, GPIO_PIN_11
#define BLUE	GPIOB, GPIO_PIN_12
#define EFFECT	GPIOB, GPIO_PIN_13
#define INT_UP	GPIOB, GPIO_PIN_14
#define INT_DOWN	GPIOB, GPIO_PIN_15
#define LED_EMIT	GPIOA, GPIO_PIN_8
#define LED_ACTIVE	GPIOA, GPIO_PIN_15

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

#define ReadButton(x)	(!HAL_GPIO_ReadPin(x))


void writeLED(bool_e b)
{
	HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, b);
}

bool_e readButton(void)
{

	return !HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO, BLUE_BUTTON_PIN);
}

static void button_detection(void);
static void state_machine_emitter_process_ms(void);

#define LED_ACTIVE_DURATION	500

static volatile uint32_t t = 0;
static volatile uint32_t t_led_active = 0;

void process_ms(void)
{
	if(t)
		t--;
	state_machine_emitter_process_ms();
	if(t_led_active)
	{
		t_led_active--;
		if(!t_led_active)
			HAL_GPIO_WritePin(LED_ACTIVE, 0);
	}

}

volatile uint8_t code = 128;


typedef enum
{
	BUTTON_NONE = 0,
	BUTTON_ON_OFF,
	BUTTON_RED,
	BUTTON_GREEN,
	BUTTON_BLUE,
	BUTTON_RED_UP,
	BUTTON_RED_DOWN,
	BUTTON_GREEN_UP,
	BUTTON_GREEN_DOWN,
	BUTTON_BLUE_UP,
	BUTTON_BLUE_DOWN,
	BUTTON_INT_UP,
	BUTTON_INT_DOWN,
	BUTTON_EFFECT,
	BUTTON_NB
}button_id_e;


button_id_e button_press_event(void)
{
	static bool_e previous_state[BUTTON_NB] = {FALSE};
	button_id_e ret = BUTTON_NONE;
	button_id_e button_id;
	bool_e current_state;
	for (button_id = BUTTON_ON_OFF; button_id < BUTTON_NB; button_id++)
	{
		switch(button_id)
		{
			case BUTTON_ON_OFF:
				current_state = ReadButton(ON_OFF);
				break;
			case BUTTON_RED:
				current_state = ReadButton(RED);
				break;
			case BUTTON_GREEN:
				current_state = ReadButton(GREEN);
				break;
			case BUTTON_BLUE:
				current_state = ReadButton(BLUE);
				break;
			case BUTTON_RED_UP:
				current_state = ReadButton(RED_UP);
				break;
			case BUTTON_RED_DOWN:
				current_state = ReadButton(RED_DOWN);
				break;
			case BUTTON_GREEN_UP:
				current_state = ReadButton(GREEN_UP);
				break;
			case BUTTON_GREEN_DOWN:
				current_state = ReadButton(GREEN_DOWN);
				break;
			case BUTTON_BLUE_UP:
				current_state = ReadButton(BLUE_UP);
				break;
			case BUTTON_BLUE_DOWN:
				current_state = ReadButton(BLUE_DOWN);
				break;
			case BUTTON_INT_UP:
				current_state = ReadButton(INT_UP);
				break;
			case BUTTON_INT_DOWN:
				current_state = ReadButton(INT_DOWN);
				break;
			case BUTTON_EFFECT:
				current_state = ReadButton(EFFECT);
				break;
			default:
				current_state = 0;
				break;
		}
		if(current_state && !previous_state[button_id]){
			ret = button_id;
		}
		previous_state[button_id] = current_state;
	}
	return ret;
}


int main(void)
{
	//Initialisation de la couche logicielle HAL (Hardware Abstraction Layer)
	//Cette ligne doit rester la première étape de la fonction main().
	HAL_Init();


	//Initialisation de l'UART2 à la vitesse de 115200 bauds/secondes (92kbits/s) PA2 : Tx  | PA3 : Rx.
		//Attention, les pins PA2 et PA3 ne sont pas reliées jusqu'au connecteur de la Nucleo.
		//Ces broches sont redirigées vers la sonde de débogage, la liaison UART étant ensuite encapsulée sur l'USB vers le PC de développement.
	UART_init(UART2_ID,115200);

	//"Indique que les printf sortent vers le périphérique UART2."
	SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

	//Initialisation du port de la led Verte (carte Nucleo)
	BSP_GPIO_PinCfg(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(LED_ACTIVE, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);
	//Initialisation du port du bouton bleu (carte Nucleo)
	BSP_GPIO_PinCfg(BLUE_BUTTON_GPIO, BLUE_BUTTON_PIN, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(BLUE_UP, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(BLUE_DOWN, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(GREEN_UP, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(GREEN_DOWN, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(RED_UP, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(RED_DOWN, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(ON_OFF, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(RED, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(GREEN, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(BLUE, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(EFFECT, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(INT_UP, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(INT_DOWN, GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_HIGH);
	BSP_GPIO_PinCfg(LED_EMIT, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH);


	PWM_run(TIMER1_ID,TIM_CHANNEL_1, FALSE, 27, 0, FALSE);

	//On ajoute la fonction process_ms à la liste des fonctions appelées automatiquement chaque ms par la routine d'interruption du périphérique SYSTICK
	Systick_add_callback_function(&process_ms);


	while(1)	//boucle de tâche de fond
	{

		if(code == 0)
		{
			button_detection();
		}
	}
}

static void state_machine_emitter_process_ms(void){
	typedef enum
	{
		INIT,
		WAIT_CODE,
		SEND_CODE,
		LAST_BIT,
	}state_e;

	static state_e state = INIT;
	static uint8_t index = 0;
	switch (state)
	{
		case INIT:
			state = WAIT_CODE;
			break;
		case WAIT_CODE:
			if (code != 0){
				pwm_on();
				state = SEND_CODE;
				index = 0;
			}
			break;
		case SEND_CODE:
			if(code & (1<<index)){
				pwm_on();
			}
			else{
				pwm_off();
			}
			index ++;
			if(index == 8){
				state = LAST_BIT;
				code = 0;
			}
			break;
		case LAST_BIT:
			HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, 0);
			pwm_off();
			state= WAIT_CODE;
			break;
	}
}

static void button_detection(void){
	button_id_e event;
	event = button_press_event();
	if(event != BUTTON_NONE && event < BUTTON_NB)
	{
		HAL_GPIO_WritePin(LED_ACTIVE, 1);
		t_led_active = LED_ACTIVE_DURATION;
	}
	switch(event)
	{
		case BUTTON_NONE:
			break;
		case BUTTON_ON_OFF:
			code = POWER_ON_OFF;
			break;
		case BUTTON_RED:
			code = RGB_RED;
			break;
		case BUTTON_GREEN:
			code = RGB_GREEN;
			break;
		case BUTTON_BLUE:
			code = RGB_BLUE;
			break;
		case BUTTON_RED_UP:
			code = RGB_RED_UP;
			break;
		case BUTTON_RED_DOWN:
			code = RGB_RED_DOWN;
			break;
		case BUTTON_GREEN_UP:
			code = RGB_GREEN_UP;
			break;
		case BUTTON_GREEN_DOWN:
			code = RGB_GREEN_DOWN;
			break;
		case BUTTON_BLUE_UP:
			code = RGB_BLUE_UP;
			break;
		case BUTTON_BLUE_DOWN:
			code = RGB_BLUE_DOWN;
			break;
		case BUTTON_INT_UP:
			code = RGB_INT_UP;
			break;
		case BUTTON_INT_DOWN:
			code = RGB_INT_DOWN;
			break;
		case BUTTON_EFFECT:
			code = RUNNING;
			break;
		default:
			break;
	}
}
