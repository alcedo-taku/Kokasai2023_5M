#include "wrapper.hpp"

/* Pre-Processor Begin */
#include <stdint.h>
#include <vector>
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "xbee.hpp"

//#define XBee_AT_MODE
#define XBee_API_MODE
#define DEBUG_MODE

constexpr uint64_t XBeeTargetAddressLow = 0x0013A200419834AA;
/* Pre-Processor End */

/* Enum, Struct Begin */

struct data_t{
#if defined(XBee_AT_MODE)
	uint8_t start = 's';
#endif
	int16_t joystick[4];
	uint8_t sw=0;
#if defined(XBee_AT_MODE)
	uint8_t end = 'e';
#endif
};


struct gpio_t{
	GPIO_TypeDef* GPIOx;
	uint16_t gpio_pin;
};
/* Enum, Struct End */

/* Function Prototype Begin */
void readSW(uint8_t &data);
void readJoystick(int16_t (&data)[4]);
void updateIndicator(data_t &data);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* Function Prototype End */

/* Variable Begin */
#if defined(XBee_API_MODE)
xbee_c<sizeof(data_t)+18> xbee;
#endif

uint16_t adcBuf[4]={};
int16_t adcCenter[4]={};
int16_t adc0Range = 0;
gpio_t readPin[13];
#if defined(XBee_AT_MODE)
uint8_t returnValue = 0;
#elif defined(XBee_API_MODE)

//std::array<uint8_t, xbee.getTransmitStatusPacketSize()> transmitStatusPacket;
uint8_t transmitStatusPacket[xbee.getTransmitStatusPacketSize()];
#endif
#if defined(DEBUG_MODE)
uint8_t debug = false;
uint8_t debugBuf[sizeof(data_t)+18];
#endif
/* Variable End */

void init(void){
	for(uint8_t n=0; n<4; n++){
		HAL_TIM_PWM_Start(&htim2, n<<2);
		HAL_TIM_PWM_Start(&htim3, n<<2);
		__HAL_TIM_SET_COMPARE(&htim2, n<<2, 255);
		__HAL_TIM_SET_COMPARE(&htim3, n<<2, 255);
	}

//set pin array
	readPin[0] = {PE9_GPIO_Port, PE9_Pin};
	readPin[1] = {PF6_GPIO_Port, PF6_Pin};
	readPin[2] = {PF7_GPIO_Port, PF7_Pin};
	readPin[3] = {PB2_GPIO_Port, PB2_Pin};
	readPin[4] = {PA5_GPIO_Port, PA5_Pin};
	readPin[5] = {PA4_GPIO_Port, PA4_Pin};
	readPin[6] = {PB14_GPIO_Port, PB14_Pin};
	readPin[7] = {PB15_GPIO_Port, PB15_Pin};
//	readPin[0] = {PA4_GPIO_Port, PA4_Pin};
//	readPin[1] = {PA5_GPIO_Port, PA5_Pin};
//	readPin[2] = {PA6_GPIO_Port, PA6_Pin};
//	readPin[3] = {PB2_GPIO_Port, PB2_Pin};
//	readPin[4] = {PE8_GPIO_Port, PE8_Pin};
//	readPin[5] = {PE9_GPIO_Port, PE9_Pin};
//	readPin[6] = {PB14_GPIO_Port, PB14_Pin};
//	readPin[7] = {PB15_GPIO_Port, PB15_Pin};
//	readPin[8] = {PD8_GPIO_Port, PD8_Pin};
//	readPin[9] = {PA8_GPIO_Port, PA8_Pin};
//	readPin[10] = {PF6_GPIO_Port, PF6_Pin};
//	readPin[11] = {PF7_GPIO_Port, PF7_Pin};
//	readPin[12] = {PC13_GPIO_Port, PC13_Pin};

//set parameter that relate adc
	adcCenter[0]  = 2048;
	adcCenter[1]  = 2048;
	adcCenter[2]  = 2048;
	adcCenter[3]  = 2048;
	adc0Range = 200;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, 4);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	HAL_Delay(1000);

	for(uint8_t n=0; n<4; n++){
		__HAL_TIM_SET_COMPARE(&htim2, n<<2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, n<<2, 0);
	}
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_TIM_Base_Start_IT(&htim16);
#if defined(XBee_AT_MODE)
	HAL_UART_Receive_IT(&huart1, &returnValue, 1);
#elif defined(XBee_API_MODE)
	HAL_UART_Receive_IT(&huart1, (uint8_t*)transmitStatusPacket, sizeof(transmitStatusPacket));
#endif
}

void loop(void){
}

/* Function Body Begin */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim17){
#if defined(XBee_AT_MODE)
		data_t data;
		readSW(data.sw);
		readJoystick(data.joystick);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_UART_AbortReceive_IT(&huart1);
		while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t*)&data, sizeof(data), 100));
		HAL_UART_Receive_IT(&huart1, &returnValue, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		updateIndicator(data);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
#elif defined(XBee_API_MODE)
		data_t data;
		readSW(data.sw);
		readJoystick(data.joystick);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
		xbee.assemblyTransmitPacket(data, (uint64_t)XBeeTargetAddressLow);
#if defined(DEBUG_MODE)
		for(uint8_t n=0; n<xbee.getBufSize(); n++){
			debugBuf[n] = *(xbee.getBufAddress()+n);
		}
#endif
		HAL_UART_AbortReceive_IT(&huart1);
		while(HAL_OK != HAL_UART_Transmit(&huart1, xbee.getBufAddress(), xbee.getBufSize(), 100));
		HAL_UART_Receive_IT(&huart1, (uint8_t*)transmitStatusPacket, sizeof(transmitStatusPacket));
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		if(xbee.isSuccessTransmitStatus(transmitStatusPacket)){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		updateIndicator(data);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
#endif //XBee_API_MODE

	}else if(htim == &htim16){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1){
#if defined(XBee_AT_MODE)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		__HAL_TIM_SET_COUNTER(&htim16, 0);
		HAL_UART_Receive_IT(&huart1, &returnValue, 1);
#elif defined(XBee_API_MODE)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		__HAL_TIM_SET_COUNTER(&htim16, 0);
		HAL_UART_Receive_IT(&huart1, (uint8_t*)transmitStatusPacket, sizeof(transmitStatusPacket));
#endif
	}
}

void readSW(uint8_t &data){
	data = 0;
	for(uint8_t n=0; n<13; n++){
		data |= (uint16_t)HAL_GPIO_ReadPin(readPin[n].GPIOx, readPin[n].gpio_pin) << n;
	}
}

void readJoystick(int16_t (&data)[4]){
	for(uint8_t n=0; n<4; n++){
		data[n] = adcBuf[n] - adcCenter[n];

		// adc0Range 内の値を0にする
		if ( adc0Range < data[n] ){
			data[n] -= adc0Range;
		}else if ( data[n] < -adc0Range ){
			data[n] += adc0Range;
		}else {
			data[n] = 0;
		}
	}
}

void updateIndicator(data_t &data){
	if(data.joystick[0]>0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, data.joystick[0]/32);
	}else if(data.joystick[0]<0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -data.joystick[0]/32);
	}else{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	}

	if(data.joystick[1]>0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, data.joystick[1]/32);
	}else if(data.joystick[1]<0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -data.joystick[1]/32);
	}else{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	}

	if(data.joystick[2]>0){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, data.joystick[2]/32);
	}else if(data.joystick[2]<0){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -data.joystick[2]/32);
	}else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	}

	if(data.joystick[3]>0){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, data.joystick[3]/32);
	}else if(data.joystick[3]<0){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -data.joystick[3]/32);
	}else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	}

	// 非常停止
	if( HAL_GPIO_ReadPin(readPin[0].GPIOx, readPin[0].gpio_pin) == GPIO_PIN_RESET ){
		HAL_GPIO_WritePin(External_LED_2_GPIO_Port, External_LED_2_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(External_LED_2_GPIO_Port, External_LED_2_Pin, GPIO_PIN_RESET);
	}
}

/* Function Body End */
