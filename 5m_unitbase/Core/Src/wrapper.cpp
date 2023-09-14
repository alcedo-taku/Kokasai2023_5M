#include "wrapper.hpp"

/* Include Begin */
#include "main.h"
#include "HAL_Extension.hpp"
#include <array>
/* Include End */

/* Enum Begin */
/* Enum End */

/* Struct Begin */
/* Struct End */

/* Variable Begin */
// モータ
std::array<halex::Motor, 7> motor = {
		halex::Motor(&htim4,  TIM_CHANNEL_1, &htim4,  TIM_CHANNEL_2), // 0 1
		halex::Motor(&htim1,  TIM_CHANNEL_1, &htim1,  TIM_CHANNEL_2), // 1 2
		halex::Motor(&htim8,  TIM_CHANNEL_1, &htim8,  TIM_CHANNEL_2), // 2 3
		halex::Motor(&htim15, TIM_CHANNEL_1, &htim15, TIM_CHANNEL_2), // 3 4
};

// エンコーダ
std::array<halex::Encoder, 2> encoder = {
		halex::Encoder(&htim2),
		halex::Encoder(&htim3)
};
std::array<int32_t, 2> encoder_count;
std::array<int32_t, 2> prev_encoder_count;

/* Variable End */

/* Class Constructor Begin */
/* Class Constructor End */

/* Function Prototype Begin */
/* Function Prototype End */

void init(void){
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO4_GPIO_Port, GPIO4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO5_GPIO_Port, GPIO5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO6_GPIO_Port, GPIO6_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
//	HAL_TIM_Base_Start_IT(&htim7);
}

void loop(void){

}

/* Function Body Begin */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//割り込みの処理内容
	if(htim == &htim7){

	}
}
/* Function Body End */
