#include "wrapper.hpp"

/* Include Begin */
#include "main.h"
/* Include End */

/* Enum Begin */
/* Enum End */

/* Struct Begin */
/* Struct End */

/* Variable Begin */
/* Variable End */

/* Class Constructor Begin */
/* Class Constructor End */

/* Function Prototype Begin */
/* Function Prototype End */

void init(void){
	HAL_TIM_Base_Start_IT(&htim7);
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
