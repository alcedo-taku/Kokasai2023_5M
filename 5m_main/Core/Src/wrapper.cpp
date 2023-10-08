#include "wrapper.hpp"

/* Include Begin */
#include "main.h"
#include "HAL_Extension.hpp"
#include "can_user/can_user.hpp"
//#include "simple_can_user/simple_can_user.hpp"
#include "mcp3208.hpp"
#include <array>
#include <string.h>
#include "data_type.hpp"
/* Include End */

/* Enum Begin */
/* Enum End */

/* Struct Begin */
/* Struct End */

/* Variable Begin */
// CAN
//simpleCanUser can(&hcan);
CanUser can(&hcan);
/*can関連*/
uint32_t mailbox0_complete_count = 0;
uint32_t mailbox1_complete_count = 0;
uint32_t mailbox2_complete_count = 0;
uint8_t can_transmit_count = 1;
uint32_t rx_id;
CAN_StatusType can_state;
HAL_CAN_StateTypeDef can_state_;
uint16_t rx0_callback_count = 0;
uint16_t transmit_frequency = 300; //データの更新周波数
uint8_t number_of_id = 8;
DataFromMainToUnit data_to_unit;
//DataFromUnitToUnit data__unit;
uint8_t debug_count = 0;

/* Variable End */

/* Class Constructor Begin */
/* Class Constructor End */

/* Function Prototype Begin */
/* Function Prototype End */

void init(void){
//	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	// CAN
	// canの初期設定
	can.init();
//	can.setFilterActivationState(ENABLE);
//	can.setFilterMode(CAN_FilterMode::PATH_FOUR_TYPE_STD_ID);
//	can.setFilterBank(14);
//	can.setStoreRxFifo(CAN_RX_FIFO0);
//	can.setFourTypePathId(100, 200, 300, 400);
//	can.setFourTypePathId(can_id.main_to_unit, can_id.unit0_to_unit1,can_id.unit1_to_unit0, 100);
//	can.setFilterConfig();
	can.setDataFrame(CAN_RTR_DATA);	// canのデータの構造を決める
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);     // 送信　メールボックスを見るための割り込み
//	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 受信　メールボックスを見るための割り込み
	HAL_CAN_TxMailbox0CompleteCallback(&hcan);							// 送信できたかの確認の割り込み　HAL_CAN_TxMailbox0CompleteCallback


	// タイマー割込み
	HAL_TIM_Base_Start_IT(&htim17);
}

void loop(void){
}




/* Function Body Begin */


// 送信成功で呼び出される
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	mailbox0_complete_count ++;
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	mailbox1_complete_count ++;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//割り込みの処理内容
	if(htim == &htim17){
		static uint16_t experiment_timer;
		experiment_timer++;
		switch (experiment_timer) {
			case 1000:
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				break;
			case 2000:
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				break;
			case 3000:
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				break;
			case 4000:
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				experiment_timer = 0;
				break;
			default:
				break;
		}

		can.setId(CAN_ID_STD, can_id.main_to_unit);
		data_to_unit.debug_count++;
		can_state = can.transmit(sizeof(data_to_unit), (uint8_t*)&data_to_unit);
		can_state_ = can.getState();
	}
}
/* Function Body End */
