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
//LED
constexpr std::array<GPIO_pin,10> LED = {{
	{L1_GPIO_Port, L1_Pin},		// 試合中は点灯
	{L2_GPIO_Port, L2_Pin},		//
	{L3_GPIO_Port, L3_Pin},
	{L4_GPIO_Port, L4_Pin},
	{L5_GPIO_Port, L5_Pin},		// 的 0-0
	{L6_GPIO_Port, L6_Pin},		// 的 1-0
	{L7_GPIO_Port, L7_Pin},		// 的 0-1
	{L8_GPIO_Port, L8_Pin},		// 的 1-1
	{L9_GPIO_Port, L9_Pin},		// 的 0-2
	{L10_GPIO_Port, L10_Pin} 	// 的 1-2
}};
//GPIO
constexpr std::array<GPIO_pin,10> gpio = {{
	{G1_GPIO_Port, G1_Pin}, 	// スタート/ストップ
	{G2_GPIO_Port, G2_Pin}, 	// リセット
	{G3_GPIO_Port, G3_Pin},
	{G4_GPIO_Port, G4_Pin},
	{G5_GPIO_Port, G5_Pin},
	{G6_GPIO_Port, G6_Pin},
	{G7_GPIO_Port, G7_Pin},
	{G8_GPIO_Port, G8_Pin},
	{G9_GPIO_Port, G9_Pin},
	{G10_GPIO_Port, G10_Pin}
}};
//非常停止
constexpr std::array<GPIO_pin,2> EMG = {{
	{GPIOF, GPIO_PIN_4},
	{GPIOA, GPIO_PIN_4},
}};
std::array<uint8_t, 2> emg_count = {0,0};
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
DataFromUnitToMain data_from_unit;
DataFromUnitToMain data_from_unit1;
DataFromMainToCtrl data_to_ctrl;
DataFromCtrlToMain data_from_ctrl0;
DataFromCtrlToMain data_from_ctrl1;
uint8_t debug_count = 0;
uint32_t mailbox;

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
	// 受信設定
	can.setFilterActivationState(ENABLE); // フィルタを有効化
	can.setFilterMode(CAN_FilterMode::PATH_FOUR_TYPE_STD_ID); // 16bitID リストモード ４種類のIDが追加可能
	can.setFilterBank(14); // どこまでのバンクを使うか
	can.setStoreRxFifo(CAN_RX_FIFO0); // 使うFIFOメモリ＿
	can.setFourTypePathId(CanId::unit0_to_main, CanId::unit1_to_main, CanId::ctrl0_to_main, CanId::ctrl1_to_main); // to main のメッセージid
	can.setFilterConfig(); // フィルターの設定を反映する
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 受信割り込みの有効化
	// 送信設定
	can.setDataFrame(CAN_RTR_DATA);	// canのデータの構造を決める
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);		// 送信　メールボックスを見るための割り込み
	HAL_CAN_TxMailbox0CompleteCallback(&hcan);							// 送信できたかの確認の割り込み　HAL_CAN_TxMailbox0CompleteCallback

	// タイマー割込み
	HAL_TIM_Base_Start_IT(&htim16); // メイン処理用
	HAL_TIM_Base_Start_IT(&htim17); // 通信用

	for (uint8_t i = 0; i < 10; ++i) {
		HAL_GPIO_WritePin(LED[i].GPIOx, LED[i].GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
	}
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
	//　割り込みの処理内容
	if(htim == &htim16){
		// LED点滅
		static uint16_t experiment_timer;
		experiment_timer++;
		switch (experiment_timer) {
			case 1000:
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
				break;
			case 2000:
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
				experiment_timer = 0;
				break;
			default:
				break;
		}

		// スタートストップ
		static uint32_t start_time;
		static uint32_t end_time;
		switch (data_to_unit.game_state) {
			case GameState::STOP:
				if ( !(bool)HAL_GPIO_ReadPin(gpio[0].GPIOx, gpio[0].GPIO_Pin) ) {
					data_to_unit.game_state = GameState::READY;
					start_time = HAL_GetTick() + 3*1000;
				}
				break;
			case GameState::READY:
				if ( start_time <= HAL_GetTick() ) {
					data_to_unit.game_state = GameState::START;
					end_time = HAL_GetTick() + 120*1000;
				}
				break;
			case GameState::START:
				if ( end_time - 4*1000 <= HAL_GetTick() ){
					data_to_unit.game_state = GameState::END_READY;
				}
				break;
			case GameState::END_READY:
				if ( end_time <= HAL_GetTick() ){
					data_to_unit.game_state = GameState::STOP;
				}
				break;
		}


	}else if(htim == &htim17){
		// CAN送信
		static uint8_t can_transmit_count = 0;
		switch(can_transmit_count){
			case 0:
				// to unit
				can.setId(CAN_ID_STD, CanId::main_to_unit);
//				data_to_unit.debug_count++;
				can_state = can.transmit(sizeof(data_to_unit), (uint8_t*)&data_to_unit);
				can_state_ = can.getState();
				mailbox = can.getUsedTxMailbox();
				can_transmit_count++;
				break;
			case 1:
				// to controller
				can.setId(CAN_ID_STD, CanId::main_to_ctrl);
				data_to_ctrl.debug_count++;
				can_state = can.transmit(sizeof(data_to_ctrl), (uint8_t*)&data_to_ctrl);
				can_state_ = can.getState();
				mailbox = can.getUsedTxMailbox();
				can_transmit_count++;
				break;
			case 2:
				can_transmit_count++;
				break;
			case 3:
				can_transmit_count = 0; // ラストは0にする
				break;
		}
		for (uint8_t i = 0; i < 2; i++) {
			emg_count[i]++;
			if (emg_count[i] >= 254) {
				// 非常停止かける
				HAL_GPIO_WritePin(EMG[i].GPIOx, EMG[i].GPIO_Pin, GPIO_PIN_RESET);
			}
		}
	}
}

// CAN受信
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	std::array<uint8_t,8>buf{};
	can_state = can.receive(CAN_RX_FIFO0,(uint8_t*)&buf);
	rx_id = can.getRxId();
	if(can_state == CAN_StatusType::HAL_OK){
//		__HAL_TIM_SET_COUNTER(&htim13, 0);
//		disconnect_count = 0;
		switch (can.getRxId()) {
			// from unit
			case CanId::unit0_to_main:
				memcpy(&data_from_unit,&buf,sizeof(data_from_unit));
				emg_count[0] = 0;
				if (data_to_unit.game_state == GameState::START || data_to_unit.game_state == GameState::END_READY) {
					HAL_GPIO_WritePin(EMG[0].GPIOx, EMG[0].GPIO_Pin, GPIO_PIN_SET);
				}else{
					HAL_GPIO_WritePin(EMG[0].GPIOx, EMG[0].GPIO_Pin, GPIO_PIN_RESET);
				}
				break;
			case CanId::unit1_to_main:
				memcpy(&data_from_unit1, &buf, sizeof(data_from_unit1));
				emg_count[1] = 0;
				if (data_to_unit.game_state == GameState::START || data_to_unit.game_state == GameState::END_READY) {
					HAL_GPIO_WritePin(EMG[1].GPIOx, EMG[1].GPIO_Pin, GPIO_PIN_SET);
				}else{
					HAL_GPIO_WritePin(EMG[1].GPIOx, EMG[1].GPIO_Pin, GPIO_PIN_RESET);
				}
				break;

			// from controller
			case CanId::ctrl0_to_main:
				memcpy(&data_from_ctrl0, &buf, sizeof(data_from_ctrl0));
				break;
			case CanId::ctrl1_to_main:
				memcpy(&data_from_ctrl1, &buf, sizeof(data_from_ctrl1));
				break;
		}
	}
	// 通信確認インジケータ
	static uint8_t blink_count = 0;
	blink_count++;
	if(blink_count == 100){
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		blink_count = 0;
	}
}


/* Function Body End */
