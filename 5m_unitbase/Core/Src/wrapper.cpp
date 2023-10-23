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

/* Define Begin */
#define IS_MOTOR_TEST 1
/* Define End */

/* Enum Begin */
/* Enum End */

/* Struct Begin */
/* Struct End */

/* Variable Begin */
uint8_t unit_num = 0;

// モータ
std::array<halex::Motor, 4> motor = {
		halex::Motor(&htim4,  TIM_CHANNEL_1, &htim4,  TIM_CHANNEL_2), // 0 1 射出
		halex::Motor(&htim1,  TIM_CHANNEL_1, &htim1,  TIM_CHANNEL_2), // 1 2 送り
		halex::Motor(&htim8,  TIM_CHANNEL_1, &htim8,  TIM_CHANNEL_2), // 2 3 横移動
		halex::Motor(&htim15, TIM_CHANNEL_1, &htim15, TIM_CHANNEL_2), // 3 4 旋回
};

// エンコーダ
std::array<halex::Encoder, 2> encoder = {
		halex::Encoder(&htim2),
		halex::Encoder(&htim3)
};
std::array<int32_t, 2> encoder_count;
std::array<int32_t, 2> prev_encoder_count;

// ADC
mcp3208::MCP3208 mcp3208_reader(hspi2,SPI2_NSS_GPIO_Port,SPI2_NSS_Pin);
std::array<uint16_t, 8> adc_value_array;

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
DataFromUnitToUnit data_receive_unit;
DataFromUnitToUnit data_transmit_unit;
DataFromMainToUnit data_from_main;
uint8_t debug_count = 0;


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
	HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO4_GPIO_Port, GPIO4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO5_GPIO_Port, GPIO5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO6_GPIO_Port, GPIO6_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);

	// unitbase unmber を設定
	unit_num = (uint8_t)HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin);

    //MD
    for(uint8_t i=0; i < motor.size(); i++){
    	motor[i].start();
    }

	// ADC
	mcp3208_reader.init();

	// エンコーダ
	encoder[0].start();
	encoder[1].start();

	// CAN
	// canの初期設定
	can.init();
	can.setFilterActivationState(ENABLE);
	can.setFilterMode(CAN_FilterMode::PATH_FOUR_TYPE_STD_ID);
	can.setFilterBank(14);
	can.setStoreRxFifo(CAN_RX_FIFO0);
//	can.setFourTypePathId(100, 200, 300, 400);
	can.setFourTypePathId(can_id.main_to_unit, can_id.unit0_to_unit1,can_id.unit1_to_unit0, 100);
	can.setFilterConfig();
	// can 最初の通信？
	can.setDataFrame(CAN_RTR_DATA);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);     // 送信
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 受信
	HAL_CAN_TxMailbox0CompleteCallback(&hcan);



	// タイマー割込み
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);


	debug_count = 0;
}

void loop(void){
//	uint16_t adc_value;
//	mcp3208_reader.update(mcp3208::Channel::CH_0, 0xF);
//	adc_value_array[0] = mcp3208_reader.get(mcp3208::Channel::CH_0);

	mcp3208_reader.update(0xF);
	adc_value_array = mcp3208_reader.get();

	for (uint8_t i = 0; i < 2; i++) {
		encoder[i].update();
		encoder_count[i] = encoder[i].getCount();
	}

}

uint16_t experiment_timer;

/* Function Body Begin */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//割り込みの処理内容
	if(htim == &htim7){
#if IS_MOTOR_TEST
		// MD実験用
//		static uint16_t experiment_timer;
		experiment_timer++;
		switch (experiment_timer) {
			case 1000:
				for(uint8_t i=0; i < motor.size(); i++){
					motor[i].setSpeed(500);
				}
//				servo.set_pulse_width(1000);
//				HAL_GPIO_WritePin(Solenoid_0_GPIO_Port, Solenoid_0_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
				break;
			case 2000:
				for(uint8_t i=0; i < motor.size(); i++){
					motor[i].setSpeed(250);
				}
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
				break;
			case 3000:
				for(uint8_t i=0; i < motor.size(); i++){
					motor[i].setSpeed(-500);
				}
//				servo.set_pulse_width(2000);
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
				break;
			case 4000:
				for(uint8_t i=0; i < motor.size(); i++){
					motor[i].setSpeed(0);
				}
				experiment_timer = 0;
//				HAL_GPIO_WritePin(Solenoid_0_GPIO_Port, Solenoid_0_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
				break;
			default:
				break;
		}
#endif

		/* CAN */
//		switch (unit_num) {
//			case 0:
//				can.setId(CAN_ID_STD, can_id.unit0_to_unit1);
//				break;
//			case 1:
//				can.setId(CAN_ID_STD, can_id.unit1_to_unit0);
//				break;
//			default:
////				ここエラー
//				break;
//		}
//		data_transmit_unit.debug_count++;
//		can_state = can.transmit(sizeof(data_transmit_unit), (uint8_t*)&data_transmit_unit);
		can_state_ = can.getState();
	}
}

//// 送信成功で呼び出される
//void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
//	mailbox0_complete_count ++;
//}
//
//void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
//	mailbox1_complete_count ++;
//}
//
//void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
//	mailbox2_complete_count ++;
//}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	std::array<uint8_t,8>buf{};
	can_state = can.receive(CAN_RX_FIFO0,(uint8_t*)&buf);
	rx_id = can.getRxId();
	if(can_state == CAN_StatusType::HAL_OK){
//		__HAL_TIM_SET_COUNTER(&htim13, 0);
//		disconnect_count = 0;
		if( (can_id.unit1_to_unit0 == can.getRxId() && unit_num == 0) || (can_id.unit0_to_unit1 == can.getRxId() && unit_num == 1) ){
			memcpy(&data_receive_unit,&buf,sizeof(data_receive_unit));
//			debug_count = data_receive_unit.debug_count;
//			ここに通信入れる
		}else if(can_id.main_to_unit == can.getRxId()){
			memcpy(&data_from_main, &buf, sizeof(data_from_main));
		}
	}
}


/* Function Body End */
