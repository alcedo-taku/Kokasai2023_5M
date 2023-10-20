/**
 * LEDの仕様
 * 0:受信成功中は点灯
 * 1:受信成功中は点滅
 * 2:送信処理中は点灯
 * 3:送信のタイマー割り込みで点滅
 */

#include "wrapper.hpp"

/* Pre-Processor Begin */
#include <stdint.h>
#include <array>
#include <string.h>
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "xbee.hpp"
#include "PwmSounds.hpp"
#include "data_type.hpp"
#include "can_user/can_user.hpp"

//#define XBee_AT_MODE
#define XBee_API_MODE 1
#define XBee_AT_MODE 0
#define XBee_DEBUG_MODE 1
#define XBee 0
#define CAN 1

constexpr uint64_t XBeeTargetAddressLow = 0x0013A200419834AA;
/* Pre-Processor End */

/* Enum, Struct Begin */

struct data_t{
#if XBee_AT_MODE
	uint8_t start = 's';
#endif
	int16_t joystick[4];
	uint8_t sw=0;
#if XBee_AT_MODE
	uint8_t end = 'e';
#endif
};

/* Enum, Struct End */

/* Function Prototype Begin */
void readSW(uint8_t &data);
void readJoystick(int16_t (&data)[4]);
void updateIndicator(data_t &data);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* Function Prototype End */

/* Variable Begin */
uint8_t ctrl_num = 0;
uint16_t adcBuf[4] = {};
int16_t adcCenter[4] = {};
int16_t adc0Range = 0;
constexpr std::array<GPIO_pin,4> led_pin = {{
	{LED_0_GPIO_Port, LED_0_Pin},
	{LED_1_GPIO_Port, LED_1_Pin},
	{LED_2_GPIO_Port, LED_2_Pin},
	{LED_3_GPIO_Port, LED_3_Pin},
}};
constexpr std::array<GPIO_pin,13> gpio_pin = {{
	// 回路上の印字と実際の配線が異なるので注意
	{GPIOA, GPIO_PIN_4},	// pwm スピーカー
	{GPIOA, GPIO_PIN_5},
	{GPIOA, GPIO_PIN_6},
	{GPIOB, GPIO_PIN_2},
	{GPIOE, GPIO_PIN_8},	// out lock on
	{GPIOE, GPIO_PIN_9},	// out locked on
	{GPIOF, GPIO_PIN_1},	// out 的0 LED
	{GPIOF, GPIO_PIN_0},	// out 的1　LED
	{GPIOC, GPIO_PIN_15},	// out 的2 LED
	{GPIOC, GPIO_PIN_14},	// in
	{GPIOC, GPIO_PIN_13},	// in
	{GPIOF, GPIO_PIN_6},	// in トリガー
	{GPIOF, GPIO_PIN_7}		// in コントローラの番号を決める
}};
DataFromUnitToCtrl data_from_unit;
DataFromCtrlToUnit data_to_unit;
DataFromMainToCtrl data_from_main;
DataFromCtrlToMain data_to_main;
#if XBee
#if XBee_API_MODE
xbee_c<sizeof(data_t)+18> xbee;
#endif
#if XBee_AT_MODE
uint8_t returnValue = 0;
#elif XBee_API_MODE

//std::array<uint8_t, xbee.getTransmitStatusPacketSize()> transmitStatusPacket;
uint8_t transmitStatusPacket[xbee.getTransmitStatusPacketSize()];
#endif
#if XBee_DEBUG_MODE
uint8_t debug = false;
uint8_t debugBuf[sizeof(data_t)+18];
#endif
#endif
#if CAN
CanUser can(&hcan);
uint32_t mailbox0_complete_count = 0;
uint32_t mailbox1_complete_count = 0;
uint32_t mailbox2_complete_count = 0;
uint8_t can_transmit_count = 1;
uint32_t rx_id; // debug用
CAN_StatusType can_state;
HAL_CAN_StateTypeDef can_state_;
uint16_t rx0_callback_count = 0;
uint16_t transmit_frequency = 300; //データの更新周波数
uint8_t number_of_id = 8;
uint8_t debug_count = 0;
#endif

/* Variable End */

void init(void){
	// LEDを全部つける
	for (uint8_t i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(led_pin[i].GPIOx, led_pin[i].GPIO_Pin, GPIO_PIN_SET);
	}
	for(uint8_t n=0; n<4; n++){
		HAL_TIM_PWM_Start(&htim2, n<<2);
		HAL_TIM_PWM_Start(&htim3, n<<2);
		__HAL_TIM_SET_COMPARE(&htim2, n<<2, 255);
		__HAL_TIM_SET_COMPARE(&htim3, n<<2, 255);
	}

	// controller number を設定
	ctrl_num = (uint8_t)HAL_GPIO_ReadPin(gpio_pin[12].GPIOx, gpio_pin[12].GPIO_Pin);

	HAL_Delay(1000);

//set parameter that relate adc
	adcCenter[0]  = 2048;
	adcCenter[1]  = 2048;
	adcCenter[2]  = 2048;
	adcCenter[3]  = 2048;
	adc0Range = 200;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, 4);


	HAL_Delay(1000);

	HAL_TIM_Base_Start_IT(&htim17); // 送信用タイマー割り込み
	HAL_TIM_Base_Start_IT(&htim16); // 受信失敗時に消灯するためのタイマー割り込み

#if XBee
#if XBee_AT_MODE
	HAL_UART_Receive_IT(&huart1, &returnValue, 1);
#elif XBee_API_MODE
	HAL_UART_Receive_IT(&huart1, (uint8_t*)transmitStatusPacket, sizeof(transmitStatusPacket));
#endif
#endif

#if CAN
	// CAN
	// CANの初期設定
	can.init();
	// 受信設定
	can.setFilterActivationState(ENABLE); // フィルタを有効化
	can.setFilterMode(CAN_FilterMode::PATH_FOUR_TYPE_STD_ID); // 16bitID リストモード ４種類のIDが追加可能
	can.setFilterBank(14); // どこまでのバンクを使うか
	can.setStoreRxFifo(CAN_RX_FIFO0); // 使うFIFOメモリ＿
	if (ctrl_num == 0) {
//		can.setFourTypePathId(400, can_id.unit0_to_ctrl0, 200, 100);
		can.setFourTypePathId(can_id.main_to_ctrl, can_id.unit0_to_ctrl0, 200, 100);
	}else if (ctrl_num == 1){
//		can.setFourTypePathId(400, can_id.unit1_to_ctrl1, 200, 100);
		can.setFourTypePathId(can_id.main_to_ctrl, can_id.unit1_to_ctrl1, 200, 100);
	}
	can.setFilterConfig(); // フィルターの設定を反映する
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 受信割り込みの有効化
	// 送信設定
	can.setDataFrame(CAN_RTR_DATA); // メッセージのフレームタイプをデータフレームに設定する
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);     // 送信割り込みの有効化
	HAL_CAN_TxMailbox0CompleteCallback(&hcan);
#endif

	// LEDを全部消す
	for (uint8_t i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(led_pin[i].GPIOx, led_pin[i].GPIO_Pin, GPIO_PIN_RESET);
	}
	for(uint8_t n=0; n<4; n++){
		__HAL_TIM_SET_COMPARE(&htim2, n<<2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, n<<2, 0);
	}
}


void loop(void){
}


/* Function Body Begin */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim17){
		data_t data;
		readSW(data.sw);
		readJoystick(data.joystick);
		HAL_GPIO_WritePin(led_pin[2].GPIOx, led_pin[2].GPIO_Pin, GPIO_PIN_SET);

#if XBee
#if XBee_AT_MODE
		HAL_UART_AbortReceive_IT(&huart1);
		while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t*)&data, sizeof(data), 100));
		HAL_UART_Receive_IT(&huart1, &returnValue, 1);
#elif XBee_API_MODE
		xbee.assemblyTransmitPacket(data, (uint64_t)XBeeTargetAddressLow);
#if XBee_DEBUG_MODE
		for(uint8_t n=0; n<xbee.getBufSize(); n++){
			debugBuf[n] = *(xbee.getBufAddress()+n);
		}
#endif
		HAL_UART_AbortReceive_IT(&huart1);
		while(HAL_OK != HAL_UART_Transmit(&huart1, xbee.getBufAddress(), xbee.getBufSize(), 100));
		HAL_UART_Receive_IT(&huart1, (uint8_t*)transmitStatusPacket, sizeof(transmitStatusPacket));
//		HAL_GPIO_WritePin(led_pin[2].GPIOx, led_pin[2].GPIO_Pin, GPIO_PIN_RESET);
		if(xbee.isSuccessTransmitStatus(transmitStatusPacket)){
//			HAL_GPIO_WritePin(led_pin[0].GPIOx, led_pin[0].GPIO_Pin, GPIO_PIN_SET);
		}else{
//			HAL_GPIO_WritePin(led_pin[0].GPIOx, led_pin[0].GPIO_Pin, GPIO_PIN_RESET);
		}
#endif //XBee_API_MODE
#endif

#if CAN
		/* CAN 送信 */
		static uint8_t can_transmit_count = 0;
		switch(can_transmit_count){
			case 0:
				// to unit
				if (ctrl_num == 0) {
					can.setId(CAN_ID_STD, can_id.ctrl0_to_unit0);
				}else if (ctrl_num == 1){
					can.setId(CAN_ID_STD, can_id.ctrl1_to_unit1);
				}
				data_to_unit.debug_count++;
				can_state = can.transmit(sizeof(data_to_unit), (uint8_t*)&data_to_unit);
				can_transmit_count++;
				break;
			case 1:
				// to main
				if (ctrl_num == 0) {
					can.setId(CAN_ID_STD, can_id.ctrl0_to_main);
				}else if (ctrl_num == 1){
					can.setId(CAN_ID_STD, can_id.ctrl1_to_main);
				}
				data_to_main.debug_count++;
				can_state = can.transmit(sizeof(data_to_main), (uint8_t*)&data_to_main);
				can_transmit_count++;
				break;
			case 2:
				can_transmit_count = 0; // ラストは0にする
				break;
		}
		can_state_ = can.getState();
#endif

		HAL_GPIO_WritePin(led_pin[2].GPIOx, led_pin[2].GPIO_Pin, GPIO_PIN_RESET);
		updateIndicator(data);
		static uint8_t blink_count = 0;
		blink_count++;
		if(blink_count == 100){
			HAL_GPIO_TogglePin(led_pin[3].GPIOx, led_pin[3].GPIO_Pin);
			blink_count = 0;
		}

	}else if(htim == &htim16){
		HAL_GPIO_WritePin(led_pin[0].GPIOx, led_pin[0].GPIO_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_TogglePin(led_pin[1].GPIOx, led_pin[1].GPIO_Pin);
	}
}

#if XBee
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1){
//		HAL_GPIO_WritePin(led_pin[0].GPIOx, led_pin[0].GPIO_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COUNTER(&htim16, 0);
#if XBee_AT_MODE
		HAL_UART_Receive_IT(&huart1, &returnValue, 1);
#elif XBee_API_MODE
		HAL_UART_Receive_IT(&huart1, (uint8_t*)transmitStatusPacket, sizeof(transmitStatusPacket));
#endif
	}
}
#endif

#if CAN
// CAN受信
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	std::array<uint8_t,8>buf{};
	can_state = can.receive(CAN_RX_FIFO0,(uint8_t*)&buf);
	rx_id = can.getRxId();
	if(can_state == CAN_StatusType::HAL_OK){
		__HAL_TIM_SET_COUNTER(&htim16, 0);
//		disconnect_count = 0;
		switch (can.getRxId()) {
			// from main
			case can_id.main_to_ctrl:
				memcpy(&data_from_main, &buf, sizeof(data_from_main));
				break;

			// from unit
			case can_id.unit0_to_ctrl0:
			case can_id.unit1_to_ctrl1:
				memcpy(&data_from_unit,&buf,sizeof(data_from_unit));
				break;
		}
	}
	// 通信確認インジケータ
	static uint8_t blink_count = 0;
	blink_count++;
	if(blink_count == 100){
		HAL_GPIO_TogglePin(led_pin[1].GPIOx, led_pin[1].GPIO_Pin);
		blink_count = 0;
	}
	HAL_GPIO_WritePin(led_pin[0].GPIOx, led_pin[0].GPIO_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim16, 0);
}
#endif

void readSW(uint8_t &data){
	data = 0;
	for(uint8_t n=0; n<13; n++){
		data |= (uint16_t)HAL_GPIO_ReadPin(gpio_pin[n].GPIOx, gpio_pin[n].GPIO_Pin) << n;
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
//	if( HAL_GPIO_ReadPin(readPin[0].GPIOx, readPin[0].gpio_pin) == GPIO_PIN_RESET ){
//		HAL_GPIO_WritePin(External_LED_2_GPIO_Port, External_LED_2_Pin, GPIO_PIN_SET);
//	}else{
//		HAL_GPIO_WritePin(External_LED_2_GPIO_Port, External_LED_2_Pin, GPIO_PIN_RESET);
//	}
}

/* Function Body End */
