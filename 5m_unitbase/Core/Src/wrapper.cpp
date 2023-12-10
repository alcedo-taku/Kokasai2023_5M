/**
 * 回路の印字は相変わらず間違っている
 *
 * GPIO
 * ピン配置：文字を読む方から見て上から順に0-5と番号を振った
 * 0:そのunitbaseの番号を決める、ジャンパーあり→0, なし→1
 *
 * LED
 * PD1			:タイマー割り込み
 * PC15(GPIO6)	:受信成功時点滅
 *
 * Encoder
 * 0(tim2) :射出
 * 1(tim3) :位置
 */
#include "wrapper.hpp"

/* Include Begin */
#include "main.h"
#include "gpio.h"
#include "HAL_Extension.hpp"
#include "mcp3208.hpp"
#include <array>
#include <string.h>
#include "ArmoredTrain.hpp"
/* Include End */

/* Define Begin */
#define IS_MOTOR_TEST 0
/* Define End */

/* Enum Begin */
/* Enum End */

/* Struct Begin */
struct DebugValue {
	// [byte]
	uint8_t size_of_u2m;
	uint8_t size_of_u2u;
	uint8_t size_of_u2c;
};
/* Struct End */

/* Variable Begin */
// 制御
uint8_t unit_num = 0;
at::ArmoredTrain armored_train;
at::InputData input_data;
at::OutputData output_data;
SensorData sensor_data;

// GPIO
constexpr std::array<GPIO_pin,6> gpio_pin = {{
	// ピン配置：文字を読む方から見て上から順に0-5と番号を振った
	{GPIOF, GPIO_PIN_4}, 	// そのunitbaseの番号を決める、ジャンパーあり→0, なし→1
	{GPIOF, GPIO_PIN_1}, 	// 射出検知
	{GPIOF, GPIO_PIN_0}, 	// 位置のリセット
	{GPIOC, GPIO_PIN_13},	// 的0
	{GPIOC, GPIO_PIN_14},	// 的1
	{GPIOC, GPIO_PIN_15}, 	// 的2
}};
std::array<halex::GPIO,6> hal_gpio = {{
	// ピン配置：文字を読む方から見て上から順に0-5と番号を振った
	halex::GPIO(GPIOF, GPIO_PIN_4 ), 	// 0 そのunitbaseの番号を決める、ジャンパーあり→0, なし→1
	halex::GPIO(GPIOF, GPIO_PIN_1 ), 	// 1 射出検知
	halex::GPIO(GPIOF, GPIO_PIN_0 ), 	// 2 位置のリセット
	halex::GPIO(GPIOC, GPIO_PIN_15),	// 3 的0
	halex::GPIO(GPIOC, GPIO_PIN_14),	// 4 的1
	halex::GPIO(GPIOC, GPIO_PIN_13), 	// 5 的2
}};

//LED
std::array<halex::GPIO,6> hal_led = {{
	// ピン配置：文字を読む方から見て上から順に0-5と番号を振った
	halex::GPIO(GPIOC, GPIO_PIN_0 ), 	// 0
	halex::GPIO(GPIOC, GPIO_PIN_1 ), 	// 1
	halex::GPIO(GPIOC, GPIO_PIN_2 ), 	// 2
	halex::GPIO(GPIOC, GPIO_PIN_3),		// 3 的0
	halex::GPIO(GPIOC, GPIO_PIN_8),		// 4 的1
	halex::GPIO(GPIOC, GPIO_PIN_9), 	// 5 的2
}};

// モータ
std::array<halex::Motor, 4> motor = {
		halex::Motor(&htim4,  TIM_CHANNEL_1, &htim4,  TIM_CHANNEL_2), // 0 1 射出
		halex::Motor(&htim1,  TIM_CHANNEL_1, &htim1,  TIM_CHANNEL_2), // 1 2 送り
		halex::Motor(&htim8,  TIM_CHANNEL_1, &htim8,  TIM_CHANNEL_2), // 2 3 横移動
		halex::Motor(&htim15, TIM_CHANNEL_2, &htim15, TIM_CHANNEL_1), // 3 4 旋回
};

// エンコーダ
std::array<halex::Encoder, 1> encoder = {
//		halex::Encoder(&htim2),	// ローラー
		halex::Encoder(&htim3)	// 位置
};
std::array<int32_t, 2> encoder_count = {0,0};
int32_t encoder_count_debug[2];

// ADC
mcp3208::MCP3208 mcp3208_reader(hspi2,SPI2_NSS_GPIO_Port,SPI2_NSS_Pin);
std::array<uint16_t, 4> adc_value_array;


// CAN
//simpleCanUser can(&hcan);
halex::CAN_Communication can(&hcan);
/*can関連*/
uint32_t mailbox0_complete_count = 0;
uint32_t mailbox1_complete_count = 0;
uint32_t mailbox2_complete_count = 0;
uint8_t can_transmit_count = 1;
uint32_t rx_id; // debug用
HAL_StatusTypeDef can_status;
HAL_CAN_StateTypeDef can_state;
uint16_t rx0_callback_count = 0;
uint16_t transmit_frequency = 300; //データの更新周波数
uint8_t number_of_id = 8;
DataFromUnitToUnit data_from_unit;
DataFromUnitToUnit data_to_unit;
DataFromMainToUnit data_from_main;
DataFromUnitToMain data_to_main;
DataFromCtrlToUnit data_from_ctrl;
DataFromUnitToCtrl data_to_ctrl;
uint8_t debug_count = 0;

DebugValue debug_value;

/* Variable End */

/* Class Constructor Begin */
/* Class Constructor End */

/* Function Prototype Begin */
/* Function Prototype End */

void init(void){
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	for (uint8_t i = 0; i < 6; i++) {
		hal_led[i].set();
	}
	HAL_Delay(1000);
	for (uint8_t i = 0; i < 6; i++) {
		hal_led[i].reset();
	}
	HAL_Delay(1000);
	for (uint8_t i = 3; i < 6; i++) {
		hal_led[i].set();
	}

	// unitbase unmber を設定
	unit_num = (uint8_t)HAL_GPIO_ReadPin(gpio_pin[0].GPIOx, gpio_pin[0].GPIO_Pin);

    //MD
    for(uint8_t i=0; i < motor.size(); i++){
    	motor[i].start();
    }

	// ADC
	mcp3208_reader.init();

	// エンコーダ
	encoder[0].start();
//	encoder[1].start();

	// CAN
	// CANの初期設定
	can.init();
	// 受信設定
	can.setSlaveStartFilterBank(14); // どこまでのバンクを使うか
	can.setFilterFIFOAssignment(CAN_RX_FIFO0); // 使うFIFOメモリ
	if (unit_num == 0) {
		can.setIdFilter(CanId::main_to_unit, CanId::unit1_to_unit0_h, CanId::unit1_to_unit0_l, CanId::ctrl0_to_unit0);
	}else if (unit_num == 1){
		can.setIdFilter(CanId::main_to_unit, CanId::unit0_to_unit1_h, CanId::unit0_to_unit1_l, CanId::ctrl1_to_unit1);
	}
	can.applyFilterConfig(); // フィルターの設定を反映する
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 受信割り込みの有効化
	// 送信設定
	can.setRemoteTransmissionRequest(halex::CAN_Communication::RemoteTransmissionRequest::Data); // メッセージのフレームタイプをデータフレームに設定する
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);     // 送信割り込みの有効化
	HAL_CAN_TxMailbox0CompleteCallback(&hcan);
	// can 開始
	can.start();



	debug_count = 0;
	debug_value.size_of_u2c = sizeof(data_to_ctrl);
	debug_value.size_of_u2m = sizeof(data_to_main);
	debug_value.size_of_u2u = sizeof(data_to_unit);

	// タイマー割込み
	HAL_TIM_Base_Start_IT(&htim16); // メイン処理用
	HAL_TIM_Base_Start_IT(&htim17); // 通信用
}

void loop(void){
//	uint16_t adc_value;
//	mcp3208_reader.update(mcp3208::Channel::CH_0, 0xF);
//	adc_value_array[0] = mcp3208_reader.get(mcp3208::Channel::CH_0);
}

void exit_gpio(void) {
	static uint32_t g_time = 0;
	if (g_time + 1 <= HAL_GetTick()) {
		encoder_count[0]++;
		g_time = HAL_GetTick();
	}
}

uint16_t experiment_timer;
uint8_t debugmato[3];

/* Function Body Begin */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//割り込みの処理内容
	if(htim == &htim16){
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
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
				break;
			case 4000:
				for(uint8_t i=0; i < motor.size(); i++){
					motor[i].setSpeed(0);
				}
				experiment_timer = 0;
				HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
				break;
			default:
				break;
		}
#endif
		experiment_timer++;
		if (experiment_timer > 4000) {
			experiment_timer = 0;
		}

		/** 受信情報を整理 **/
		/* from main インジケータ */
		data_to_ctrl.game_state = data_from_main.game_state;
		input_data.game_state = data_from_main.game_state;
		/* from controller */
		input_data.ctrl = data_from_ctrl.ctrl_data;
		/* from unit */
		input_data.enemy = data_from_unit.sensor_data;

		/** 入力 **/

		/* センサーの値の代入 */
		// adc
		mcp3208_reader.update(0xF);
//		adc_value_array = mcp3208_reader.get();
		for(uint8_t i = 0; i < 4; i++){
			adc_value_array[i] = mcp3208_reader.get( (mcp3208::Channel)i );
		}
		// encoder
//		for (uint8_t i = 0; i < 1; i++) {
//			encoder[i].update();
//			encoder_count[i] = encoder[i].getCount();
//		}
		encoder[0].update();
		encoder_count[1] = encoder[0].getCount();
		encoder_count_debug[1] = encoder[0].getCount();
		if (hal_gpio[2].isSet()) {
//			encoder[0].resetCount();
			encoder[0].setCount(FieldData::rail_length/RobotStaticData::enc_to_pos_ratio);
			armored_train.reset_position();
		}
		// リミットスイッチ
		input_data.hit_points_gpio = 0;
		input_data.hit_points_gpio = input_data.hit_points_gpio | (uint8_t)!hal_gpio[3].isSet() << 2;
		input_data.hit_points_gpio = input_data.hit_points_gpio | (uint8_t)!hal_gpio[4].isSet() << 1;
		input_data.hit_points_gpio = input_data.hit_points_gpio | (uint8_t)!hal_gpio[5].isSet() << 0;

		// 代入
		input_data.myself.enc_roller_rotation = encoder_count[0];
		input_data.myself.enc_position = encoder_count[1];
		input_data.myself.pot_angle_of_turret = adc_value_array[0];
		input_data.is_pusshed_lounch_reset = !(bool)HAL_GPIO_ReadPin(gpio_pin[1].GPIOx, gpio_pin[1].GPIO_Pin);

		/* 敵ロボットからの情報の代入 */
		input_data.enemy = data_from_unit.sensor_data;

		/** 演算 **/
		/* update & 取得 */
		armored_train.update(input_data, output_data);

		/** 出力 **/
		/* モータに送る */
		for(uint8_t i=0; i < motor.size(); i++){
			motor[i].setSpeed(output_data.compare[i]);
		}
		/* LED */
		hal_led[3].setIf(!(bool)((output_data.hit_points & (1<<2))>>2));
		hal_led[4].setIf(!(bool)((output_data.hit_points & (1<<1))>>1));
		hal_led[5].setIf(!(bool)((output_data.hit_points & (1<<0))>>0));

		/** 送信情報を整理 **/
		/* to controller */
		data_to_ctrl.lock_on = output_data.lock_on | data_from_unit.lock_on <<1;
		data_to_ctrl.mato = output_data.hit_points;
		/* to unit */
		data_to_unit.sensor_data = input_data.myself;
		data_to_unit.lock_on = output_data.lock_on;
		/* to main */
		data_to_main.hit_points = output_data.hit_points;
		data_to_main.last_bullet = output_data.last_bullet;

	}else if(htim == &htim17){
		/* CAN 送信 */
		static uint8_t can_transmit_count = 0;
		std::array<uint8_t,8>buf{};
		switch(can_transmit_count){
			case 0:
				// to main
				if (unit_num == 0) {
					can.setId(halex::CAN_Communication::IdentifierType::Standard, CanId::unit0_to_main);
				}else if (unit_num == 1){
					can.setId(halex::CAN_Communication::IdentifierType::Standard, CanId::unit1_to_main);
				}
				data_to_main.debug_count++;
				can_status = can.transmit(sizeof(data_to_main), (uint8_t*)&data_to_main);
				can_transmit_count++;
				break;
			case 1:
				// to unit h
				if (unit_num == 0) {
					can.setId(halex::CAN_Communication::IdentifierType::Standard, CanId::unit0_to_unit1_h);
				}else if (unit_num == 1){
					can.setId(halex::CAN_Communication::IdentifierType::Standard, CanId::unit1_to_unit0_h);
				}
				data_to_unit.debug_count = experiment_timer/1000;

				memcpy(&buf,&data_to_unit,sizeof(buf));
				can_status = can.transmit(sizeof(buf), (uint8_t*)&buf);
				can_transmit_count++;
				break;
			case 2:
				// to unit l
				if (unit_num == 0) {
					can.setId(halex::CAN_Communication::IdentifierType::Standard, CanId::unit0_to_unit1_l);
				}else if (unit_num == 1){
					can.setId(halex::CAN_Communication::IdentifierType::Standard, CanId::unit1_to_unit0_l);
				}

				memcpy(&buf,&data_to_unit.sensor_data.enc_position,sizeof(buf));
				can_status = can.transmit(sizeof(buf), (uint8_t*)&buf);
				can_transmit_count++;
				break;
			case 3:
				// to controller
				if (unit_num == 0) {
					can.setId(halex::CAN_Communication::IdentifierType::Standard, CanId::unit0_to_ctrl0);
				}else if (unit_num == 1){
					can.setId(halex::CAN_Communication::IdentifierType::Standard, CanId::unit1_to_ctrl1);
				}
				can_status = can.transmit(sizeof(data_to_ctrl), (uint8_t*)&data_to_ctrl);
				can_transmit_count = 0; // ラストは0にする
				break;
		}
		can_state = can.getState();
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

// CAN受信
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	std::array<uint8_t,8>buf{};
	can_status = can.receive(CAN_RX_FIFO0, (uint8_t*)&buf);
	rx_id = can.getRxId();
	if(can_status == HAL_StatusTypeDef::HAL_OK){
//		__HAL_TIM_SET_COUNTER(&htim13, 0);
//		disconnect_count = 0;
		switch (can.getRxId()) {
			// from main
			case CanId::main_to_unit:
				memcpy(&data_from_main, &buf, sizeof(data_from_main));
				break;

			// from unit h
			case CanId::unit1_to_unit0_h:
			case CanId::unit0_to_unit1_h:
				memcpy(&data_from_unit,&buf,sizeof(buf));
				break;

			// from unit l
			case CanId::unit1_to_unit0_l:
			case CanId::unit0_to_unit1_l:
				memcpy((uint8_t*)(&data_from_unit.sensor_data.enc_position),&buf,sizeof(buf));
				break;

			// from controller
			case CanId::ctrl0_to_unit0:
			case CanId::ctrl1_to_unit1:
				memcpy(&data_from_ctrl,&buf,sizeof(data_from_ctrl));
				break;
			default:
				break;
		}
	}
	// 通信確認インジケータ
	static uint8_t blink_count = 0;
	blink_count++;
	if(blink_count == 100){
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		blink_count = 0;
	}
}
/* Function Body End */
