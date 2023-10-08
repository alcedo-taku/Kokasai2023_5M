#ifndef DATA_TYPE
#define DATA_TYPE

#include <array>
#include <cstdint>

constexpr uint8_t MAIN_ADDRESS = 01;
constexpr uint8_t ACTR_ADDRESS = 02;
constexpr uint64_t TARGET_XBee_ADDRESS = 0x0013A2004198443F; // 運転段階用

/**
 * controller → main の通信データ
 */
struct DataFromCtrlToMain{
	std::array<int16_t, 4> joystick_data;
	uint8_t button_state; //!< 非常停止、右発射、左発射、装填、旋回左、旋回右、（駆動高速）
//	uint8_t check_sum;
};

/**
 * main → controller の通信データ
 */
struct DataFromMainToCtrl{

};

/**
 * main → unitbase の通信データ
 */
struct DataFromMainToUnit{
	uint8_t debug_count = 0;
//	uint8_t send_flag;
//	std::array<int16_t, 7> motor_compare;	//!< モータの値（0~3:駆動、4:腰、5:右腕、6:左腕）
//	float servo_angle; //!< 指の角度
//	uint8_t command; //!< 右指ソレノイド、アウトリガー、右腕リセット待機中
};

/**
 * unitbase → main の通信データ
 */
struct DataFromUnitToMain{
	std::array<int32_t, 2> encoder_count;	//!< 腕のエンコーダの値 右、左 [count]
	std::array<int32_t, 2> encoder_speed;	//!< 腕の速さ [count/s]
	uint8_t command;						//!< 右腕リセット、左腕リセット
};

/**
 * controller →　unitbase の通信データ
 */
struct DataFromControllerToUnit{

};

/**
 * unitbase → controller の通信データ
 */
struct DataFromUnitToCtrl{

};

/**
 * unitbase → unitbase のデータ
 */
struct DataFromUnitToUnit{
	uint8_t debug_count = 0;
};

/**
 * can data の id
 */
struct CanId{
	uint32_t main_to_unit	= 0x1;
	uint32_t main_to_ctrl	= 0x2;
	uint32_t ctrl1_to_main  = 0x3;
	uint32_t ctrl2_to_main  = 0x4;
	uint32_t unit1_to_main  = 0x5;
	uint32_t unit2_to_main  = 0x6;
	uint32_t unit0_to_unit1 = 0x7;
	uint32_t unit1_to_unit0 = 0x8;
}can_id;


#endif /* DATA_TYPE */
