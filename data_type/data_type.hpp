#ifndef DATA_TYPE
#define DATA_TYPE

#include <array>
#include <cstdint>
#include "main.h"
#include "ArmoredTrain.hpp"

//constexpr uint8_t MAIN_ADDRESS = 01;
//constexpr uint8_t ACTR_ADDRESS = 02;
//constexpr uint64_t TARGET_XBee_ADDRESS = 0x0013A2004198443F; // 運転段階用

/** 便利な構造体 **/
struct GPIO_pin {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
};
/** 便利な構造体 **/


/** センサーやコントローラ **/

/**
 * センサーの生値
 */
struct SensorData {
	uint16_t enc_roller_rotation;
	int16_t enc_position;
	uint32_t pot_angle_of_turret;
};

/**
 * コントローラの値
 */
struct ControllerData{
	int16_t left_handle;
	int16_t right_handle;
	bool is_pulled_trigger;
};
/** センサーやコントローラ **/


/** 通信 **/

/**
 * controller → main の通信データ
 */
struct DataFromCtrlToMain{

};

/**
 * main → controller の通信データ
 */
struct DataFromMainToCtrl{
	uint8_t debug_count = 0;
};

/**
 * main → unitbase の通信データ
 */
struct DataFromMainToUnit{
	uint8_t debug_count = 0;
	uint8_t is_moving_time = 0; //!< 右から1bit目:可否, 2bit目:ready
};

/**
 * unitbase → main の通信データ
 */
struct DataFromUnitToMain{
	uint8_t debug_count = 0;
	uint8_t hit_points; 	//!< 右3bitを使って0,1で格納
};

/**
 * controller →　unitbase の通信データ
 */
struct DataFromCtrlToUnit{
	uint8_t debug_count = 0;
	ControllerData ctrl_data;
};

/**
 * unitbase → controller の通信データ
 */
struct DataFromUnitToCtrl{
	uint8_t lock_on; //!< 右から 1bit目:locking, 2bit目:locked
};

/**
 * unitbase → unitbase の通信データ
 */
struct DataFromUnitToUnit{
	uint8_t debug_count = 0;
	uint8_t lock_on; //!< 1bit目:locked, 2bit目:lock
	SensorData sensor_data;
};

/**
 * can data の id
 */
struct CanId{
	// unit to unit
	static constexpr uint32_t unit0_to_unit1 = 0x100;
	static constexpr uint32_t unit1_to_unit0 = 0x101;
	// main to
	static constexpr uint32_t main_to_unit	= 0x113;
	static constexpr uint32_t main_to_ctrl	= 0x114; // 不要
	// to main
	static constexpr uint32_t ctrl0_to_main  = 0x103; // 不要
	static constexpr uint32_t ctrl1_to_main  = 0x104; // 不要
	static constexpr uint32_t unit0_to_main  = 0x105;
	static constexpr uint32_t unit1_to_main  = 0x106;
	// unit - ctrl
	static constexpr uint32_t ctrl0_to_unit0 = 0x109;
	static constexpr uint32_t ctrl1_to_unit1 = 0x110;
	static constexpr uint32_t unit0_to_ctrl0 = 0x111;
	static constexpr uint32_t unit1_to_ctrl1 = 0x112;
}can_id;

/** 通信 **/


#endif /* DATA_TYPE */
