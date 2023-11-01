#ifndef DATA_TYPE_CAN
#define DATA_TYPE_CAN

#include <array>
#include <cstdint>
#include "main.h"
#include "data_type_robot.hpp"

//constexpr uint8_t MAIN_ADDRESS = 01;
//constexpr uint8_t ACTR_ADDRESS = 02;
//constexpr uint64_t TARGET_XBee_ADDRESS = 0x0013A2004198443F; // 運転段階用

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
//	uint8_t debug_count = 0;
	GameState game_state = GameState::STOP;
};

/**
 * unitbase → main の通信データ
 */
struct DataFromUnitToMain{
	uint8_t debug_count = 0;
	uint8_t hit_points; 	//!< 右3bitを使って0,1で格納
	uint8_t last_bullet;
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
	uint8_t lock_on; 		//!< 右から 1bit目:locking, 2bit目:locked
	GameState game_state = GameState::STOP;
	uint8_t mato;
};

/**
 * unitbase → unitbase の通信データ
 */
struct DataFromUnitToUnit{
	uint8_t debug_count = 0;
	uint8_t lock_on; 			//!< 1bit目:locked, 2bit目:lock
	SensorData sensor_data;
};

/**
 * can data の id
 */
struct CanId{
	// unit to unit
	static constexpr uint32_t unit0_to_unit1_h = 0x100;
	static constexpr uint32_t unit0_to_unit1_l = 0x102;
	static constexpr uint32_t unit1_to_unit0_h = 0x101;
	static constexpr uint32_t unit1_to_unit0_l = 0x104;
	// main to
	static constexpr uint32_t main_to_unit	= 0x113;
	static constexpr uint32_t main_to_ctrl	= 0x114; // 不要
	// to main
	static constexpr uint32_t ctrl0_to_main  = 0x107; // 不要
	static constexpr uint32_t ctrl1_to_main  = 0x108; // 不要
	static constexpr uint32_t unit0_to_main  = 0x105;
	static constexpr uint32_t unit1_to_main  = 0x106;
	// unit - ctrl
	static constexpr uint32_t ctrl0_to_unit0 = 0x109;
	static constexpr uint32_t ctrl1_to_unit1 = 0x110;
	static constexpr uint32_t unit0_to_ctrl0 = 0x111;
	static constexpr uint32_t unit1_to_ctrl1 = 0x112;
};
//CanId can_id;

/** 通信 **/


#endif /* DATA_TYPE_CAN */
