#ifndef DATA_TYPE
#define DATA_TYPE

#include <array>
#include <cstdint>
//#include "ArmoredTrain.hpp"

//constexpr uint8_t MAIN_ADDRESS = 01;
//constexpr uint8_t ACTR_ADDRESS = 02;
//constexpr uint64_t TARGET_XBee_ADDRESS = 0x0013A2004198443F; // 運転段階用

/**
 * controller → main の通信データ
 */
struct DataFromCtrlToMain{

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

};

/**
 * unitbase → main の通信データ
 */
struct DataFromUnitToMain{
	uint8_t hit_points; //!< 右3bitを使って0,1で格納
};

/**
 * controller →　unitbase の通信データ
 */
struct DataFromCtrlToUnit{
	int16_t left_handle;
	int16_t right_handle;
	bool is_pulled_trigger;
};

/**
 * unitbase → controller の通信データ
 */
struct DataFromUnitToCtrl{
	uint8_t lock_on; //!< 右から 1bit目:locking, 2bit目:locked
};

/**
 * unitbase → unitbase のデータ
 */
struct DataFromUnitToUnit{
	uint8_t debug_count = 0;
	uint8_t lock_on; //!< 1bit目:locked, 2bit目:lock

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
