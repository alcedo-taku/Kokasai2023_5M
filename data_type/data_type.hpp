#ifndef DATA_TYPE
#define DATA_TYPE

#include <array>
#include <cstdint>

constexpr uint64_t TARGET_XBee_ADDRESS = 0x0013A2004198443F; // 運転段階用

/** 便利な構造体 **/
struct GPIO_pin {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
};
/** 便利な構造体 **/


/** 通信 **/

/**
 * controller → main の通信データ
 */
struct DataFromCtrlToMain{
	uint8_t debug_count = 0;
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
	int16_t left_handle;
	int16_t right_handle;
	bool is_pulled_trigger;
};

/**
 * unitbase → controller の通信データ
 */
struct DataFromUnitToCtrl{
	uint8_t debug_count = 0;
	uint8_t lock_on; 		//!< 右から1bit目:locked, 2bit目:lock
};

/**
 * unitbase → unitbase の通信データ
 */
struct DataFromUnitToUnit{
	uint8_t debug_count = 0;
	uint8_t lock_on; 		//!< 右から1bit目:なし, 2bit目:lock
};

/**
 * can data の id
 */
struct CanId{
	// main to
	static constexpr uint32_t main_to_unit	= 0x1;
	static constexpr uint32_t main_to_ctrl	= 0x2; // 不要
	// to main
	static constexpr uint32_t ctrl0_to_main  = 0x3; // 不要
	static constexpr uint32_t ctrl1_to_main  = 0x4; // 不要
	static constexpr uint32_t unit0_to_main  = 0x5;
	static constexpr uint32_t unit1_to_main  = 0x6;
	// unit to unit
	static constexpr uint32_t unit0_to_unit1 = 0x7;
	static constexpr uint32_t unit1_to_unit0 = 0x8;
	// unit - ctrl
	static constexpr uint32_t ctrl0_to_unit0 = 0x9;
	static constexpr uint32_t ctrl1_to_unit1 = 0x10;
	static constexpr uint32_t unit0_to_ctrl0 = 0x11;
	static constexpr uint32_t unit1_to_ctrl1 = 0x12;
}can_id;

/** 通信 **/


#endif /* DATA_TYPE */
