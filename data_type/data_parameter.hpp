#ifndef DATA_PARAMETER_ROBOT
#define DATA_PARAMETER_ROBOT

#include <array>
#include <cstdint>
#include "main.h"
#include <cmath>
#include "Automatic_Control_Assistant.hpp"

//constexpr uint8_t MAIN_ADDRESS = 01;
//constexpr uint8_t ACTR_ADDRESS = 02;
//constexpr uint64_t TARGET_XBee_ADDRESS = 0x0013A2004198443F; // 運転段階用

/**
 * フィールドの寸法等を格納する構造体
 */
struct FieldData{
	static constexpr float opposing_distance = 3;		//!< 対向距離()[m]
	static constexpr float rail_length = 1.4 - 0.012*2;	//!< 横移動可能長さ[m]
	static constexpr float gravity = 9.8;				//!< 重力加速度[m/s]
};

/**
 * ロボットの静的な情報を格納する構造体
 */
struct RobotStaticData {
	static constexpr float angle_of_depression = 2 * M_PI/180;	//!< 砲塔俯角[rad]
	static constexpr float turret_length = 0.050;				//!< 旋回中心から速度計側部までの距離[m] (l0)
	static constexpr float radius_of_roller = 0.0245;			//!< ローラー半径[m]
	static constexpr std::array<TargetPosition, 3> mato = {{	//!< ロボット内座標
			{-0.010, 0.0, 0},
			{0.0, 0.010, 0},
			{0.010, 0.0, 0},
	}};
	static constexpr float time_lug1 = 0.5;				//!< 射出までにかかる時間[s]
	static constexpr float turret_angle_max = 0.5;//40 * M_PI/180; //!< 最大砲塔旋回角度
	static constexpr float enc_to_pos_ratio = 1.0f / (5120.0f/*PPR*/*4.0f) * 15/*ギヤ数*/ * 6.28f/1000.0f;
	static constexpr uint8_t max_bullet = 31;
};
// todo パラメータ入力しろ
//RobotStaticData static_data;

#define ID 0
#if ID == 0
// 0
struct RobotUniquData {
	static constexpr aca::PID_Element pid_parameter_position = {5,0,0};
	static constexpr aca::PID_Element pid_parameter_angle {4500,0,0};
	static constexpr float bane_rate = 0.20;
	static constexpr aca::PID_Element pid_parameter_roller {2,0,0};
//	static constexpr std::array<int16_t,4> adcCenter = {1820, 2040, 2048,2048};
};
#elif ID == 1
// 1
struct RobotUniquData {
	static constexpr aca::PID_Element pid_parameter_position = {5,0,0};
	static constexpr aca::PID_Element pid_parameter_angle {7500,0,0};
	static constexpr float bane_rate = 0.15;
	static constexpr aca::PID_Element pid_parameter_roller {2,0,0};
//	static constexpr std::array<int16_t,4> adcCenter = {1900, 2023, 2048,2048};
};
#elif ID == 2
// 2
struct RobotUniquData {
	static constexpr aca::PID_Element pid_parameter_position = {5,0,0};
	static constexpr aca::PID_Element pid_parameter_angle {8000,0,0};
	static constexpr float bane_rate = 0.15;
	static constexpr aca::PID_Element pid_parameter_roller {2,0,0};
//	static constexpr std::array<int16_t,4> adcCenter = {1950, 2048, 2048,2048};
};
#elif ID == 3
// 3
//	static constexpr std::array<int16_t,4> adcCenter = {1820, 2040, 2048,2048};

#endif

constexpr uint16_t frequency = 500;

#endif /* DATA_PARAMETER_ROBOT */
