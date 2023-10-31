#ifndef DATA_TYPE_ROBOT
#define DATA_TYPE_ROBOT

#include <array>
#include <cstdint>
#include "main.h"
#include <cmath>

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
	int32_t enc_roller_rotation;	//!< count(微分していない値)
	int32_t enc_position;			//!< count(微分していない値)
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

/**
 * ロボットの動的な情報を格納する構造体
 */
struct RobotMovementData {
	float position;							//!< 位置[m]
	float velocity;							//!< 速度[m/s]
	float angle_of_turret;					//!< 砲塔角度[rad]
	float angular_velocity_of_truret;		//!< 砲塔角速度[rad/s]
//	float angle_of_depression;				//!< 砲塔俯角[rad]
	float roller_rotation;					//!< ローラー回転数[rad/s]
};

/**
 * フィールドの寸法等を格納する構造体
 */
struct FieldData{
	static constexpr float opposing_distance = 3;		//!< 対向距離()[m]
	static constexpr float rail_length = 1.4 - 0.012*2;	//!< 横移動可能長さ[m]
	static constexpr float gravity = 9.8;				//!< 重力加速度[m/s]
};

/**
 * ロボット上の的の位置を格納する構造体
 */
struct TargetPosition {
	float x,y,angle;
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
	static constexpr float turret_angle_max = 40 * M_PI/180; //!< 最大砲塔旋回角度
	static constexpr float enc_to_pos_ratio = 1.0f / (5120.0f/*PPR*/*4.0f) * 15/*ギヤ数*/ * 6.28f/1000.0f;
	static constexpr uint8_t max_bullet = 30;
};
// todo パラメータ入力しろ
//RobotStaticData static_data;

/**
 * 自分からの相対座標系での敵の的の角度
 */
struct TargetPositionR {
	float l,angle_pos,angle_set;
};


/**
 * 自分と敵の情報のセットを格納する構造体
 */
struct RobotMovementDataSet {
	RobotMovementData myself;   //!< 自分の動き
	RobotMovementData enemy;    //!< 敵の動き
};

struct BulletVelocity {
	float x,y,z;
};

constexpr uint16_t frequency = 1000;

/**
 * 発射結果等を格納する構造体
 */
struct ShotData {
	float l;
	float v0;
	float time;
};

enum class ShotState : uint8_t{
	SHOTING_0, 		//! 射出中（リミットスイッチ押し）
	SHOTING_1,		//! 射出中（リミットスイッチ開放）
	COOLING_TIME,	//! 冷却時間（あんまり連射すると、すぐ打ち切ってしまうから）
	STOP, 			//! 停止
};

/** センサーやコントローラ **/

/** enum **/
enum class GameState : uint8_t{
	READY, 		//! スタート5秒前
	START,		//! スタート
	END_READY,	//! 停止5秒前
	STOP, 		//! 停止
};
/** enum **/

#endif /* DATA_TYPE_ROBOT */
