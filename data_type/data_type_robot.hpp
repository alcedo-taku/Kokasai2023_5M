#ifndef DATA_TYPE_ROBOT
#define DATA_TYPE_ROBOT

#include <array>
#include <cstdint>
#include "main.h"
#include <cmath>

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
 * ロボット上の的の位置を格納する構造体
 */
struct TargetPosition {
	float x,y,angle;
};

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
	START_0,	//! スタート5秒前（ローラー未回転）
	READY_0,	//! スタート(ローラー未回転)
	READY, 		//! スタート5秒前
	START,		//! スタート
	END_READY,	//! 停止5秒前
	STOP, 		//! 停止
	DEBUGING,   //! セッティングモード
};
/** enum **/

#endif /* DATA_TYPE_ROBOT */
