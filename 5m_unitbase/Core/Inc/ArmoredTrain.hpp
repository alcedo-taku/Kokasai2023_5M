/*
 * ArmoredTrain.hpp
 *
 *  Created on: Oct 15, 2023
 *      Author: takuj
 */

#ifndef ARMOREDTRAIN_HPP_
#define ARMOREDTRAIN_HPP_

#include "Automatic_Control_Assistant.hpp"
#include "data_type.hpp"

namespace at {

/**
 * このクラスに入ってくる構造体
 */
struct InputData {
	SensorData myself;
	SensorData enemy;
	bool is_pusshed_lounch_reset;
	ControllerData ctrl;
	GameState game_state;
};

/**
 * このクラスから出ていく構造体
 */
struct OutputData {
	std::array<int16_t, 4> compare; //
	uint8_t lock_on; // ロックオンできたか
};

/**
 * フィールドの寸法等を格納する構造体
 */
struct FieldData{
	static constexpr float opposing_distance = 2;	//!< 対向距離()[m]
	static constexpr float rail_length = 1.4;			//!< 横移動可能長さ[m]
	static constexpr float gravity = 9.8;			//!< 重力加速度[m/s]
};

/**
 * ロボット上の的の位置を格納する構造体
 */
struct TargetPositionA {
	float x,y,angle;
};

/**
 * ロボットの静的な情報を格納する構造体
 */
struct RobotStaticData {
	static constexpr float angle_of_depression = 10 * M_PI/180;	//!< 砲塔俯角[rad]
	static constexpr float turret_length = 0.050;			//!< 旋回中心から速度計側部までの距離[m] (l0)
	static constexpr float radius_of_roller = 0.0245;		//!< ローラー半径[m]
	static constexpr std::array<TargetPositionA, 3> mato = {{	//!< ロボット内座標
			{0.010, 0.010, 12},
			{0.010, 0.010, 12},
			{0.010, 0.010, 12},
	}};
	static constexpr float time_lug1 = 0.5;				//!< 射出までにかかる時間[s]
	static constexpr float turret_angle_max = 40 * M_PI/180; //!< 最大砲塔旋回角度
	static constexpr float enc_to_pos_ratio = 1.0f / (5120.0f/*PPR*/*4.0f) * 15/*ギヤ数*/ * 6.28f/1000.0f;
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

class ArmoredTrain {
private:
	// 変数
	RobotMovementDataSet now;
	RobotMovementDataSet future;
	RobotMovementData future0_myself;
	RobotMovementData target;
	std::array<TargetPositionR, 3> mato;	//!< 的の位置
	ShotData shot_data;
	SensorData prev_myself;
	SensorData prev_enemy;
	std::array<int16_t, 4> prev_compare = {0,0,0,0};
	GameState prev_game_state = GameState::STOP;
	// とりあえずここに置く変数
	float pos_e;
	float ang_e;
	float evaluation_value = 100000;
	// pidクラス
	aca::PID_Element pid_parameter_position = {5,0,0};
	aca::PID_controller pid_position = aca::PID_controller (pid_parameter_position, frequency);
	aca::PID_Element pid_parameter_angle {5,0,0};
	aca::PID_controller pid_angle = aca::PID_controller (pid_parameter_angle, frequency);
	aca::PID_Element pid_parameter_roller {2,0,0};
	aca::PID_controller pid_roller = aca::PID_controller (pid_parameter_roller, frequency);
	// 関数
	template <typename T> T map(T x, T in_min, T in_max, T out_min, T out_max);
	template <typename T> T suppress_value(T value, T max_abs_value);
	void convert_to_SI(SensorData& prev_sensor_data, SensorData& sensor_data, RobotMovementData& movement_data);
	void calc_initial_velocity(RobotMovementData& movement_data, BulletVelocity& bullet_velocity);	//!< 砲弾の初速度を求める 運動学　いらない
	void calc_roller_rotation();
	void calc_pos_fut(RobotMovementData& movement_data_now, RobotMovementData movement_data_fut, uint16_t time_lug);
	void calc_shot(RobotMovementDataSet& movement_data, ShotData& shot_data);
	void calc_pos_of_mato(RobotMovementDataSet& movement_data, std::array<TargetPositionR, 3>& mato);		//!< 本体の位置から的の位置を計算する
	uint8_t judge_mato(std::array<TargetPositionR, 3>& mato, float& angle_of_shot);
	void calc_output(RobotMovementData& now, RobotMovementData& target, uint8_t& mato_num, InputData& input_data, OutputData& output_data);
public:
	ArmoredTrain();
	void update(InputData& input_data, OutputData& output_data);
};

} /* namespace at */

#endif /* ARMOREDTRAIN_HPP_ */
