/*
 * ArmoredTrain.hpp
 *
 *  Created on: Oct 15, 2023
 *      Author: takuj
 */

#ifndef ARMOREDTRAIN_HPP_
#define ARMOREDTRAIN_HPP_

#include <data_type_can.hpp>
#include <data_type_robot.hpp>
#include "Automatic_Control_Assistant.hpp"
#include "ConvertToSI.hpp"
#include <valarray>

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
	uint8_t hit_points_gpio = 0; 	//!< 右3bitを使って0,1で格納（0がデフォ、1が打たれたとき）
};

/**
 * このクラスから出ていく構造体
 */
struct OutputData {
	std::array<int16_t, 4> compare; //!<
	uint8_t lock_on; 				//!< ロックオンできたか
	uint8_t hit_points; 			//!< 右3bitを使って0,1で格納
};


class ArmoredTrain {
private:
	// 変数
	bool is_position_reseted = false;
	RobotMovementDataSet now;
	RobotMovementDataSet future;
	RobotMovementData future0_myself;
	RobotMovementData target;
	std::array<TargetPositionR, 3> mato;	//!< 的の位置
	ShotData shot_data;
	std::array<int16_t, 4> prev_compare = {0,0,0,0};
	GameState prev_game_state = GameState::STOP;
	uint8_t mato_num;
	uint8_t last_bullet = RobotStaticData::max_bullet;
	float manual_angle;
	float prev_manual_angle;
	float target_angle;
	// とりあえずここに置く変数
	float pos_e;
	float ang_e;
	float evaluation_value = 100000;
	// convertクラス
	ConvertToSI cnv_myself;
	ConvertToSI cnv_enemy;
	// pidクラス
	aca::PID_Element pid_parameter_position = {5,0,0};
	aca::PID_controller pid_position = aca::PID_controller (pid_parameter_position, frequency);
	aca::PID_Element pid_parameter_angle {1000,0,0};
	aca::PID_controller pid_angle = aca::PID_controller (pid_parameter_angle, frequency);
	aca::PID_Element pid_parameter_roller {2,0,0};
	aca::PID_controller pid_roller = aca::PID_controller (pid_parameter_roller, frequency);
	// 関数
	template <typename T> T map(T x, T in_min, T in_max, T out_min, T out_max);
	template <typename T> T suppress_value(T value, T max_abs_value);
	void convert_to_SI(SensorData& prev_sensor_data, SensorData& sensor_data, RobotMovementData& movement_data);
	void calc_initial_velocity(RobotMovementData& movement_data, BulletVelocity& bullet_velocity);	//!< 砲弾の初速度を求める 運動学　いらない
	void calc_roller_rotation();
	void calc_pos_fut(RobotMovementData& movement_data_now, RobotMovementData& movement_data_fut, uint16_t time_lug);
	void calc_shot(RobotMovementDataSet& movement_data, ShotData& shot_data);
	void calc_pos_of_mato(RobotMovementDataSet& movement_data, std::array<TargetPositionR, 3>& mato);		//!< 本体の位置から的の位置を計算する
	uint8_t judge_mato(std::array<TargetPositionR, 3>& mato, float& angle_of_shot);
	void update_mato(uint8_t& hit_points_gpio, uint8_t& hit_points);
	void calc_output(RobotMovementData& now, RobotMovementData& target, uint8_t& mato_num, InputData& input_data, OutputData& output_data);
public:
	ArmoredTrain();
	void update(InputData& input_data, OutputData& output_data);
	void reset_position();
};

} /* namespace at */

#endif /* ARMOREDTRAIN_HPP_ */
