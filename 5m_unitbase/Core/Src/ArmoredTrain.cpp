/*
 * ArmoredTrain.cpp
 *
 *  Created on: Oct 15, 2023
 *      Author: takuj
 */

#include "ArmoredTrain.hpp"

namespace at {

/**
 * コンストラクタ
 */
ArmoredTrain::ArmoredTrain() {
//	pid_position = aca::PID_controller (pid_parameter_position, frequency);
}

/**
 * センサーの入力を、SI単位系に変換する関数
 * @param sensor_data
 * @param movement_data
 */
void ArmoredTrain:: convert_to_SI(RobotSensorData& sensor_data, RobotMovementData& movement_data) {
	static RobotSensorData prev_sensor_data;
	movement_data.angle_of_turret			 = sensor_data.pot_angle_of_turret * 0.01;
	movement_data.position					 = sensor_data.enc_position * 0.1;
	movement_data.angular_velocity_of_truret = (sensor_data.pot_angle_of_turret	 - prev_sensor_data.pot_angle_of_turret	) * 0.01 * frequency;
	movement_data.velocity					 = (sensor_data.enc_position		 - prev_sensor_data.enc_position		) * 0.1 * frequency;
	movement_data.roller_rotation			 = (sensor_data.enc_roller_rotation	 - prev_sensor_data.enc_roller_rotation	) * 0.001 * frequency;
}

/**
 * 砲弾の絶対座標系での初速度を計算する関数
 */
void ArmoredTrain::calc_initial_velocity() {
	float abs_bullet_velocity = robot_static_data.radius_of_roller * now.myself.roller_rotation;
	constexpr float cos_depression = std::cos(robot_static_data.angle_of_depression);
	constexpr float sin_depression = std::sin(robot_static_data.angle_of_depression);
	float cos_turret = std::cos(now.myself.angle_of_turret + M_PI_2);
	float sin_turret = std::sin(now.myself.angle_of_turret + M_PI_2);
	bullet_velocity.x = abs_bullet_velocity * cos_depression * cos_turret
						+ now.myself.velocity
						- robot_static_data.turret_length * now.myself.roller_rotation * sin_turret;
	bullet_velocity.y = abs_bullet_velocity * cos_depression * sin_turret
						+ robot_static_data.turret_length * now.myself.roller_rotation * cos_turret;
	bullet_velocity.z = abs_bullet_velocity * sin_depression;
}

/**
 * ローラーの速度と、角度を求める
 */
void ArmoredTrain::calc_roller_rotation() {

}

/**
 * 未来の位置を予測する
 * @param movement_data_now
 * @param movement_data_fut
 * @param time_lug
 */
void ArmoredTrain::calc_pos_fut(RobotMovementData& movement_data_now, RobotMovementData movement_data_fut, uint16_t time_lug){
	movement_data_fut.velocity 						= movement_data_now.velocity;
	movement_data_fut.position 						= movement_data_now.position 		+ movement_data_fut.velocity * time_lug;
	movement_data_fut.angular_velocity_of_truret 	= movement_data_now.angular_velocity_of_truret;
	movement_data_fut.angle_of_turret 				= movement_data_now.angle_of_turret + movement_data_fut.angular_velocity_of_truret * time_lug;
}

void ArmoredTrain::calc_shot(RobotMovementDataSet& movement_data, ShotData& shot_data){
	// 2台間の距離を計算
	float x = movement_data.myself.position + movement_data.enemy.position - field_data.rail_length;
	float y = field_data.opposing_distance;
	shot_data.l = std::sqrt(x*x + y*y);
	// どの速度なら届くかを計算
	constexpr float cos_depression = std::cos(robot_static_data.angle_of_depression);
	constexpr float sin_depression = std::sin(robot_static_data.angle_of_depression);
	shot_data.v0 = std::sqrt( (shot_data.l*field_data.gravity) / (2*sin_depression*cos_depression) );
	// その速度でどのくらいの時間がかかるか計算
	shot_data.time = 2*shot_data.v0*sin_depression/field_data.gravity;
}

/**
 * 本体位置から的位置を計算する
 */
void ArmoredTrain::calc_pos_of_target(RobotMovementData& mydata, RobotMovementData& enemydata){
	for (uint8_t i = 0; i < 3; i++) {
		// 砲塔についていないターゲット
		float x = mydata.position + enemydata.position - field_data.rail_length + robot_static_data.target[i].x;
		float y = field_data.opposing_distance - robot_static_data.target[i].y;
		target[i].l = std::sqrt(x*x + y*y);
		target[i].angle = std::atan2(x, y);
	}
}

/**
 * どの的を狙うか、昇順補助するか、ロックオンできているか
 */
uint8_t ArmoredTrain:: judge_target() {
//	todo ここやる！
	uint8_t target;

	return target;
}

/**
 * 出力構造体の計算
 */
void ArmoredTrain::calc_output(){

}

/**
 * update関数
 * @param input_data
 * @param output_data
 */
void ArmoredTrain::update(InputData& input_data, OutputData& output_data) {
	this->input_data = input_data;
	// SI単位系に変換
	convert_to_SI(input_data.myself, now.myself);
	convert_to_SI(input_data.enemy, now.enemy);
//	ここから
	// 射出時にいる位置を計算
	calc_pos_fut(now.myself, future.myself, robot_static_data.time_lug1);
	calc_pos_fut(now.enemy, future.enemy, robot_static_data.time_lug1);
	// 射出に必要な情報(飛翔時間等)を計算
	calc_shot(future, shot_data);
	// 未来の位置を計算
	calc_pos_fut(now.myself, future.myself, shot_data.time + robot_static_data.time_lug1);
	calc_pos_fut(now.enemy, future.enemy, shot_data.time + robot_static_data.time_lug1);
	// 射出に必要な情報(飛翔時間等)を再計算
	// 未来の位置を再計算
//	ここまでの計算mainでやってもいいのでは？　ま、一旦そのままでやるけど

	// 未来の位置から、的中心への向きを求める
	calc_pos_of_target(future.myself, future.enemy);
	// 今の状態だとどの方向にどの方向に飛んでいくかを計算　とりま、今向いている角度でよくね？
	calc_initial_velocity(); // todo 飛んでいく角度を変数に入力？　とりあえずやらなくてよくない
	// どの的を狙うかを計算
	judge_target();
	// 発射パラメータ（ローラの速度、発射方向）を計算　とりま発射方向は的の角度でよくね？
// todo ここどうにかする　とりまローラ速度だけで考えることはいいだろ
	// pid等の処理をする
	calc_output();
}

} /* namespace at */
