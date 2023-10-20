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
 * 砲弾の初速度を計算する関数
 */
void ArmoredTrain::calc_initial_velocity() {
	float abs_bullet_velocity = robot_static_data.radius_of_roller * myself.roller_rotation;
	float cos_depression = std::cos(robot_static_data.angle_of_depression);
	float sin_depression = std::sin(robot_static_data.angle_of_depression);
	float cos_turret = std::cos(myself.angle_of_turret + M_PI_2);
	float sin_turret = std::sin(myself.angle_of_turret + M_PI_2);
	bullet_velocity.x = abs_bullet_velocity * cos_depression * cos_turret
						+ myself.velocity
						- robot_static_data.turret_length * myself.roller_rotation * sin_turret;
	bullet_velocity.y = abs_bullet_velocity * cos_depression * sin_turret
						+ robot_static_data.turret_length * myself.roller_rotation * cos_turret;
	bullet_velocity.z = abs_bullet_velocity * sin_depression;
}

/**
 * 未来の位置を予測する
 * @param movement_data_now
 * @param movement_data_fut
 * @param time_lug
 */
void ArmoredTrain::calc_pos_fut(RobotMovementData& movement_data_now, RobotMovementData movement_data_fut, uint16_t time_lug){
	// 今の直線距離から、時間を決める

	// その時間から未来の時間を決める

	// 未来の距離から時間を決める
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
	convert_to_SI(input_data.myself, myself);
	convert_to_SI(input_data.enemy, enemy);
}

} /* namespace at */
