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
 *
 * @tparam T
 * @param x
 * @param in_min
 * @param in_max
 * @param out_min
 * @param out_max
 * @return
 */
template <typename T> T ArmoredTrain::map(T x, T in_min, T in_max, T out_min, T out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename T> T ArmoredTrain::suppress_value(T value, T max_abs_value){
	if (value > max_abs_value) {
		return max_abs_value;
	}else if (value < - max_abs_value) {
		return - max_abs_value;
	}else {
		return value;
	}
}


/**
 * センサーの入力を、SI単位系に変換する関数
 * @param sensor_data
 * @param movement_data
 */
void ArmoredTrain:: convert_to_SI(SensorData& prev_sensor_data, SensorData& sensor_data, RobotMovementData& movement_data) {
	movement_data.angle_of_turret			 = -map<float>(sensor_data.pot_angle_of_turret, 12928, 21960, -99.3/2*M_PI/180, 99.3/2*M_PI/180); // todo
	movement_data.position					 = (float)sensor_data.enc_position * RobotStaticData::enc_to_pos_ratio;
	movement_data.angular_velocity_of_truret = (sensor_data.pot_angle_of_turret	 - prev_sensor_data.pot_angle_of_turret	) * 0.01 * frequency;
	movement_data.velocity					 = (sensor_data.enc_position		 - prev_sensor_data.enc_position		) * 0.1 * frequency;
	movement_data.roller_rotation			 = (sensor_data.enc_roller_rotation	 - prev_sensor_data.enc_roller_rotation	) / 20.0f * 2*M_PI * frequency;
	prev_sensor_data = sensor_data;
}

/**
 * 砲弾の絶対座標系での初速度を計算する関数
 */
void ArmoredTrain::calc_initial_velocity(RobotMovementData& movement_data, BulletVelocity& bullet_velocity) {
	float abs_bullet_velocity = RobotStaticData::radius_of_roller * movement_data.roller_rotation;
	constexpr float cos_depression = std::cos(RobotStaticData::angle_of_depression);
	constexpr float sin_depression = std::sin(RobotStaticData::angle_of_depression);
	float cos_turret = std::cos(movement_data.angle_of_turret + M_PI_2);
	float sin_turret = std::sin(movement_data.angle_of_turret + M_PI_2);
	bullet_velocity.x = abs_bullet_velocity * cos_depression * cos_turret
						+ movement_data.velocity
						- RobotStaticData::turret_length * movement_data.roller_rotation * sin_turret;
	bullet_velocity.y = abs_bullet_velocity * cos_depression * sin_turret
						+ RobotStaticData::turret_length * movement_data.roller_rotation * cos_turret;
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

/**
 * 射出後の状態を計算する
 * @param movement_data
 * @param shot_data
 */
void ArmoredTrain::calc_shot(RobotMovementDataSet& movement_data, ShotData& shot_data){
	// 2台間の距離を計算
	float x = movement_data.myself.position + movement_data.enemy.position - FieldData::rail_length;
	float y = FieldData::opposing_distance;
	shot_data.l = std::sqrt(x*x + y*y);
	// どの速度なら届くかを計算
	constexpr float cos_depression = std::cos(RobotStaticData::angle_of_depression);
	constexpr float sin_depression = std::sin(RobotStaticData::angle_of_depression);
	shot_data.v0 = std::sqrt( (shot_data.l*FieldData::gravity) / (2*sin_depression*cos_depression) );
	// その速度でどのくらいの時間がかかるか計算
	shot_data.time = 2*shot_data.v0*sin_depression/FieldData::gravity;
}

/**
 * 本体位置から的位置を計算する
 * @param movement_data
 * @param mato
 */
void ArmoredTrain::calc_pos_of_mato(RobotMovementDataSet& movement_data, std::array<TargetPositionR, 3>& mato){
	for (uint8_t i = 0; i < 3; i++) {
		// 砲塔についていないターゲット
		float x = movement_data.myself.position + movement_data.enemy.position - FieldData::rail_length + RobotStaticData::mato[i].x;
		float y = FieldData::opposing_distance - RobotStaticData::mato[i].y;
		mato[i].l = std::sqrt(x*x + y*y);
		mato[i].angle = std::atan2(x, y);
	}
}

/**
 * どの的を狙うか、昇順補助するか、ロックオンできているか
 * @param mato
 * @param angle_of_shot
 * @return
 */
uint8_t ArmoredTrain:: judge_mato(std::array<TargetPositionR, 3>& mato, float& angle_of_shot) {
	uint8_t mato_num;
	float pos_e;
	float ang_e;
	float evaluation_value = 100000;
	constexpr float ratio = 0.7;
	for (uint8_t i = 0; i < 3; i++) {
		pos_e = std::pow(angle_of_shot - RobotStaticData::mato[i].angle, 2);
		ang_e = std::pow(angle_of_shot - mato[i].angle, 2);
		float this_evaluation_value = pos_e * ratio + ang_e * (1-ratio);
		if (this_evaluation_value < evaluation_value) {
			mato_num = i;
		}
	}
	if (ang_e < 0.1) { // ロックオンできたか
		mato_num = mato_num;
	}else if (ang_e < 0.5) { // ここ変数にする 補正範囲に入ったか
		mato_num = mato_num + 3;
	}else { // この的を狙う
		mato_num = mato_num + 6;
	}
	return mato_num;
}

/**
 * 出力構造体の計算
 * @param now
 * @param target
 * @param mato_num
 * @param input_data
 * @param output_data
 */
void ArmoredTrain::calc_output(RobotMovementData& now, RobotMovementData& target, uint8_t& mato_num, InputData& input_data, OutputData& output_data){
	/* motor */
	// 射出
	pid_roller.update_operation(target.roller_rotation - now.roller_rotation);
	output_data.compare[0] = pid_roller.get_operation();
	// 送り
	static ShotState state = ShotState::STOP;
	static uint32_t cooling_start;
	switch (state) {
		case ShotState::SHOTING_0:
			if (input_data.is_pusshed_lounch_reset == false) {
				state = ShotState::SHOTING_1;
			}
			output_data.compare[1] = 2000;
			break;
		case ShotState::SHOTING_1:
			if (input_data.is_pusshed_lounch_reset == true) {
				state = ShotState::COOLING_TIME;
				cooling_start = HAL_GetTick();
			}
			output_data.compare[1] = 2000;
			break;
		case ShotState::COOLING_TIME:
			if (cooling_start + 1000 <= HAL_GetTick()) {
				state = ShotState::STOP;
			}
			output_data.compare[1] = 0;
			break;
		case ShotState::STOP:
			if (input_data.ctrl.is_pulled_trigger == true) {
				state = ShotState::SHOTING_0;
			}
			output_data.compare[1] = 0;
			break;
	}
	// 横移動
	output_data.compare[2] = input_data.ctrl.right_handle * 0.8;
	// 砲塔旋回角度
	target.angle_of_turret = suppress_value<float>(target.angle_of_turret, RobotStaticData::turret_angle_max);
	pid_angle.update_operation(target.angle_of_turret - now.angle_of_turret);
	int16_t manual_operation_value = input_data.ctrl.left_handle * 0.3;
	if (mato_num < 6) {
		constexpr float ratio = 0.7; // 補正の強さ
		output_data.compare[3] += pid_angle.get_operation_difference();
		output_data.compare[3] = output_data.compare[3] * ratio + manual_operation_value * (1-ratio);
	}else{
		output_data.compare[3] = manual_operation_value;
	}
	// 最大角度の時、出力を正す
	if (now.angle_of_turret <= -RobotStaticData::turret_angle_max && output_data.compare[3] < 0) {
		output_data.compare[3] = 0;
	}else if(RobotStaticData::turret_angle_max <= now.angle_of_turret && output_data.compare[3] > 0){
		output_data.compare[3] = 0;
	}

	for (uint8_t i = 0; i < 4; i++) {
		// compare STARTの時に0にする
		if (prev_game_state == GameState::READY && input_data.game_state == GameState::START) {
			output_data.compare[i] = 0;
			prev_compare[i] = 0;
		}
		// compare 上限調整
		output_data.compare[i] = suppress_value<int16_t>(output_data.compare[i], 3900);
	}
	// compare 加速度調整
	output_data.compare[0] = prev_compare[0] + suppress_value(output_data.compare[0]-prev_compare[0], 1);
	prev_game_state = input_data.game_state;
	prev_compare = output_data.compare;

	/* LED等 */
	if (mato_num < 3) {
		output_data.lock_on = 1<<0;
	}else{
		output_data.lock_on = 0<<0;
	}
}

/**
 * update関数
 * @param input_data
 * @param output_data
 */
void ArmoredTrain::update(InputData& input_data, OutputData& output_data) {
	// SI単位系に変換
	convert_to_SI(prev_myself, input_data.myself, now.myself);
	convert_to_SI(prev_enemy, input_data.enemy, now.enemy);
	/* 未来の位置、および的への角度を計算 */
//	ここから
	// 射出時にいる位置を計算
	calc_pos_fut(now.myself, future.myself, RobotStaticData::time_lug1);
	calc_pos_fut(now.enemy, future.enemy, RobotStaticData::time_lug1);
	future0_myself = future.myself;
	// 射出に必要な情報(飛翔時間等)を計算
	calc_shot(future, shot_data);
	// 未来の位置を計算
	calc_pos_fut(now.myself, future.myself, shot_data.time + RobotStaticData::time_lug1);
	calc_pos_fut(now.enemy, future.enemy, shot_data.time + RobotStaticData::time_lug1);
	// 射出に必要な情報(飛翔時間等)を再計算
	// 未来の位置を再計算
//	ここまでの計算mainでやってもいいのでは？　ま、一旦そのままでやるけど
	// 未来の位置から、的中心への向きを求める
	calc_pos_of_mato(future, mato);

	/* 狙う的の決定 */
	// 今の状態だとどの方向にどの方向に飛んでいくかを計算　とりま、今向いている角度でよくね？
//	calc_initial_velocity(now.myself, bullet_velocity); // todo 飛んでいく角度を変数に入力？　とりあえずやらなくてよくない？
	// どの的を狙うかを計算
	uint8_t mato_num = judge_mato(mato, future0_myself.angle_of_turret);

	/* 射撃パラメータ(ローラー回転数、砲塔角度)の計算 */
	// 発射パラメータ（ローラの速度、発射方向）を計算　とりま発射方向は的の角度でよくね？
	target.roller_rotation = shot_data.v0 / 0.010;
	target.angle_of_turret = mato[mato_num % 3].angle;

	// pid等の処理をする
	calc_output(now.myself, target, mato_num, input_data, output_data);
}

} /* namespace at */
