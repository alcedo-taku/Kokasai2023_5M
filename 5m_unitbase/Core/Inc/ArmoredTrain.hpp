/*
 * ArmoredTrain.hpp
 *
 *  Created on: Oct 15, 2023
 *      Author: takuj
 */

#ifndef ARMOREDTRAIN_HPP_
#define ARMOREDTRAIN_HPP_

#include "Automatic_Control_Assistant.hpp"

namespace at {

struct RobotSensorData {
	uint16_t enc_roller_rotation;
	int16_t enc_position;
	uint32_t pot_angle_of_turret;
};

struct InputData {
	RobotSensorData myself;
	RobotSensorData enemy;
	bool is_pusshed_lounch_reset;
};

struct OutputData {

};

struct FieldData{
	static constexpr float opposing_distance = 0.5;	//<! 対向距離()[m]
	static constexpr float rail_length = 2;			//<! 横移動可能長さ
	static constexpr float gravity = 9.8;			//<! 重力加速度
}field_data;

struct TargetPositionA {
	float x,y,angle;
};

struct RobotStaticData {
	static constexpr float angle_of_depression = 12;	//<! 砲塔俯角[rad]
	static constexpr float turret_length = 0.1;			//<! 旋回中心から速度計側部までの距離[m] (l0)
	static constexpr float radius_of_roller = 0.05;		//<! ローラー半径[m]
	static constexpr std::array<TargetPositionA, 3> target = {{	//<! ロボット内座標
			{0.01, 0.01, 12},
			{0.01, 0.01, 12},
			{0.01, 0.01, 12},
	}};
}robot_static_data;

struct TargetPositionR {
	float l,angle;
};

struct RobotMovementData {
	float position;							//<! 位置[m]
	float velocity;							//<! 速度[m/s]
	float angle_of_turret;					//<! 砲塔角度[rad]
	float angular_velocity_of_truret;		//<! 砲塔角速度[rad/s]
	float angle_of_depression;				//<! 砲塔俯角[rad]
	float roller_rotation;					//<! ローラー回転数[rad/s]
};

struct BulletVelocity {
	float x,y,z;
};

constexpr uint16_t frequency = 1000;


class ArmoredTrain {
private:
	RobotMovementData myself;		//<! 自分の動き
	RobotMovementData enemy;		//<! 敵の動き
	std::array<TargetPositionR, 3> target;	//<! 的の位置
	BulletVelocity bullet_velocity;
	InputData input_data;
	OutputData output_data;
	aca::PID_Element pid_parameter_position = {5,0,0};
	aca::PID_controller pid_position = aca::PID_controller (pid_parameter_position, frequency);
	aca::PID_Element pid_parameter_angle {5,0,0};
	aca::PID_controller pid_angle = aca::PID_controller (pid_parameter_angle, frequency);
	void calc_initial_velocity();	//<! 砲弾の初速度を求める
	void judge_locked_on();
	void convert_to_SI(RobotSensorData& sensor_data, RobotMovementData& movement_data);
	void calc_pos_of_target(RobotMovementData& mydata, RobotMovementData& enemydata);		//<! 本体の位置から的の位置を計算する
public:
	ArmoredTrain();
	void update(InputData& input_data, OutputData& output_data);
};

} /* namespace at */

#endif /* ARMOREDTRAIN_HPP_ */
