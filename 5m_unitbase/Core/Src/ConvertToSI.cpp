/*
 * ConvertToSI.cpp
 *
 *  Created on: Oct 30, 2023
 *      Author: takuj
 */

#include "ConvertToSI.hpp"

ConvertToSI::ConvertToSI() {
	// TODO Auto-generated constructor stub

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
template <typename T> T ConvertToSI::map(T x, T in_min, T in_max, T out_min, T out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 *
 * @param sensor_data
 * @param movement_data
 */
void ConvertToSI::update(SensorData& sensor_data, RobotMovementData& movement_data){
	// angle_of_turret
	constexpr float max_angle = 99.3f/2*M_PI/180;
	angle_buf = angle_buf.shift(-1);
	for (uint8_t i = 0; i < 50; i++) {
		debug[i] = angle_buf[i];
	}
	angle_buf[0] = map<float>((float)sensor_data.pot_angle_of_turret, 22216.0f, 13120.0f, -max_angle, max_angle);
	//	angle[angle_div] = map<float>((float)sensor_data.pot_angle_of_turret, 21350.0f, 12330.0f, -max_angle, max_angle);
	movement_data.angle_of_turret 			 = angle_buf.sum() / 50.0f;

	// position
	movement_data.position					 = (float)sensor_data.enc_position * RobotStaticData::enc_to_pos_ratio;

	// angular_velocity_of_truret
	movement_data.angular_velocity_of_truret = (sensor_data.pot_angle_of_turret	 - prev_sensor_data.pot_angle_of_turret	) * 0.01 * frequency;

	// velocity
	movement_data.velocity					 = (sensor_data.enc_position		 - prev_sensor_data.enc_position		) * 0.1 * frequency;

	// roller_rotation
	roller_rotation_buf = roller_rotation_buf.shift(-1);
	roller_rotation_buf[0] = (sensor_data.enc_roller_rotation - prev_sensor_data.enc_roller_rotation	) / 20.0f * 2*M_PI * frequency;
	movement_data.roller_rotation			 = roller_rotation_buf.sum() / 50.0f;

	prev_sensor_data = sensor_data;
}
