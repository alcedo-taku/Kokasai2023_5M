/*
 * ArmoredTrain.cpp
 *
 *  Created on: Oct 15, 2023
 *      Author: takuj
 */

#include "ArmoredTrain.hpp"

namespace at {

ArmoredTrain::ArmoredTrain() {
//	pid_position = aca::PID_controller (pid_parameter_position, frequency);
}

void ArmoredTrain::calc_initial_velocity() {

}

void ArmoredTrain:: judge_locked_on() {

}


void ArmoredTrain:: convert_to_SI(RobotSensorData& sensor_data, RobotMovementData& movement_data) {
	static RobotSensorData prev_sensor_data;
	movement_data.angle_of_turret			 = sensor_data.pot_angle_of_turret * 0.01;
	movement_data.position					 = sensor_data.enc_position * 0.1;
	movement_data.angular_velocity_of_truret = (sensor_data.pot_angle_of_turret	 - prev_sensor_data.pot_angle_of_turret	) * 0.01 * frequency;
	movement_data.velocity					 = (sensor_data.enc_position		 - prev_sensor_data.enc_position		) * 0.1 * frequency;
	movement_data.roller_rotation			 = (sensor_data.enc_roller_rotation	 - prev_sensor_data.enc_roller_rotation	) * 0.001 * frequency;
}

void ArmoredTrain::update(InputData& input_data, OutputData& output_data) {
	this->input_data = input_data;
	convert_to_SI(input_data.myself, myself);
	convert_to_SI(input_data.enemy, enemy);
}

} /* namespace at */
