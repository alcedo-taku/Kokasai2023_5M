/*
 * ConvertToSI.hpp
 *
 *  Created on: Oct 30, 2023
 *      Author: takuj
 */

#ifndef SRC_CONVERTTOSI_HPP_
#define SRC_CONVERTTOSI_HPP_

#include "data_type_robot.hpp"
#include "data_parameter.hpp"
#include <valarray>
#include <iostream>
#include <valarray>
#include <array>

class ConvertToSI {
private:
	std::array<float, 50> debug;
	SensorData prev_sensor_data;
	std::valarray<float> angle_buf = std::valarray<float>(0.0, 50);
	std::valarray<float> roller_rotation_buf = std::valarray<float>(0.0, 50);
public:
	ConvertToSI();
	template <typename T> T map(T x, T in_min, T in_max, T out_min, T out_max);
	void update(SensorData& sensor_data, RobotMovementData& movement_data);
};

#endif /* SRC_CONVERTTOSI_HPP_ */
