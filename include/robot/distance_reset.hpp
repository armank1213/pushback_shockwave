#pragma once

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/chassis_config.hpp" // IWYU pragma: keep


double angle_error(double angle_a, double angle_b);

std::pair<double,double> reset_distance();