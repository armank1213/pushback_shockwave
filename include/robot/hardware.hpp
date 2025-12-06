#pragma once

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"

// Controller
extern pros::Controller controller;

// Motor Groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// Individual Motors
extern pros::Motor outtakeMotor;
extern pros::Motor intakeMotor;
extern pros::Motor sortMotor;
extern pros::Motor middletakeMotor;

// Sensors
extern pros::Imu imu;
extern pros::Optical colorSensor;
extern pros::Distance distanceSensor;
extern pros::Distance LeftDistanceSensor;
extern pros::Distance RightDistanceSensor;
extern pros::Distance BackDistanceSensor;


// Pneumatics
extern pros::adi::Pneumatics matchLoad;
extern pros::adi::Pneumatics limiter;
extern pros::adi::Pneumatics wing;

// Tracking Wheels
extern pros::Rotation horizontal_rotation;
extern pros::Rotation vertical_rotation;

