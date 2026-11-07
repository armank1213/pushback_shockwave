#pragma once

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

// Controller
extern pros::Controller controller;

// Motor Groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// Cascade Motors
extern pros::Motor cascadeRight;
extern pros::Motor cascadeLeft;

// Intake Motor
extern pros::Motor intakeMotor;

// Claw Motors
extern pros::Motor clawLiftMotor;
extern pros::Motor clawSpinMotor;

// Sensors
extern pros::Imu imu;

extern pros::Distance ldist_sens;
extern pros::Distance rdist_sens;
extern pros::Distance fdist_sens;
extern pros::Distance bdist_sens;

// Pneumatics
extern pros::adi::Pneumatics matchLoad;
extern pros::adi::Pneumatics limiter;
extern pros::adi::Pneumatics wing;
extern pros::adi::Pneumatics claw; 

// Tracking Wheels
extern pros::Rotation horizontal_rotation;
extern pros::Rotation vertical_rotation;

extern pros::Motor verticalLiftMotor;
