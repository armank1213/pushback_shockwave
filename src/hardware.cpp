#include "robot/hardware.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// Drive: 4 blue motors, {front, back} per side.
pros::MotorGroup leftMotors({-14, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({1, 12}, pros::MotorGearset::blue);

// Cascade Motors
pros::Motor cascadeRight(15, pros::v5::MotorGears::green);
pros::Motor cascadeLeft(-4, pros::v5::MotorGears::green);

// Intake Motor
pros::Motor intakeMotor(16, pros::v5::MotorGears::green);

// Sensors
pros::Imu imu(11);
pros::Rotation vertical_rotation(20);
pros::Rotation horizontal_rotation(19);

// Perimeter distance sensors (localization).
pros::Distance ldist_sens(5);
pros::Distance rdist_sens(3);
pros::Distance fdist_sens(4);
pros::Distance bdist_sens(10);

// Pneumatics
pros::adi::Pneumatics matchLoad('E', false);
pros::adi::Pneumatics limiter('A', false);
pros::adi::Pneumatics wing('C', false);