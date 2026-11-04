#include "robot/hardware.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// Drive: 4 blue motors, {front, back} per side.
pros::MotorGroup leftMotors({11, 12}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({13, 14}, pros::MotorGearset::blue);

// Individual Motors
pros::Motor cascadeRight(1, pros::v5::MotorGears::green);
pros::Motor cascadeLeft(-2, pros::v5::MotorGears::green);

// Sensors
pros::Imu imu(6);
pros::Rotation vertical_rotation(-19);
pros::Rotation horizontal_rotation(20);

// Perimeter distance sensors (localization).
pros::Distance ldist_sens(5);
pros::Distance rdist_sens(3);
pros::Distance fdist_sens(4);
pros::Distance bdist_sens(10);

// Pneumatics
pros::adi::Pneumatics matchLoad('E', false);
pros::adi::Pneumatics limiter('A', false);
pros::adi::Pneumatics wing('C', false);