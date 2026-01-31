#include "robot/hardware.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
/*
    Motor Layout:
        Left Side Motors:
            - Port 6 = Front Motor
            - Port 5 = Reversed Motor (Top Stack Motor)
            - Port 4 = Bottom Stack Motor
        Right Side Motors:
            - Port 11 = Reversed Motor (Top Stack Motor)
            - Port 12 = Bottom Stack Motor
            - Port 7 = Front Motor
*/
/*
pros::MotorGroup leftMotors({-6, 5, -4}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({-11, 12, 7}, pros::MotorGearset::green);
*/
// Individual Motors
pros::Motor outtakeMotor(15, pros::v5::MotorGears::green); // outtake motor on port 15
pros::Motor intakeMotor(14, pros::v5::MotorGears::green); // intake motor on port 2
pros::Motor sortMotor(21, pros::v5::MotorGears::green); // sorting motor on port 1
pros::Motor middletakeMotor(14, pros::v5::MotorGears::green); // middletake motor on port 9

pros::Motor fullMotor(14, pros::v5::MotorGears::blue);
pros::Motor halfMotor(14, pros::v5::MotorGears::green);
/*
// Sensors
pros::Imu imu(16); // Inertial Sensor on port 16
*/
pros::Optical colorSensor(14); // Optical Sensor on port 19
pros::Distance distanceSensor(14); // Distance Sensor on port 14
pros::Distance LeftDistanceSensor(16);
pros::Distance RightDistanceSensor(13); 
pros::Distance BackDistanceSensor(14);

// Pneumatics
pros::adi::Pneumatics matchLoad('A', false); // Matchload Pneumatics on port A
pros::adi::Pneumatics limiter('C', false); // Limiter Pneumatics on port G
pros::adi::Pneumatics wing('E', false); // Wing Pneumatics on port C

/* // Tracking Wheels
// Horizontal tracking wheel encoder. Rotation sensor, port 10, not reversed
pros::Rotation horizontal_rotation(10);
// Vertical tracking wheel encoder. Rotation sensor, port 18, reversed
pros::Rotation vertical_rotation(18);
*/

pros::MotorGroup leftMotors({-8, -11, -3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({9, 20, 17}, pros::MotorGearset::blue);

pros::Imu imu(6); // Inertial Sensor on port 18
pros::Rotation horizontal_rotation(-18);
// Vertical tracking wheel encoder. Rotation sensor, port 19, reversed
pros::Rotation vertical_rotation(-19);