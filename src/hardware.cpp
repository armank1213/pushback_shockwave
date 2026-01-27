#include "robot/hardware.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
/*
    L-Bot Motor Layout:
        Left Side Motors:
            - Port 6 = Front Motor
            - Port 5 = Reversed Motor (Top Stack Motor)
            - Port 4 = Bottom Stack Motor
        Right Side Motors:
            - Port 11 = Reversed Motor (Top Stack Motor)
            - Port 12 = Bottom Stack Motor
            - Port 7 = Front Motor

    S-Bot Motor Layout:
        Left Side Motors:
            - Port 8 = Front Motor
            - Port 11 = Middle Motor
            - Port 3 = Back Motor (Vertical)
        Right Side Motors:
            - Port 9 = Front Motor
            - Port 20 = Middle Motor
            - Port 10 = Back Motor (Vertical)
*/
pros::MotorGroup leftMotors({8, 11, 3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({9, 20, 10}, pros::MotorGearset::blue);

// Individual Motors
pros::Motor outtakeMotor(2, pros::v5::MotorGears::green);
pros::Motor intakeMotor(5, pros::v5::MotorGears::green);
pros::Motor middleMotor(1, pros::v5::MotorGears::blue);

// Sensors
pros::Imu imu(16);
pros::Optical colorSensor(19);
pros::Distance distanceSensor(14);
pros::Distance LeftDistanceSensor(16);
pros::Distance RightDistanceSensor(13);
pros::Distance BackDistanceSensor(17);

// Pneumatics
pros::adi::Pneumatics matchLoad('A', false);
pros::adi::Pneumatics limiter('C', false);
pros::adi::Pneumatics wing('E', false);

// Tracking Wheels
// Horizontal tracking wheel encoder
pros::Rotation horizontal_rotation(10); // redo all ports
// Vertical tracking wheel encoder
pros::Rotation vertical_rotation(18); // redo all ports