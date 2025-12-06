#include "main.h"
#include "robot/hardware.hpp"
#include "robot/chassis_config.hpp"
#include "robot/ui.hpp"
#include "robot/autonomous.hpp"
#include "robot/opcontrol.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    // Calibrate chassis (drivetrain, odometry, PID, sensors, controller steering)
    chassis.calibrate();

    // Reset rotation sensors
    vertical_rotation.reset();
    horizontal_rotation.reset();

    // Wait for IMU calibration
    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    // Initialize UI
    initializeUI();

    // Thread for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // Log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // Delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * Runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

/**
 * Runs during autonomous
 */
void autonomous() {
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Run intake/sort/outtake while chassis is in motion
    while (chassis.isInMotion()) {
        intakeMotor.move(127);
        sortMotor.move(127);
        outtakeMotor.move(-127);
    }

    // Set initial pose
    chassis.setPose(-54.422, -9.185, 0);

    // Auton selector
    switch (autonSelection) {
        case 1:
            blue_left_auton();
            break;
        case 2:
            blue_right_auton();
            break;
        case 3:
            red_left_auton();
            break;
        case 4:
            red_right_auton();
            break;
        default:
            // Default autonomous (currently red right)
            red_right_auton();
            break;
    }
}

/**
 * Runs during operator control
 * Implementation is in opcontrol.cpp
 */
