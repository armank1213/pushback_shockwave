#include "main.h"
#include "robot/hardware.hpp"
#include "robot/chassis_config.hpp"
#include "robot/ui.hpp"
#include "robot/autonomous.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "robot/auton_helpers.hpp"// IWYU pragma: keep
#include "robot/motors.hpp"
#include "robot/color_sort.hpp" 
#include "robot/pneumatics.hpp"

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

    red_left_auton();
}

void opcontrol() {

    intakeMotor.set_voltage_limit(5500);
    
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // Matchload piston variables
    bool matchloadTogle = false;
    static bool lastAButtonState = false;

    // Limiter piston variables
    bool limiterPistonToggle = false;
    static bool lastYButtonState = false;

    // Wing piston variables
    bool wingPistonToggle = false;
    static bool lastXButtonState = false;

    // Anti-jam control variables
    const bool allianceColor = true; // true for red, false for blue
    bool isOurBlock = false;

    while (true) {
        // Get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Chassis Drive Functions
        chassis.arcade(leftY, rightX, false, .3);

        // Intake and outtake control functions
        intakeControl();
        outtakeControl();
        
        // Set light to 100% and get distance and color readings
        colorSensor.set_led_pwm(100);
        int distance = distanceSensor.get_distance();
        double hue = colorSensor.get_hue();

        // Color sorting functions
        if (colorSortMode == 0) {
            red_colorSort(distance, hue);
        }
        else if (colorSortMode == 1) {
            blue_colorSort(distance, hue);
        }
        else {
            skills_colorSort(distance,hue);
        }

        // Matchload Pneumatics Toggle
        matchloadToggle();

        // Limiter Pneumatics Toggle
        limiterToggle();

        // Wing Mech Pneumatics Toggle
        wingToggle();


        // Logic for anti-jam control
        if (distance < 135) {
            if (allianceColor && ((hue <= 360 && hue >= 300) || (hue >= 0 && hue <= 35))) {
                isOurBlock = true;
            } else if (!allianceColor && (hue <= 250 && hue >= 180)) {
                isOurBlock = true;
            }
        }
        bool antiJamButtonPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
        antiJamControl(antiJamButtonPressed, isOurBlock);

        // Delay to save resources
        lv_timer_handler();
        pros::delay(30);
    }
}
