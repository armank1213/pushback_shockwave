#include "main.h"
#include "robot/hardware.hpp"
#include "robot/chassis_config.hpp"
#include "robot/ui.hpp"
#include "robot/autonomous.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "robot/distance_reset.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" 
#include "robot/color_sort.hpp" 

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

void opcontrol() {
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Matchload piston variables
    bool matchloadTogle = false;
    static bool lastAButtonState = false;

    // Limiter piston variables
    bool limiterPistonToggle = false;
    static bool lastYButtonState = false;

    // Wing piston variables
    bool wingPistonToggle = false;
    static bool lastXButtonState = false;

    // Color sorting variables
    static int sortMode = 0;
    static bool LastB_ButtonState = false;

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

        // Logic for sorting mode toggle
        bool CurrentB_ButtonState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (CurrentB_ButtonState && !LastB_ButtonState) {
            sortMode = 1 - sortMode;
        }
        LastB_ButtonState = CurrentB_ButtonState;

        // Color sorting functions
        if (colorSortMode == 0) {
            red_colorSort(sortMode, distance, hue);
        }
        else if (colorSortMode == 1) {
            blue_colorSort(sortMode, distance, hue);
        }

        // Matchload Pneumatics Toggle
        matchloadToggle();

        // Limiter Pneumatics Toggle
        limiterToggle();

        // Wing Mech Pneumatics Toggle
        bool currentX = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        
        if (currentX && !lastXButtonState) {
            wingPistonToggle = !wingPistonToggle;
            wing.set_value(wingPistonToggle);
        }
        lastXButtonState = currentX;

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
