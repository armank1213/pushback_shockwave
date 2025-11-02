#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drivetrain motor groups
pros::MotorGroup leftMotors({6, -5, 4}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({-11, 12, 7}, pros::MotorGearset::green);

// Individual Motors
pros::Motor middletakeMotor(9, pros::v5::MotorGears::green);
pros::Motor outtakeMotor(13, pros::v5::MotorGears::green);
pros::Motor intakeMotor(2, pros::v5::MotorGears::green);
pros::Motor sortMotor(1, pros::v5::MotorGears::green); // Sorting motor on Port 1

// Sensors
pros::Optical colorSensor(19); // Port 19
pros::Distance distanceSensor(14); // Port 14

// --- SENSORS FOR ODOMETRY ---
pros::Rotation horizontal_rotation(-10);
pros::Rotation vertical_rotation(18);
lemlib::TrackingWheel horizontal_wheel(&horizontal_rotation, lemlib::Omniwheel::NEW_2, -1);
lemlib::TrackingWheel vertical_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_2, 0.0625);
// --- END OF ODOMETRY SENSORS ---


// Drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              12.75,
                              lemlib::Omniwheel::NEW_325,
                              360,
                              2
);

// PID controllers
lemlib::ControllerSettings lateralController(10, 0, 3, 0, 0, 0, 0, 0, 0);
lemlib::ControllerSettings angularController(2, 0, 10, 0, 0, 0, 0, 0, 0);

// Odometry sensors object
lemlib::OdomSensors sensors(&vertical_wheel,
                            nullptr,
                            &horizontal_wheel,
                            nullptr,
                            nullptr
);

// Drive curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// Chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors, &throttleCurve, &steerCurve);

// Function Declarations
void sort(int power);
void manual_sort();
void run_color_sorting(int sortMode, int distance, double hue);


/**
 * Runs initialization code.
 */
void initialize() {
    pros::lcd::initialize();
    colorSensor.set_led_pwm(100);

    // Reset and calibrate odometry sensors
    vertical_rotation.reset();
    horizontal_rotation.reset();
    chassis.calibrate();

    // Set brake modes for all motors to coast
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    middletakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    outtakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    sortMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


    // Task to display odometry readings on the brain screen
    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

/**
 * Required empty functions.
 */
void disabled() {}
void competition_initialize() {}
void autonomous() {}

/**
 * Runs the user control loop.
 */
void opcontrol() {
    int sortMode = 0;
    bool lastBButtonState = false;

    while (true) {
        // Drivetrain Control
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(-rightX, -leftY, false, .3);

        // --- Your Working Motor Code ---
        // Outtake Section
        if (controller.get_digital(DIGITAL_L1)) {
            middletakeMotor.move(127);
            outtakeMotor.move(-127);
        }
        else if (controller.get_digital(DIGITAL_L2)) {
            middletakeMotor.move(-127);
            outtakeMotor.move(127);
        }
        else {
            middletakeMotor.move(0);
            outtakeMotor.move(0);
        }
        // Intake Section
        if (controller.get_digital(DIGITAL_R1)) {
            intakeMotor.move(127);
        }
        else if (controller.get_digital(DIGITAL_R2)) {
            intakeMotor.move(-127);
        }
        else {
           intakeMotor.move(0);
        }
        // --- End of Working Code ---


        // --- Color Sorting Section ---
        // Toggle sorting mode with 'B' button
        bool currentBButtonState = controller.get_digital(DIGITAL_B);
        if (currentBButtonState && !lastBButtonState) {
            sortMode = 1 - sortMode; // Toggles between 0 and 1
        }
        lastBButtonState = currentBButtonState;

        // Get sensor data
        int distance = distanceSensor.get();
        double hue = colorSensor.get_hue();

        // Run the sorting logic
        run_color_sorting(sortMode, distance, hue);

        // Add a small delay
        pros::delay(25);
    }
}


/**
 * Helper function to control the sort motor.
 */
void sort(int power) {
    sortMotor.move(power);
}

/**
 * Helper function for manual sorting control.
 */
void manual_sort() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      sort(127);
    }
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      sort(-127);
    }
	else {
      sort(0);
    }
}

/**
 * Main color sorting logic (for Red Alliance).
 */
void run_color_sorting(int sortMode, int distance, double hue) {
    if (distance < 135) { // If an object is detected
        if (sortMode == 0) { // Mode 0: Fully Automatic
            if (hue >= 0 && hue <= 35) { // If Red
                sort(127);
            } else if (hue >= 200 && hue <= 350) { // If Blue
                sort(-127);
            } else {
                sort(0); // No clear color
            }
        }
        else if (sortMode == 1) { // Mode 1: Manual Override for Blue
            if (hue >= 0 && hue <= 35) { // If Red
                sort(127); // Still auto-sort red
            } else if (hue >= 200 && hue <= 350) { // If Blue
                manual_sort(); // Allow manual control for blue
            } else {
                sort(0);
            }
        }
    } else { // No object detected
        sort(0);
    }
}