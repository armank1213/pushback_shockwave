#include "main.h"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups

/*
    Motor Layout:
        Left Side Motors:
            - Port 18 = Front Motor
            - Port 19 = Reversed Motor (Top Stack Motor)
            - Port 20 = Bottom Stack Motor
        Right Side Motors:
            - Port 11 = Reversed Motor (Top Stack Motor)
            - Port 12 = Bottom Stack Motor
            - Port 13 = Front Motor
*/
pros::MotorGroup leftMotors({18, -19, 20}, pros::MotorGearset::green); // left motor group - ports 18, 19 (reversed), 20
pros::MotorGroup rightMotors({-11, 12, 13}, pros::MotorGearset::green); // right motor group - ports 11 (reversed), 12, 13

// Inertial Sensor on port 1
pros::Imu imu(1);

// Motors
pros::Motor outtakeMotor(5, pros::v5::MotorGears::green); // outtake motor on port 9
pros::Motor intakeMotor(-6, pros::v5::MotorGears::green); // intake motor on port 6

pros::Motor sortMotor(16, pros::v5::MotorGears::green); // sorting motor on port 16
pros::Motor middletakeMotor(15, pros::v5::MotorGears::green); // middletake motor on port 15

// long goal outtake motor group
pros::MotorGroup longTake({5, -6}); 

// middle goal outtake motor group
pros::MotorGroup middleTake({-5, 15});

// vision sensor signature IDs
// pros::Vision visionSensor(8);
// pros::vision_signature_s_t BLUE_SIG  = pros::Vision::signature_from_utility(1, -4089, -2329, -3210, 2711, 4961, 3836, 2.100, 0);
// pros::vision_signature_s_t RED_SIG  = pros::Vision::signature_from_utility(2, 4861, 11873, 8368, -1889, -225, -1058, 1.300, 0);

// Optical Sensor
pros::Optical colorSensor(10);

// Distance Sensor 
pros::Distance distanceSensor(7);

// Pneumatics
pros::adi::Pneumatics matchLoad('H', false);

// tracking wheels
// 11 inch long
// 13.5 inch wide
// Tracking center (6.875, 5.5)
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontal_rotation(-2);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation vertical_rotation(3);
// horizontal tracking wheel. 2" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal_wheel(&horizontal_rotation, lemlib::Omniwheel::NEW_2, -1);
// vertical tracking wheel. 2" diameter, 0.37" offset, left of the robot (negative)
lemlib::TrackingWheel vertical_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_2, 0.0625);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.75, // 12.75 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral PID controller
lemlib::ControllerSettings lateralController(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
// Tracking center: (8,8.25)
// Vertical Tracking Wheel Offset: 8-7.63 = 0.37 left of the tracking center (-0.37)
// Horizontal Tracking Wheel Offset: 


lemlib::OdomSensors sensors(&vertical_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    vertical_rotation.reset(); // reset vertical rotation sensor
    horizontal_rotation.reset(); // reset horizontal rotation sensor
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // pros::lcd::print(3, "Vertical Rotation Sensor: %i", vertical_rotation.get_position());
            // pros::lcd::print(4, "Horizontal Rotation Sensor: %i", horizontal_rotation.get_position());

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during autonomous
 */
void autonomous() {

    // Angular PID Tuning 

    chassis.setPose(0, 0,0);
    chassis.turnToHeading(90, 100000);

    /* Lateral PID Tuning 
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(24, 0, 100000);
    */
}


void opcontrol() {
    
	void long_goal();

	// void manual_sort();

	// void redSort(int sortMode, int distance, double hue);

    void blueSort(int sortMode, int distance, double hue);

    void middle_goal();

    void antiJamControl();

    bool pistonToggle = false;
    static bool lastAButtonState = false;

    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the chassis with curvature drive
		// chassis.tank(leftY, rightX);
        // chassis.curvature(-rightX, -leftY, false);
        
        chassis.arcade(-rightX, -leftY, false, .3);

        long_goal();

        middle_goal();


        int distance = distanceSensor.get_distance();

        double hue = colorSensor.get_hue();

        static int sortMode = 0;
        static bool LastButtonState = false;

        bool CurrentButtonState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (CurrentButtonState && !LastButtonState) {
            sortMode = 1 - sortMode;
        }

        LastButtonState = CurrentButtonState;


        // redSort(sortMode, distance, hue);

        blueSort(sortMode, distance, hue);

        antiJamControl();

        // manual_sort();

        // Pneumatics Toggle
        bool currentA = controller.get_digital(DIGITAL_A);

        if (currentA && !lastAButtonState) {
            pistonToggle = !pistonToggle;
            if (pistonToggle) {
                matchLoad.extend();
            } else {
                matchLoad.retract();
            }
        }
        lastAButtonState = currentA;

        // delay to save resources
        pros::delay(25);
    }
}

// Intake/Outtake Motor Function
void longout(int in_out_power) {
	longTake.move(in_out_power);
}

// Sort Motor Function
void sort(int sortPower) {
	sortMotor.move(sortPower);
}


// Middle Take Motor Function
void middleout(int middletakePower) {
    middleTake.move(middletakePower);
}

void middle_goal() {
    if (controller.get_digital(DIGITAL_L1)) {
        middleout(127);
    } 
    else if (controller.get_digital(DIGITAL_L2)) {
        middleout(-127);
    }
    else {
        middleout(0);
    }
}

// Manual Intake/Outtake
void long_goal() {
	if (controller.get_digital(DIGITAL_R1)) {
		longout(-127);
	}
	else if (controller.get_digital(DIGITAL_R2)) {
      longout(127);
    }
	else {
      longout(0);
    }
}

// Manual Sorting
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

// Color Sorting 
void redSort(int sortMode, int distance, double hue) {

    if (distance < 135) { // if an object is detected within 135mm
        if (sortMode == 0) { 
            if ((hue <= 360 && hue >= 300) || (hue >= 0 && hue <= 35)) {
                sort(127);
            } else if (hue <= 250 && hue >= 180) {
                sort(-127);
            } else {
                sort(0);
            }
        }

        else if (sortMode == 1) {
            if (hue <= 250 && hue >= 180) {
                sort(127);
            } else if ((hue <= 360 && hue >= 300) || (hue >= 0 && hue <= 35)) {
                sort(127);
            } else {
                sort(0);
            }
        }
    } else {
        sort(0);
    }
}

void blueSort(int sortMode, int distance, double hue) {

    if (distance < 135) { // if an object is detected within 135mm
        if (sortMode == 0) { 
            if (hue <= 250 && hue >= 180) {
                sort(127);
            } else if ((hue <= 360 && hue >= 300) || (hue >= 0 && hue <= 35)) {
                sort(-127);
            } else {
                sort(0);
            }
        }

        else if (sortMode == 1) {
            if ((hue <= 360 && hue >= 300) || (hue >= 0 && hue <= 35)) {
                sort(127);
            } else if (hue <= 250 && hue >= 180) {
                sort(127);
            } else {
                sort(0);
            }
        }
    } else {
        sort(0);
    }
}

void antiJamControl() {

    static int lastToggleTime = 0;
    static int direction = 1;
    static bool antiJamActive = false;


    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        antiJamActive = true;
        int currentTime = pros::millis();

        if (currentTime - lastToggleTime > 500) {
            direction = (rand() % 2 == 0) ? 1: -1; // Randomly choose direction
            lastToggleTime = currentTime;
        }

        intakeMotor.move(127 * direction);
        sortMotor.move(127 * direction);
        middletakeMotor.move(-127 * direction);
        outtakeMotor.move(-127 * direction);
        
    } else {
        if (antiJamActive) {
            intakeMotor.move(0);
            sortMotor.move(0);
            middletakeMotor.move(0);
            outtakeMotor.move(0);
            antiJamActive = false;
        }
    }
}
