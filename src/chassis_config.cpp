#include "robot/chassis_config.hpp"
#include "robot/hardware.hpp"

// Tracking wheels (2.75" omnis). Offsets are from the tracking center.
// Horizontal wheel: 5.75" behind center (negative).
lemlib::TrackingWheel horizontal_wheel(&horizontal_rotation, lemlib::Omniwheel::NEW_275, -5.75);
// Vertical wheel: 2.5" left of center (negative).
lemlib::TrackingWheel vertical_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_275, -2.5);

// Drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // track width, inches
                              lemlib::Omniwheel::NEW_275, // 2.75" drive omnis
                              360, // drivetrain rpm
                              8 // horizontal drift
);

// Lateral PID controller
lemlib::ControllerSettings lateralController(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// Angular PID controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Sensors for odometry (two tracking wheels + IMU).
lemlib::OdomSensors sensors(&vertical_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// Input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// Input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// Create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors, &throttleCurve, &steerCurve);

void initializeChassis() {
    // Calibration is done in main.cpp initialize() function
}

