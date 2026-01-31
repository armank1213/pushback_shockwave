#include "robot/autonomous.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"

// Get paths used for pure pursuit

void blue_left_auton() {
    
}

void blue_right_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,10,1000);
    chassis.turnToHeading(90,1000);
    /*
    chassis.moveToPoint(10,30,1000);
    intake(127);
    middletakeMotor.move(127);
    outtakeMotor.move(-127);
    pros::delay(3000);
    chassis.moveToPoint(-16,31,3000, {.forwards=false});*/
}

void red_left_auton() {

}

void red_right_auton() {

}