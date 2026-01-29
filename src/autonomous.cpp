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
    chassis.setPose(0,0,90);
    chassis.moveToPoint(0,10,1000);
}

void red_left_auton() {
    int delta_t;
    int delta_x;
    int delta_y;

    chassis.setPose(0, 0, 0);

    chassis.moveToPose(0,33,0,1500);
    correct_y(33, 0, 0, 1000, false, false);

    /*delta_t = chassis.getPose().theta;
    if (delta_t != 0) {
        chassis.turnToHeading(-delta_t,500);
    }*/

    chassis.turnToHeading(90,1000);
    delta_t = chassis.getPose().theta;
    correct_t(90, 0, 32, 1000, false, true);


    //matchLoad.set_value(1);


    chassis.moveToPose(11,33,90,1000);
    delta_x = chassis.getPose().x;
    /*if (delta_x != 10) {
        chassis.moveToPose(10-delta_x,32,90,500);
    }*/
    correct_x(11, 33, 90, 1000, false, true); // final x is 11 but correction x is 10

    intake(127);
    pros::delay(2300);

    delta_t = chassis.getPose().theta;

    chassis.moveToPose(-22,33,90,3000, {.forwards=false}, false);

    //limiter.set_value(1);


    intake(127);
    outtake(127);

    /*
    chassis.moveToPose(10,32,95,500);
    delta_t = chassis.getPose().theta;
    chassis.moveToPose(10,32,95,500);
    delta_t = chassis.getPose().theta;
    */
/*
    int i = 0;
    for (i = 0; i < 5; i++) {
        chassis.turnToHeading(delta_t+20,500);
        chassis.turnToHeading(delta_t-20, 500);
    }
*/
    pros::delay(5000);
}

void red_right_auton() {

}