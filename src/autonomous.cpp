#include "robot/autonomous.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep

// Get paths used for pure pursuit

void blue_left_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,30,3000, {.maxSpeed=100,.earlyExitRange=2});
}

void blue_right_auton() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,32,1000);
    chassis.turnToHeading(90,1000);
    matchLoad.set_value(1);
    chassis.moveToPoint(12,28,1000);
    chassis.turnToHeading(85,500);
    chassis.turnToHeading(95,500);
    chassis.turnToHeading(90,500);

   /* intake(127);
    outtake(127);
    limiter.set_value(1);
    pros::delay(2000);
    chassis.moveToPoint(-12,32,1000,{.forwards=false});*/
    /*
    chassis.moveToPoint(10,30,1000);
    intake(127);
    middletakeMotor.move(127);
    outtakeMotor.move(-127);
    pros::delay(3000);
    chassis.moveToPoint(-16,31,3000, {.forwards=false});*/
}

void red_left_auton() {
    chassis.setPose(0,0,0);

    limiter.set_value(1);
    intakeMotor.move(-127);
    middleMotor.move(-127);
    outtake(127);
    //chassis.moveToPose(4, 15, 9, 1500, {.forwards = true, .lead = 0.5, .maxSpeed = 80}, true);
    // pros::delay(250);
    chassis.moveToPose(0, 32, -7, 2500, {.forwards = true, .lead = 0.8, .minSpeed=17}, false);
    //chassis.turnToHeading(-96, 2000);
    chassis.moveToPose(0,27, -114, 2500, {.forwards = false, .lead = 0, .minSpeed=40}, false);
    limiter.set_value(0);
    //middleMotor.move(100);
    middleMotor.move(-127);
    outtake(-127);
    pros::delay(1500);
    //chassis.moveToPose(-48, -15, -150, 3000, {.forwards = true, .lead = .6, .minSpeed=67}, false);
    chassis.moveToPoint(-44, -10, 3000, {.forwards=true, .minSpeed=55});
    chassis.turnToHeading(-150, 1000);
    matchLoad.set_value(1);
    chassis.moveToPoint(-44, -25, 3000, {.forwards=true, .minSpeed=35}, true);
    limiter.set_value(1);
    intakeMotor.move(-127);
    middleMotor.move(-127);
    outtake(127);
    chassis.turnToHeading(-154, 1000);
    chassis.moveToPoint(-34, 2, 2000, {.forwards=false, .minSpeed=45}, false);
    limiter.set_value(0);
    outtake(127);
    middleMotor.move(-127);
    
    
    /*matchLoad.set_value(1);
    limiter.set_value(1);
    chassis.moveToPose(-50, -20, -152, 1500, {.forwards = true, .lead = 0, .minSpeed=35}, false);
    intakeMotor.move(-127);
    middleMotor.move(-127);
    outtake(127);*/

}

void red_right_auton() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(-5, 20, 0, 1000, {.forwards = true, .lead = .4, .maxSpeed = 60}, false);
    

}

void skills() {
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,33.5,1000);
    chassis.turnToHeading(-90,1000);
    matchLoad.set_value(1);
    chassis.moveToPoint(-12.5,35.5,1000);
    limiter.set_value(1);
    intake(127);
}
