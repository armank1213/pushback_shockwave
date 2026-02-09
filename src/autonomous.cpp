#include "robot/autonomous.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"

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
    // MID ROUTE - TODO
    // make a route that intakes mid 3 blocks and scores them in top mid goal (this is left auton).
    // make that perfect, then after that go backwards with {.forwards=false} and then matchload.
    chassis.setPose(0,0,0);
    limiter.set_value(1);
    //collect 3 blocks
    chassis.moveToPose(0, 29, 0, 1500, {.forwards = true, .lead = 0, .maxSpeed = 80}, true);
    intake(127); // intake plus limiter
    outtake(127);
    pros::delay(1750);
    intake(0);
    // score in mid goal
    chassis.moveToPose(21, 37.5, -109, 2500, {.forwards = false, .lead = 0, .maxSpeed = 90}, false);
    intake(-80);
    outtake(-127);
    limiter.set_value(0);
    pros::delay(250);
    intake(127);
    outtake(-127);
    chassis.moveToPose(10, 37.5, -109, 1000, {.forwards = false, .lead = 0, .maxSpeed = 200}, false);
    chassis.moveToPose(21, 37.5, -109, 1000, {.forwards = false, .lead = 0, .maxSpeed = 200}, false);

    pros::delay(1000);
    // back up to align with matchload
    limiter.set_value(1);
    chassis.moveToPose(-36, 16, -155, 2250, {.forwards = true, .lead = 0, .maxSpeed = 90}, true);
    intake(50);
    chassis.moveToPose(-41, -1, -155, 1500, {.forwards = true, .lead = 0, .maxSpeed = 100}, true);
    //chassis.moveToPoint(-41, 3, 5000, {.forwards = true, .maxSpeed = 90}, true);
    matchLoad.set_value(1);
    intake(127);
    outtake(127);
    pros::delay(2500);
    chassis.moveToPose(-27, 32, -160, 5000, {.forwards = false, .lead = 0, .maxSpeed = 110}, true);
    matchLoad.set_value(0);
    pros::delay(1200);
    limiter.set_value(0);
    outtake(127);





    
//-38 5 -152
    // MATCHLOAD AUTONOMOUS ROUTE DO NOT USE FOR NOW TRY MID ROUTE ABOVE
    /*chassis.moveToPoint(0,32,1000);
    chassis.turnToHeading(-90,1000);
    matchLoad.set_value(1);
    chassis.moveToPoint(-11,32,1000,{.earlyExitRange=.5});
    limiter.set_value(1);
    intake(127);
    outtake(-100);
    pros::delay(3000);
    chassis.moveToPoint(11,32,1000,{.forwards=false});*/

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