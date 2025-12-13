#include "robot/autonomous.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/distance_reset.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"

// Get paths used for pure pursuit
ASSET(red_right_txt); // '.' replaced with "_" to make c++ happy
ASSET(red_right_a_txt);

void newPose(void* param) {
    std::pair <double,double> coords = reset_distance();
    double theta = chassis.getPose().theta;
    lemlib::Pose updatedPose(coords.first, coords.second, theta);
    chassis.setPose(updatedPose);
}

void blue_left_auton() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(-5, 20, 0, 1000, {.forwards=true, .lead=0.4, .maxSpeed=65});
    chassis.moveToPose(-12, 42, 0, 3000, {.forwards=true, .lead=0, .maxSpeed=65});
    intake(127);
    sort(127);
    outtake(127);
    chassis.moveToPose(5, 45, 45, 2500, {.forwards=true, .lead=0.0, .maxSpeed=55});
    pros::delay(2000);
    outtake(-127);
    sort(-127);
    middleTake(-127);
    pros::delay(2500);
}

void blue_right_auton() {
    // TODO: Implement blue right autonomous
}

void red_left_auton() {

    chassis.setPose(0, 0, 0);
    // used to be chassis.moveToPose(-5, 20, 0, 1000, {.forwards=true, .lead=0.4, .maxSpeed=65});
    
    chassis.moveToPose(-15, 18, 0, 1000, {.forwards=true, .lead=.3, .maxSpeed=60});
    chassis.moveToPose(-20, 37, 0, 3000, {.forwards=true, .lead=.1, .maxSpeed=80});
    sort(127);
    outtake(-127);
    middleTake(127);
    pros::delay(2000);
    outtake(127);
    sort(-127);
    middleTake(-127);
    pros::delay(2000);

    /*chassis.moveToPose(-25,4.5,135,3500, {.forwards=false, .lead=0, .maxSpeed=110});
    matchLoad.set_value(1);
    sort(127);
    middleTake(127);
    outtake(127);
    chassis.moveToPose(-25,-10.5,0,2200, {.forwards=true, .lead=0, .maxSpeed=90});
    pros::delay(2500);
    chassis.moveToPose(-25,-24,0,2200, {.forwards=false, .lead=0, .maxSpeed=110});*/

    


}

void red_right_auton() {

}

