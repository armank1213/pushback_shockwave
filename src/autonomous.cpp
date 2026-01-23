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
   
}

void blue_right_auton() {
    // TODO: Implement blue right autonomous

    limiter.set_value(0);
    chassis.setPose(0, 0, 0);
    // used to be chassis.moveToPose(-5, 20, 0, 1000, {.forwards=true, .lead=0.4, .maxSpeed=65});

    //chassis.moveToPose(-5, 20, 0, 1000, {.forwards=true, .lead=.0, .maxSpeed=65});
    intake(127);
    sort(127);
    outtake(-127);
    middleTake(127);
   // chassis.moveToPose(-2, 21, 1, 500, {.forwards=true, .lead=0, .maxSpeed=50}); // prev x = -3 and lead = 0 and maxSpeed = 50
    pros::delay(10);
  //  chassis.moveToPose(0, 25, 1, 1000, {.forwards=true, .lead=0, .maxSpeed=50}); // prev x = -3 and lead = 0 and maxSpeed = 50
   // pros::delay(250);
    chassis.moveToPose(0, -30, -1, 800, {.forwards=true, .lead=0, .maxSpeed=50});
    middleTake(0);
    chassis.moveToPose(0, -33, -64, 1000, {.forwards=true, .lead=0, .maxSpeed=50});
 
   // pros::delay(250);
    chassis.moveToPose(-20, -40, -67, 1500, {.forwards=true, .lead=0, .maxSpeed=55}); //25x 41y
    //chassis.moveToPose(18, 42, 94, 3000, {.forwards=true, .lead=.0, .maxSpeed=60});
    pros::delay(1250);
    intake(-127);
    sort(-127);
    outtake(127);
    middleTake(-127);
    pros::delay(1500);
    intake(0);
    sort(0);
    outtake(0);
    middleTake(0);

    chassis.moveToPose(34, -20, -200, 3000, {.forwards=false, .lead=0, .maxSpeed=110});
    pros::delay(20);
    matchLoad.set_value(1);
    //chassis.moveToPose(-34, -10, 200, 2500, {.forwards=true, .lead=0, .maxSpeed=90});
    chassis.moveToPose(40, 5, -200, 2500, {.forwards=true, .lead=0, .maxSpeed=10});
    intake(127); 
    sort(127);
    outtake(127);
    middleTake(-127);
    pros::delay(1500);
    intake(0);
    sort(0);
    outtake(0);
    middleTake(0);

    chassis.moveToPose(25, -42, -205, 2500, {.forwards=false, .lead=0, .maxSpeed=90});
    limiter.set_value(0);
    intake(127);
    sort(127);
    outtake(-127);
    middleTake(127);

}

void red_left_auton() {
    chassis.setPose(0, 0, 0);
    // used to be chassis.moveToPose(-5, 20, 0, 1000, {.forwards=true, .lead=0.4, .maxSpeed=65});

    //chassis.moveToPose(-5, 20, 0, 1000, {.forwards=true, .lead=.0, .maxSpeed=65});
    intake(127);
    sort(127);
   // outtake(-127);
    middleTake(127);
   // chassis.moveToPose(-2, 21, 1, 500, {.forwards=true, .lead=0, .maxSpeed=50}); // prev x = -3 and lead = 0 and maxSpeed = 50
    pros::delay(10);
  //  chassis.moveToPose(0, 25, 1, 1000, {.forwards=true, .lead=0, .maxSpeed=50}); // prev x = -3 and lead = 0 and maxSpeed = 50
   // pros::delay(250);
    chassis.moveToPose(0, 30, 1, 800, {.forwards=true, .lead=0, .maxSpeed=50});
    middleTake(0);
    
     pros::delay(50);
    
     
    chassis.moveToPose(0, 38, 67, 1000, {.forwards=true, .lead=0, .maxSpeed=50});
    matchLoad.set_value(1);
    chassis.moveToPose(1, 32, 67, 1000, {.forwards=true, .lead=0, .maxSpeed=50});
    //chassis.moveToPose(0, 34, 67, 1000, {.forwards=false, .lead=0, .maxSpeed=50});
   // pros::delay(250);
    chassis.moveToPose(20, 35, 67, 1500, {.forwards=true, .lead=0, .maxSpeed=55}); //25x 41y
    
    //chassis.moveToPose(18, 42, 94, 3000, {.forwards=true, .lead=.0, .maxSpeed=60});
    pros::delay(1250);
    intake(127);
    sort(127);
    outtake(127);
    middleTake(-127);
    pros::delay(750);
     middleTake(0);

    chassis.moveToPose(-35, 20, 205, 3000, {.forwards=false, .lead=0, .maxSpeed=110});
    pros::delay(20);
    intake(127); 
    sort(127);
    //chassis.moveToPose(-34, -10, 200, 2500, {.forwards=true, .lead=0, .maxSpeed=90});
    
    sort(127);
    //outtake(127);
    //middleTake(-127);
    pros::delay(1500);
    limiter.set_value(1);
   
    chassis.moveToPose(-24, 42, 210, 2500, {.forwards=false, .lead=0, .maxSpeed=90});
  
    intake(127);
    sort(127);
    outtake(-127);
    middleTake(127);

}

void red_right_auton() {

    limiter.set_value(0);
    chassis.setPose(0, 0, 0);
    // used to be chassis.moveToPose(-5, 20, 0, 1000, {.forwards=true, .lead=0.4, .maxSpeed=65});

    //chassis.moveToPose(-5, 20, 0, 1000, {.forwards=true, .lead=.0, .maxSpeed=65});
    intake(127);
    sort(127);
   // outtake(-127);
    middleTake(127);
   // chassis.moveToPose(-2, 21, 1, 500, {.forwards=true, .lead=0, .maxSpeed=50}); // prev x = -3 and lead = 0 and maxSpeed = 50
    pros::delay(10);
  //  chassis.moveToPose(0, 25, 1, 1000, {.forwards=true, .lead=0, .maxSpeed=50}); // prev x = -3 and lead = 0 and maxSpeed = 50
   // pros::delay(250);
    chassis.moveToPose(0, -30, -1, 800, {.forwards=true, .lead=0, .maxSpeed=50});
    middleTake(0);
    chassis.moveToPose(0, -33, -64, 1000, {.forwards=true, .lead=0, .maxSpeed=50});
 
   // pros::delay(250);
    chassis.moveToPose(-20, -40, -67, 1500, {.forwards=true, .lead=0, .maxSpeed=55}); //25x 41y
    //chassis.moveToPose(18, 42, 94, 3000, {.forwards=true, .lead=.0, .maxSpeed=60});
    pros::delay(1250);
    intake(127);
    sort(127);
    outtake(127);
    middleTake(-127);
    pros::delay(1500);
    intake(0);
    sort(0);
    outtake(0);
    middleTake(0);

    chassis.moveToPose(34, -20, -200, 3000, {.forwards=false, .lead=0, .maxSpeed=110});
    pros::delay(20);
    matchLoad.set_value(1);
    //chassis.moveToPose(-34, -10, 200, 2500, {.forwards=true, .lead=0, .maxSpeed=90});
    chassis.moveToPose(40, 5, -200, 2500, {.forwards=true, .lead=0, .maxSpeed=10});
    intake(127); 
    sort(127);
    //outtake(127);
    middleTake(-127);
    pros::delay(1500);
    intake(0);
    sort(0);
    outtake(0);
    middleTake(0);

    chassis.moveToPose(25, -42, -205, 2500, {.forwards=false, .lead=0, .maxSpeed=90});
    limiter.set_value(0);
    intake(127);
    sort(127);
    outtake(-127);
    middleTake(127);
}

