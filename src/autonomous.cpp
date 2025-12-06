#include "robot/autonomous.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"

// Get paths used for pure pursuit
ASSET(red_right_txt); // '.' replaced with "_" to make c++ happy
ASSET(red_right_a_txt);

void blue_left_auton() {
    // TODO: Implement blue left autonomous
}

void blue_right_auton() {
    // TODO: Implement blue right autonomous
}

void red_left_auton() {
    // TODO: Implement red left autonomous
}

void red_right_auton() {
    //chassis.follow(red_right_txt, 3, 4000,true,false);
    // TODO: Implement red right autonomous
}

