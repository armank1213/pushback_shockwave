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

    while (true) {
        pros::Task updatePoseTask(newPose,nullptr,(uint32_t)TASK_PRIORITY_DEFAULT,(uint16_t)TASK_STACK_DEPTH_DEFAULT);
    }

    intakeMotor.move(127);
    sortMotor.move(127);
    middletakeMotor.move(127);
    outtakeMotor.move(-127);
    
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,30,1000);
    chassis.turnToHeading(-60,1000, {.minSpeed=50, .earlyExitRange=2});
    //chassis.moveToPoint(0,15,1000);
    //intakeMotor.move(-127);
    //sortMotor.move(-127);
    //middletakeMotor.move(-127);
    //outtakeMotor.move(-127);
    //pros::delay(2000);
    //intakeMotor.stop();
    //sortMotor.stop();
    //middletakeMotor.stop();
    //chassis.move(0,-60,1000);
    //chassis.moveToHeading(60,1000,{.minSpeed=50, .earlyExitRange=2});
    //
}

