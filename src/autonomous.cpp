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
    // have to change this
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

void red_right_auton() {
    //chassis.follow(red_right_txt, 3, 4000,true,false);
    // TODO: Implement red right autonomous

    // Time Tracking
    double startTime = pros::millis();
    chassis.setPose(0, 0, 115);

    intake(127); 
    outtake(127);
    chassis.moveToPoint(-22.131, -24.413, 1000, {.maxSpeed=100, .minSpeed=20});
    pros::delay(1000);
    chassis.turnToPoint(-7.051, -6.814, 1000, {.maxSpeed=75, .minSpeed=20, .earlyExitRange=5});
    chassis.moveToPoint(-7.051, -6.814, 1000, {.maxSpeed=60, .minSpeed=20});
    intake(0);
    outtake(0);

    outtake(-127);
    intake(-127);
    pros::delay(2000);
    intake(0);
    outtake(0);

    chassis.moveToPoint(-48.593, -46.693, 1000, {.maxSpeed=100, .minSpeed=20});
    chassis.turnToPoint(-62.604, -47.387, 1000, {.maxSpeed=75, .minSpeed=20, .earlyExitRange=5});

    intake(127);
    outtake(127);
    matchloadToggle();
    chassis.moveToPoint(-62.604, -47.387, 1000, {.maxSpeed=50, .minSpeed=20});
    pros::delay(2500);
    intake(0);
    outtake(0);
    
    chassis.moveToPoint(-27.508, -47.61, 1000, {.maxSpeed=100, .minSpeed=20});
    matchloadToggle();
    limiterToggle();
    outtake(127);
    pros::delay(3000);

    controller.print(0,0,"Auton Time: %.2f", (pros::millis() - startTime)/1000);

}

