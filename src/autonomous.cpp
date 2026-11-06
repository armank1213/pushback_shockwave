#include "robot/autonomous.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "robot/distance_reset.hpp" // IWYU pragma: keep
#include "robot/oekf_rerun.hpp" // IWYU pragma: keep
#include "robot/localization.hpp" // IWYU pragma: keep
#include "robot/start_pose.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep
#include "robot/jerryPaths.hpp" // IWYU pragma: keep

// EKF/MCL replay lives in src/oekf_rerun.cpp / src/mcl_rerun.cpp.
//
// MCL pose-correction for hand-coded autons: bracket your moves with
// loc::start(fieldX, fieldY, fieldDeg) ... loc::stop(). Coords MUST be
// field-absolute (0,0 = red bottom-left, 144"). See corrected_auton().

void angular_tuning() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 1000, {}, false);
}

void lateral_tuning() {
    chassis.setPose(0, -7, 0);
    chassis.moveToPoint(0, 23.8125, 1500, {}, false);
}

void pure_pursuit() {
    chassis.setPose(-66.369,-35.525,72.291);
    //chassis.follow(path1_txt, 12, 1000);
    //chassis.follow(path2_txt, 12, 1000);
    //chassis.follow(path3_txt, 12, 1000);
    chassis.follow(armanRoute_txt, 12, 1000);
}

void left_auton() {
    chassis.setPose(0,0,0);
    wing.set_value(0);
    limiter.set_value(1);
    //Picking up 3 blocks
    chassis.moveToPose(0, 13, 10, 1000, {.forwards = true, .lead = 0, .minSpeed=30}, false);
    chassis.moveToPose(0, 17, 10, 1000, {.forwards = true, .lead = 0, .minSpeed=50}, false);

    //pros::delay(100);
    chassis.moveToPose(0, 19, -3, 1000, {.forwards = true, .lead = 0, .maxSpeed=30}, false);
    //pros::delay(100);
    chassis.moveToPose(0,27,10,1000,{.forwards=true, .lead=0, .maxSpeed=35},false);
    //pros::delay(100);
    chassis.turnToHeading(-109,500);
    //middle goal scoring
    chassis.moveToPoint(20,37,2000,{.forwards=false,.maxSpeed=65},false);
    chassis.turnToHeading(-113,200);
    limiter.set_value(1);
    // FIRST TRILAT
    //matchloading
    chassis.moveToPoint(-34,11,2500,{.forwards=true,.maxSpeed=80},false);
    chassis.turnToHeading(-156,500);
    //pros::delay(50);
    matchLoad.set_value(true);

    chassis.moveToPoint(-36,0,500,{.forwards=true},false);
    pros::delay(500);
    //long goal scoring
    chassis.turnToHeading(-160,500);
    chassis.moveToPoint(-26,35,1500,{.forwards=false},false);
    matchLoad.set_value(false);
    limiter.set_value(0);
    pros::delay(2500);

}

ASSET(jerry1_txt);

void left_auton_override() {
    chassis.setPose(0,0,0);
    chassis.follow(jerry1_txt, 15, 1000, {}, false); // 15 inch lookahead 
}


void right_auton() {
    chassis.setPose(0, 0, 0);
}

void park_auton() {
    chassis.setPose(0,0,0);
    wing.set_value(0);
    limiter.set_value(1);
    //Matchload No.1
    chassis.moveToPoint(-2,41,2000,{.forwards=true},false);
    chassis.turnToHeading(91,500,{});
    matchLoad.set_value(false);
    chassis.moveToPoint(14,0,2000,{.forwards=true},false);
    pros::delay(2000);
}


// ── LOCALIZATION TEST ─────────────────────────────────────────────────
// Place the robot near a corner (so two walls are within sensor range) at
// heading TDEG. Correction is OFF: the filter only estimates, so you can
// compare its output to where the robot actually is.
//
//   od  = wheel odometry (x y theta)
//   est = filter estimate (x y theta)
//   var = position variance (shrinks as it gets confident); n = N_eff (MCL)
//
// Tests:
//  1. Held still: est should sit at the true coord, var should shrink.
//  2. Seed wrong on purpose: est should converge back using the walls.
//  3. Hand-push the robot: est should track the new position.
//  4. Swap M to loc::Method::EKF and compare.
void localization_test() {
    // TDEG is the cardinal heading the robot was placed at. The sensors infer
    // the field-absolute start pose; loc::start seeds both odom and filter
    // there, so od and est begin at the same coordinate.
    const double TDEG = 0;
    const loc::Method M = loc::Method::MCL;       // swap to ::EKF to test EKF

    lemlib::Pose sp = determine_start_pose(TDEG); // field pose from the walls
    controller.print(0, 0, "start %4d %4d", (int)sp.x, (int)sp.y);
    pros::delay(600);
    loc::start(sp.x, sp.y, sp.theta, M, /*correct=*/false);

    while (true) {
        lemlib::Pose p = chassis.getPose();
        loc::Estimate e = loc::estimate();
        controller.print(0, 0, "od  %4d %4d %4d", (int)p.x, (int)p.y, (int)p.theta);
        pros::delay(60);
        controller.print(1, 0, "est %4d %4d %4d", (int)e.x, (int)e.y, (int)e.theta_deg);
        pros::delay(60);
        controller.print(2, 0, "var %5.1f n%3d ", e.var_xy, (int)e.extra);
        pros::delay(400);
    }
}


// ── EXAMPLE: hand-coded auton with MCL correction running ─────────────
// Field-absolute coords (0,0 = red-side bottom-left, 144"). Landmarks:
// center goal ~70,70; long goals at y≈23 and y≈117.
void corrected_auton() {

}


void skills_auton() {
    
}
