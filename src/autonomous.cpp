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
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 20, 1000, {}, false);
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

void left_auton_override() {
    chassis.setPose(-65.219, -2.591, 90);

    // Start → P1: front faces goal 2 (north side)
    chassis.moveToPose(-46.904, -19.68, 180, 2000, {.lead = 0.3, .maxSpeed = 50});
    chassis.waitUntilDone();
    // ...outtake into goal 2...

    // Back off, aim the back at P2, reverse in
    chassis.moveToPoint(-46.904, -9.5, 800, {.forwards = false});
    chassis.turnToHeading(290, 900);
    chassis.moveToPose(-26.312, -23.356, 270, 2000,
                       {.forwards = false, .lead = 0.4, .maxSpeed = 50});
    chassis.waitUntilDone();
    // ...intake at P2...

    // P2 → goal 2 east side
    chassis.moveToPoint(-41.17, -24.28, 1500, {.maxSpeed = 50});
    chassis.waitUntilDone();
    // ...outtake the P2 block into goal 2...

    // Goal 2 → loader, BACK FIRST
    chassis.moveToPoint(-32, -24.28, 900, {.forwards = false});
    chassis.turnToHeading(0, 900);                       // back points south, down the lane
    chassis.moveToPoint(-37, -59.046, 2000, {.forwards = false, .maxSpeed = 60});
    chassis.turnToHeading(90, 900);                      // back faces the loader
    chassis.moveToPoint(-63.267, -59.046, 2000, {.forwards = false, .maxSpeed = 60});

    for (int i = 0; i < 2; i++) {
        chassis.waitUntilDone();
        // matchload

        // Loader → goal 2 south side (P5): drive FORWARD off the loader
        chassis.moveToPoint(-59.5, -59.046, 700);
        chassis.turnToHeading(0, 700);
        chassis.moveToPoint(-59.5, -38, 1200, {.minSpeed = 40, .earlyExitRange = 3});
        chassis.moveToPose(-47.159, -29.228, 0, 2000, {.lead = 0.3, .maxSpeed = 60});
        chassis.waitUntilDone();
        // ...outtake into goal 2...

        if (i == 2 - 1) break;

        // P5 → loader, back first
        chassis.moveToPose(-59.5, -44, 0, 1500,
                           {.forwards = false, .lead = 0.5, .minSpeed = 40, .earlyExitRange = 3});
        chassis.moveToPoint(-59.5, -59.046, 1200, {.forwards = false});
        chassis.turnToHeading(90, 700);
        chassis.moveToPoint(-63.267, -59.046, 800, {.forwards = false});
    }
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
    // Seed pose + spawn the MCL correction task.
    loc::start(56.5, 23, 0);

    wing.set_value(0);
    limiter.set_value(1);
    matchLoad.set_value(false);

    // Plain LemLib moves — MCL trims drift underneath.
    chassis.moveToPoint(70, 24, 2000, {.forwards = true}, false);   // to bottom long goal
    chassis.turnToHeading(0, 600, {}, false);
    chassis.moveToPose(70, 60, 0, 2500, {.forwards = true}, false); // up toward center goal
    pros::delay(1000);

    loc::stop();   // always stop the task before the function returns
}


void skills_auton() {
    // Field-absolute skills route with discrete snapPose corrections.
    loc::start(70, 24, 0, loc::Method::MCL, false);
    wing.set_value(0);
    limiter.set_value(1);
    matchLoad.set_value(false);
    // Blocking moves: each finishes before the next, so the robot is stopped
    // when snapPose() runs.
    chassis.moveToPose(70, 27, 0, 500, {.forwards=true,.lead=0}, false);
    loc::snapPose();
    chassis.moveToPose(12, 27, -90, 1700, {.forwards=true,.lead=0}, false);
    loc::snapPose();
    chassis.moveToPose(12,115,0, 2500, {.lead=0}, false);
    loc::snapPose();   // robot fully stopped -> clean reading, x/y only
    chassis.moveToPose(26,119,45,1000,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(27,119,0,1000,{.forwards=true,.lead=0},false);
    loc::snapPose();
    matchLoad.set_value(true);
    pros::delay(500);
    chassis.moveToPose(26,149,0,700,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(26,152,0,500,{.forwards=true,.lead=0},true);
    chassis.moveToPose(26,154,0,200,{.forwards=true,.lead=0},false);
    chassis.moveToPose(26,152,0,200,{.forwards=true,.lead=0},false);
    chassis.moveToPose(26,154,0,200,{.forwards=true,.lead=0},false);
    loc::snapPose();
    pros::delay(2100);
    chassis.moveToPose(27,110,0,1000,{.forwards=false,.lead=0},false);
    loc::snapPose();
    matchLoad.set_value(false);
    chassis.moveToPose(27,119,90,700,{.forwards=false,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(88,106.5,90,2000,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(88,106.5,55,800,{.forwards=true,.lead=0},false);//
    loc::snapPose();
    chassis.moveToPose(73,93,90,1200,{.forwards=false,.lead=0},false);
    loc::snapPose();
    limiter.set_value(0);
    pros::delay(300);
    pros::delay(2000);
    chassis.moveToPose(88,93,90,800,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(88,93,135,500,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(42.5,118,-86,1200,{.forwards=false,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(15.5,114,-90,2000,{.forwards=true,.lead=0},false);
    loc::snapPose();
    matchLoad.set_value(true);
    pros::delay(200);
    chassis.moveToPose(5,115,-90,500,{.forwards=true,.lead=0},true);
    chassis.moveToPose(2,115,-90,500,{.forwards=true,.lead=0},false);
    chassis.moveToPose(5,115,-90,500,{.forwards=true,.lead=0},false);
    chassis.moveToPose(2,115,-90,500,{.forwards=true,.lead=0},false);
    loc::snapPose();
    limiter.set_value(1);
    pros::delay(2100);
    chassis.moveToPose(15.5,114,-90,2000,{.forwards=true,.lead=0},false);
    matchLoad.set_value(false);

    chassis.moveToPose(88,106.5,-90,2000,{.forwards=false,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(88,106.5,55,800,{.forwards=true,.lead=0},false);//
    loc::snapPose();
    chassis.moveToPose(73,92,90,1200,{.forwards=false,.lead=0},false);
    loc::snapPose();
    limiter.set_value(0);
    pros::delay(300);
    pros::delay(2000);
    loc::snapPose();
    pros::delay(200);
    chassis.moveToPose(140,94,90,2000,{.forwards=true,.lead=0},true);
    loc::snapPose();
    chassis.moveToPose(140,94,180,500,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(140,14,180,2000,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(140,14,270,3000,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(70,10,270,1500,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(75,0,270,2000,{.forwards=true,.lead=0, .minSpeed=70},false);
    //chassis.moveToPose(120,30,180,100);
    /*
    chassis.moveToPose(118,20,180,2000,{.forwards=true,.lead=0},false);
    loc::snapPose();
    matchLoad.set_value(true);
    chassis.moveToPose(118,15,180,2000,{.forwards=true,.lead=0},false);
    loc::snapPose();
    pros::delay(1000);
    chassis.moveToPose(118,30,180,2000,{.forwards=false,.lead=0},false);
    chassis.moveToPose(118,30,90,2000,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(130,30,90,2000,{.forwards=true,.lead=0},false);
    loc::snapPose();
    chassis.moveToPose(130,30,0,2000,{.forwards=false,.lead=0},false);
    loc::snapPose();
*/




    // Drive into park (disabled):
    //chassis.moveToPoint(18,0,2000,{.forwards=true, .maxSpeed=110, .minSpeed=95},false);
    // Pick up blocks that don't clear in park (disabled):
    /*
    pros::delay(2000);
    */
    loc::stop();
}
