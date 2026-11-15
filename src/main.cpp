#include "main.h"         // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot/autonomous.hpp"
#include "robot/chassis_config.hpp"
#include "robot/distance_reset.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp"
#include "robot/jerryPaths.hpp" // IWYU pragma: keep
#include "robot/localization.hpp"
#include "robot/motors.hpp"
#include "robot/pneumatics.hpp"
#include "robot/re_run_helpers.hpp"
#include "robot/start_pose.hpp"
#include "robot/ui.hpp" // IWYU pragma: keep

#include <cstdio>
#include <fstream>
#include <iostream>

std::ofstream ofs;
bool open = false;

// Route-recording start pose: field-absolute inches, (0,0) = red-side
// bottom-left corner, heading 0 = facing +Y. Place the robot's tracking
// center on this exact spot before pressing RIGHT.
constexpr double REC_START_X = 50.0;
constexpr double REC_START_Y = 0.0;
constexpr double REC_START_DEG = 0.0;

void initialize() {
  chassis.calibrate();

  vertical_rotation.reset();
  horizontal_rotation.reset();

  while (imu.is_calibrating()) {
    pros::delay(10);
  }

  clawLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  clawSpinMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  initializeUI();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  // oekf_rerun();        // EKF replay of a recorded/drawn route
  //mcl_rerun();           // MCL replay of a recorded/drawn route
  // corrected_auton();   // hand-coded auton + background MCL correction
  // localization_test(); // live readout to test EKF/MCL (correction off)
  // skills_auton();
  // angular_tuning();/
  // lateral_tuning();
  //left_auton_override();
  // pure_pursuit();
  ourToggle_auton();
  // oppositieTeamToggle_auton();
}

void opcontrol() {
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  verticalLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  clawLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  cascadeLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cascadeRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  open = false; // recording file opens on RIGHT press, not here

  // Bench localization test: UP toggles MCL (correction OFF) and shows odom
  // vs estimate on the controller while you drive.
  bool locTestOn = false;
  int dbgTick = 0;

  while (true) {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    chassis.arcade(leftY, rightX, false, .7);
    control();
    //matchloadToggle();
    //wingToggle();
    clawToggle();

    /*
    // UP = toggle the localization bench test (disabled to free UP for claw
    intake motor) if
    (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) { if
    (!locTestOn) { lemlib::Pose sp = determine_start_pose(0.0);   // infer field
    pose from walls loc::start(sp.x, sp.y, sp.theta, loc::Method::MCL,
    /*correct=* /false); locTestOn = true; controller.rumble("-"); } else {
            loc::stop();
            locTestOn = false;
        }
    }
    */

    // Cycle the 3 readout lines to stay under the controller's print rate
    // limit (~50ms/line): od = odom, est = filter estimate, var = variance.
    if (locTestOn) {
      loc::Estimate e = loc::estimate();
      lemlib::Pose p = chassis.getPose();
      int phase = dbgTick % 15;
      if (phase == 0)
        controller.print(0, 0, "od  %4d %4d %4d", (int)p.x, (int)p.y,
                         (int)p.theta);
      if (phase == 5)
        controller.print(1, 0, "est %4d %4d %4d", (int)e.x, (int)e.y,
                         (int)e.theta_deg);
      if (phase == 10)
        controller.print(2, 0, "var %5.1f n%3d ", e.var_xy, (int)e.extra);
    }
    dbgTick++;

    // RIGHT starts recording: set the pose to the fixed start spot
    // (REC_START_*), then open the log. Ignored once recording, so a second
    // RIGHT press doesn't restart and wipe the log.
    if (!open && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      if (pros::usd::is_installed()) {
        chassis.setPose(REC_START_X, REC_START_Y, REC_START_DEG);
        controller.print(0, 0, "Start %.0f %.0f", REC_START_X, REC_START_Y);

        if (ofs.is_open())
          ofs.close(); // close any previous session
        ofs.open("/usd/dtData.txt", std::ofstream::out | std::ofstream::trunc);
        if (ofs.is_open()) {
          open = true;
          controller.rumble("."); // confirm to driver
        }
      }
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      cascadeRight.move_velocity(200);
      cascadeLeft.move_velocity(200);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      cascadeRight.move_velocity(-200);
      cascadeLeft.move_velocity(-200);

    } else {
      cascadeRight.move_velocity(0);
      cascadeLeft.move_velocity(0);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {

      intakeMotor.move_velocity(200);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {

      intakeMotor.move_velocity(-200);

    } else {

      intakeMotor.move_velocity(0);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      clawLiftMotor.move_velocity(200);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      clawLiftMotor.move_velocity(-200);

    } else {
      clawLiftMotor.move_velocity(0);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      clawSpinMotor.move_velocity(200);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      clawSpinMotor.move_velocity(-200);

    } else {
      clawSpinMotor.move_velocity(0);
    }
    pose_recording(ofs, open);

    pros::delay(20);
  }

  if (ofs.is_open())
    ofs.close();
}
