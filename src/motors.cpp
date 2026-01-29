#include "robot/motors.hpp"
#include "robot/hardware.hpp"

// Basic motor control functions
void intake(int intakePower) {
    intakeMotor.move(intakePower);
}

void outtake(int outtakePower) {
    outtakeMotor.move(-outtakePower);
    middletakeMotor.move(outtakePower);
}

void middleTake(int middlePower) {
    middletakeMotor.move(middlePower);
    outtakeMotor.move(-middlePower);
}

void sort(int sortPower) {
    sortMotor.move(sortPower);
}

void fullMotorControl() {
    fullMotor.move(-127);
}

void halfMotorControl() {
    halfMotor.move(-127);
}

// Control functions
void intakeControl() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // R2 for chey, L2 for naman
        intake(-127);
    }
    else {
        intake(30);
    }
}

void outtakeControl() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        outtake(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // L2 for chey, R2 for namna
        outtake(-127);
    }
    else {
        outtake(30);
    }
}

void manual_colorSort() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        sort(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        sort(-127);
    }
}

