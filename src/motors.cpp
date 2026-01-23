#include "robot/motors.hpp"
#include "robot/hardware.hpp"
#include "robot/ui.hpp"
#include <cstdlib>


// Basic motor control functions
void intake(int intakePower) {
    intakeMotor.move(intakePower);
}

void outtake(int outtakePower) {
    outtakeMotor.move(-outtakePower);
    middletakeMotor.move(outtakePower);
}

void middletake(int middlePower) {
    middletakeMotor.move(middlePower);
    outtakeMotor.move(-middlePower);
}

void sort(int sortPower) {
    sortMotor.move(sortPower);
}


// Control functions
void intakeControl() {
    if (driver == 0) { // chey binds
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake(-127);
        }
        else {
            intake(0);
        }
    }
    else if (driver == 1) { // naman binds
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // change to naman binds
            intake(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // change to naman binds
            intake(-127);
        }
        else {
            intake(0);
        }
    }
    else if (driver == 2) { // ani binds
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // change to ani binds
            intake(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // change to ani binds
            intake(-127);
        }
        else {
            intake(0);
        }
    }
}


void outtakeControl() {
    if (driver == 0) { // chey binds
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            outtake(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            outtake(-127);
        }
        else {
            outtake(0);
        }
    }
    else if (driver == 1) { // naman binds
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // change to naman binds
            outtake(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // change to naman binds
            outtake(-127);
        }
        else {
            outtake(0);
        }
    }
    else if (driver == 2) { // ani binds
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // change to ani binds
            outtake(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // change to ani binds
            outtake(-127);
        }
        else {
            outtake(0);
        }
    }
}


// Will be implemented in the future for s bot
void antiJamControl(bool antiJamButtonPressed, bool isOurBlock) {
    static int lastToggleTime = 0;
    static int direction = 1;
    static bool antiJamActive = false;

    if (antiJamButtonPressed) {
        int currentTime = pros::millis();

        if (!isOurBlock) { 
            antiJamActive = true;

            if (currentTime - lastToggleTime > 500) {
                direction = (rand() % 2 == 0) ? 1 : -1;
                lastToggleTime = currentTime;
            }

            intakeMotor.move(127 * direction);
            sortMotor.move(127 * direction);
            middletakeMotor.move(-127 * direction);
            outtakeMotor.move(-127 * direction);
        } else {
            intakeMotor.move(0);
            sortMotor.move(0);
            middletakeMotor.move(0);
            outtakeMotor.move(0);
        }
    } else if (antiJamActive) {
        intakeMotor.move(0);
        sortMotor.move(0);
        middletakeMotor.move(0);
        outtakeMotor.move(0);
        antiJamActive = false;
    }
}
