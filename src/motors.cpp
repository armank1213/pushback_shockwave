#include "robot/motors.hpp"
#include "robot/hardware.hpp"
#include "robot/ui.hpp"
#include <cstdlib>


// Basic motor control functions
void intake(int intakePower) {
    intakeMotor.move(intakePower);
    middleMotor.move(intakePower);
}

void outtake(int outtakePower) {
    outtakeMotor.move(-outtakePower);
}

// Control functions
void intakeControl() {
    if (driver == 0) { // chey binds
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake(-127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake(127);
        }
        else {
            intake(0);
        }
    }
}


void outtakeControl() {
    if (driver == 0) { // chey binds
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            outtake(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
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
            middleMotor.move(127 * direction);
            outtakeMotor.move(-127 * direction);
        } else {
            intakeMotor.move(0);
            middleMotor.move(0);
            outtakeMotor.move(0);
        }
    } else if (antiJamActive) {
        intakeMotor.move(0);
        middleMotor.move(0);
        outtakeMotor.move(0);
        antiJamActive = false;
    }
}
