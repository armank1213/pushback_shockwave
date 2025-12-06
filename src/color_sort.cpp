#include "robot/color_sort.hpp"
#include "robot/motors.hpp"
#include "robot/hardware.hpp"
#include <cstdlib>

void red_colorSort(int sortMode, int distance, double hue) {
    if (distance < 95) { // if an object is detected within 95 mm
        if (sortMode == 0) { 
            if ((hue >= 0 && hue <= 30)) {
                sort(127);
            } else if (hue >= 220 && hue <= 360) {
                sort(-127);
            }
        }
    }
    else {
        sort(0);
    }
}

void blue_colorSort(int sortMode, int distance, double hue) {
    if (distance < 95) { // if an object is detected within 95 mm
        if (sortMode == 0) { 
            if ((hue >= 220 && hue <= 360)) {
                sort(127);
            } else if (hue >= 0 && hue <= 30) {
                sort(-127);
            }
        }
    }
    else {
        sort(0);
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

