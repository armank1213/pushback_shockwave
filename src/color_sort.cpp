#include "robot/color_sort.hpp"
#include "robot/motors.hpp"
#include "robot/hardware.hpp"
#include <cstdlib>

void red_colorSort(int distance, double hue) {
    if (distance < 95) { // if an object is detected within 95 mm
        if ((hue >= 0 && hue <= 30)) {
            //sort(127);
        } else if (hue >= 220 && hue <= 360) {
            //sort(127); // supposed to be -127
        }
    }
    else {
        //sort(0);
    }
}

void blue_colorSort(int distance, double hue) {
    if (distance < 95) { // if an object is detected within 95 mm
        if ((hue >= 220 && hue <= 360)) {
            //sort(127);
        } else if (hue >= 0 && hue <= 30) {
            //sort(127); // supposed to be -127
        }
    }
    else {
        //sort(0);
    }
}

void skills_colorSort(int distance, double hue) {
    if (distance < 95) { // if an object is detected within 95 mm
        if ((hue >= 220 && hue <= 360)) {
            //sort(127);
        } else if (hue >= 0 && hue <= 30) {
            //sort(127);
        }
    }
    else {
        //sort(0);
    }
}

void manual_colorSort() { // soon to be implemented
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        //sort(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        //sort(-127);
    }
}