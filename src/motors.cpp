#include "robot/motors.hpp"
#include "robot/hardware.hpp"

// Drivetrain-only robot: no intake/middle/outtake motors. The limiter piston
// keeps its original button mapping.
void control() {
   if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
       limiter.set_value(1);
   }
   else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)
            || controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)
            || controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
       limiter.set_value(0);
   }
}
