#include "robot/opcontrol.hpp"
#include "robot/hardware.hpp"
#include "robot/chassis_config.hpp"
#include "robot/motors.hpp"
#include "robot/color_sort.hpp"
#include "robot/ui.hpp"
#include "liblvgl/lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

void opcontrol(void) {
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Matchload piston variables
    bool matchloadPistonToggle = false;
    static bool lastAButtonState = false;

    // Limiter piston variables
    bool limiterPistonToggle = false;
    static bool lastYButtonState = false;

    // Wing piston variables
    bool wingPistonToggle = false;
    static bool lastXButtonState = false;

    // Color sorting variables
    static int sortMode = 0;
    static bool LastB_ButtonState = false;

    // Anti-jam control variables
    const bool allianceColor = true; // true for red, false for blue
    bool isOurBlock = false;

    while (true) {
        // Get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Chassis Drive Functions
        chassis.arcade(leftY, rightX, false, .3);

        // Intake and outtake control functions
        intakeControl();
        outtakeControl();
        
        // Set light to 100% and get distance and color readings
        colorSensor.set_led_pwm(100);
        int distance = distanceSensor.get_distance();
        double hue = colorSensor.get_hue();

        // Logic for sorting mode toggle
        bool CurrentB_ButtonState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (CurrentB_ButtonState && !LastB_ButtonState) {
            sortMode = 1 - sortMode;
        }
        LastB_ButtonState = CurrentB_ButtonState;

        // Color sorting functions
        if (colorSortMode == 0) {
            red_colorSort(sortMode, distance, hue);
        }
        else if (colorSortMode == 1) {
            blue_colorSort(sortMode, distance, hue);
        }

        // Matchload Pneumatics Toggle
        bool currentA = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (currentA && !lastAButtonState) {
            matchloadPistonToggle = !matchloadPistonToggle;
            matchLoad.set_value(matchloadPistonToggle);
        }
        lastAButtonState = currentA;

        // Limiter Pneumatics Toggle
        bool currentY = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

        if (currentY && !lastYButtonState) {
            limiterPistonToggle = !limiterPistonToggle;
            limiter.set_value(limiterPistonToggle);
        }
        lastYButtonState = currentY;

        // Wing Mech Pneumatics Toggle
        bool currentX = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        
        if (currentX && !lastXButtonState) {
            wingPistonToggle = !wingPistonToggle;
            wing.set_value(wingPistonToggle);
        }
        lastXButtonState = currentX;

        // Logic for anti-jam control
        if (distance < 135) {
            if (allianceColor && ((hue <= 360 && hue >= 300) || (hue >= 0 && hue <= 35))) {
                isOurBlock = true;
            } else if (!allianceColor && (hue <= 250 && hue >= 180)) {
                isOurBlock = true;
            }
        }
        bool antiJamButtonPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
        antiJamControl(antiJamButtonPressed, isOurBlock);

        // Delay to save resources
        lv_timer_handler();
        pros::delay(30);
    }
}

#ifdef __cplusplus
}
#endif

