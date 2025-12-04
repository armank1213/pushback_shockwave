#include "main.h"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include <cmath>
#include "liblvgl/lvgl.h" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// lvgl
lv_obj_t *poseXYlabel = NULL; // x and y coords
lv_obj_t *poseTlabel = NULL; // theta (direction robot is facing)
static int autonSelection = 0;
static int colorSortMode = 0; // 0 for red, 1 for blue


// motor groups
/*
    Motor Layout:
        Left Side Motors:
            - Port 6 = Front Motor
            - Port 5 = Reversed Motor (Top Stack Motor)
            - Port 4 = Bottom Stack Motor
        Right Side Motors:
            - Port 11 = Reversed Motor (Top Stack Motor)
            - Port 12 = Bottom Stack Motor
            - Port 7 = Front Motor
*/
pros::MotorGroup leftMotors({-6, 5, -4}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({-11, 12, 7}, pros::MotorGearset::green);

// Inertial Sensor on port 16
pros::Imu imu(16);

// Motors
pros::Motor outtakeMotor(15, pros::v5::MotorGears::green); // outtake motor on port 13 ORIGINALLY now on 15 for testing
pros::Motor intakeMotor(2, pros::v5::MotorGears::green); // intake motor on port 2

pros::Motor sortMotor(1, pros::v5::MotorGears::green); // sorting motor on port 1
pros::Motor middletakeMotor(9, pros::v5::MotorGears::green); // middletake motor on port 9

/*
// long goal outtake motor group
pros::MotorGroup longTake({-13, -2}); 

// middle goal outtake motor group
pros::MotorGroup middleTake({20, 9});
*/

// vision sensor signature IDs
// pros::Vision visionSensor(8);
// pros::vision_signature_s_t BLUE_SIG  = pros::Vision::signature_from_utility(1, -4089, -2329, -3210, 2711, 4961, 3836, 2.100, 0);
// pros::vision_signature_s_t RED_SIG  = pros::Vision::signature_from_utility(2, 4861, 11873, 8368, -1889, -225, -1058, 1.300, 0);

// Optical Sensor
pros::Optical colorSensor(19);

// Distance Sensor 
pros::Distance distanceSensor(14);

// Matchload Pneumatics
pros::adi::Pneumatics matchLoad('A', false);
// Limiter Pneumatics
pros::adi::Pneumatics limiter('G', false);

pros::adi::Pneumatics wing('C', false);

// tracking wheels
// 11 inch long
// 13.5 inch wide
// Tracking center (6.875, 5.5)
// horizontal tracking wheel encoder. Rotation sensor, port 10, not reversed
pros::Rotation horizontal_rotation(10);
// vertical tracking wheel encoder. Rotation sensor, port 18, reversed
pros::Rotation vertical_rotation(18);
// horizontal tracking wheel. 2" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal_wheel(&horizontal_rotation, lemlib::Omniwheel::NEW_2, -5.5);
// vertical tracking wheel. 2" diameter, 0.37" offset, left of the robot (negative)
lemlib::TrackingWheel vertical_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_2, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 12.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral PID controller
lemlib::ControllerSettings lateralController(13, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              9, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
// Tracking center: (4,6.3125)
// Vertical Tracking Wheel Offset: 6.3125-6.3125 = 0.000
// Horizontal Tracking Wheel Offset: 4-(-1.5) = 5.5 (this value 5.5 should be negative since its at the back of the robot)


lemlib::OdomSensors sensors(&vertical_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors, &throttleCurve, &steerCurve);



// lvgl display (auton selector)
void blueLeft_ButtonEvent(lv_event_t *e) {
    autonSelection = 1;
}

void blueRight_ButtonEvent(lv_event_t *e) {
    autonSelection = 2;
}

void redLeft_ButtonEvent(lv_event_t *e) {
    autonSelection = 3;
}

void redRight_ButtonEvent(lv_event_t *e) {
    autonSelection = 4;
}
void red_colorSort(lv_event_t *e) {
    colorSortMode = 0;
}
void blue_colorSort(lv_event_t *e) {
    colorSortMode = 1;
}

void blueLeft_Button(void) {
    lv_obj_t *button1 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(button1, blueLeft_ButtonEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *button1_label = lv_label_create(button1);
    lv_label_set_text(button1_label, "Blue Left Auton");
    lv_obj_align(button1, LV_ALIGN_TOP_LEFT,0,80);
}

void blueRight_Button(void) {
    lv_obj_t *button2 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(button2, blueRight_ButtonEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *button2_label = lv_label_create(button2);
    lv_label_set_text(button2_label, "Blue Right Auton");
    lv_obj_align(button2, LV_ALIGN_TOP_RIGHT,0,30);
}
void redLeft_Button(void) {
    lv_obj_t *button3 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(button3, redLeft_ButtonEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *button3_label = lv_label_create(button3);
    lv_label_set_text(button3_label, "Red Left Auton");
    lv_obj_align(button3, LV_ALIGN_TOP_LEFT, 0,30);
}
void redRight_Button(void) {
    lv_obj_t *button4 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(button4, redRight_ButtonEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *button4_label = lv_label_create(button4);
    lv_label_set_text(button4_label, "Red Right Auton");
    lv_obj_align(button4, LV_ALIGN_TOP_RIGHT,0,80);
}
void redcolorSortButton(void) {
    lv_obj_t *red_colorSortButton = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(red_colorSortButton, red_colorSort, LV_EVENT_ALL, NULL);
    lv_obj_t *redcolorSort_label = lv_label_create(red_colorSortButton);
    lv_label_set_text(redcolorSort_label, "Red Color Sort");
    lv_obj_align(red_colorSortButton, LV_ALIGN_BOTTOM_MID,0,-60);   
}
void bluecolorSortButton(void) {
    lv_obj_t *blue_colorSortButton = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(blue_colorSortButton, blue_colorSort, LV_EVENT_ALL, NULL);
    lv_obj_t *bluecolorSort_label = lv_label_create(blue_colorSortButton);
    lv_label_set_text(bluecolorSort_label, "Blue Color Sort");
    lv_obj_align(blue_colorSortButton, LV_ALIGN_BOTTOM_MID,0,-10);
}

void poseXY_Label(void) {
    poseXYlabel = lv_label_create(lv_screen_active());
    lv_obj_align(poseXYlabel,LV_ALIGN_TOP_MID,0,30);
}
void poseT_Label(void) {
    poseTlabel = lv_label_create(lv_screen_active());
    lv_obj_align(poseTlabel,LV_ALIGN_TOP_MID,0,60);
}
void update_VHP_Labels(void* param) {
    while (true) {
        if (poseXYlabel != NULL) {
            lv_label_set_text_fmt(poseXYlabel, "Pose: (%i, %i)", (int)chassis.getPose().x, (int)chassis.getPose().y);
        }
        if (poseTlabel != NULL) {
            lv_label_set_text_fmt(poseTlabel, "Theta: %i", (int)chassis.getPose().theta);
        }
        pros::delay(50);
    }
}
void blue_left_auton() {

}

void blue_right_auton() {

}

void red_left_auton() {

}

void red_right_auton() {
    //chassis.follow(red_right_txt, 3, 4000,true,false);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is reommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

    /*void blueLeft_Button();
    void blueRight_Button();
    void redLeft_Button();
    void redRight_Button();
    void red_colorSort();
    void blue_colorSort();
    void blueLeft_ButtonEvent();
    void blueRight_ButtonEvent();
    void redLeft_ButtonEvent();
    void blueRight_ButtonEvent();
    void redcolorSortButton();
    void bluecolorSortButton();
    void poseXY_Label();
    void poseT_Label();
    void update_VHP_Labels(void *param);*/

    pros::lcd::initialize();
    chassis.calibrate(); // calibrate drivetrain, odometry, PID, sensors, controller steering

    vertical_rotation.reset(); // reset vertical rotation sensor
    horizontal_rotation.reset(); // reset horizontal rotation sensor

    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    lv_obj_t *brain_screen = lv_obj_create(NULL);
    lv_screen_load(brain_screen);

    poseXY_Label();
    poseT_Label();
    blueLeft_Button();
    blueRight_Button();
    redLeft_Button();
    redRight_Button();
    bluecolorSortButton();
    redcolorSortButton();

    // create a pros task that updates the vertical and horizontal rotation sensor measurements on the brain screen
    pros::Task VHP_TASK(update_VHP_Labels,nullptr,(uint32_t)TASK_PRIORITY_DEFAULT,(uint16_t)TASK_STACK_DEPTH_DEFAULT);

    //chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers

    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            //pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            //pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            //pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            //pros::lcd::print(0, "Vertical Rotation Sensor: %i", vertical_rotation.get_position()); | commented out due to lvgl conflict
            //pros::lcd::print(1, "Horizontal Rotation Sensor: %i", horizontal_rotation.get_position()); | commented out due to lvgl conflict

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(red_right_txt); // '.' replaced with "_" to make c++ happy
ASSET(red_right_a_txt);


/**
 * Runs during autonomous
 */
void autonomous() {

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


    
    // auton code so far
    intakeMotor.move(127);
    sortMotor.move(127);
    outtakeMotor.move(-127);
    chassis.setPose(-51.804,-10.238,115);
    chassis.follow(red_right_txt,3,4000,true,false);
    // chassis.moveToPoint(-22.042,-22.601,7000);
    
    


    // Angular PID Tuning 
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(180, 100000);

    // Lateral PID Tuning 
    //chassis.setPose(0, 0, 0);
    //chassis.moveToPoint(0, 20, 999999);

    // auton selector
    /*switch (autonSelection) {
        case 1:
            blue_left_auton();
            break;
        case 2:
            blue_right_auton();
            break;
        case 3:
            red_left_auton();
            break;
        case 4:
            red_right_auton();
            break;
    }*/
}

/**
 * Runs during operator control
 */

void opcontrol() {

    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // prototypes
    void intakeControl();
    void outtakeControl();
    void manual_colorSort();
    void red_colorSort(int sortMode, int distance, double hue);
    void blue_colorSort(int sortMode, int distance, double hue);
    void antiJamControl(bool antiJamButtonPressed, bool isOurBlock);

    // matchloadPiston variables
    bool matchloadPistonToggle = false;
    static bool lastAButtonState = false;

    // limiterPiston variables
    bool limiterPistonToggle = false;
    static bool lastYButtonState = false;

    // wingPiston variables
    bool wingPistonToggle = false;
    static bool lastXButtonState = false;

    // color sorting variables
    static int sortMode = 0;
    static bool LastB_ButtonState = false;

    // anti-jam control variables
    const bool allianceColor = true; // true for red, false for blue
    bool isOurBlock = false;

    while (true) {

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Chassis Drive Functions
		// chassis.tank(leftY, rightX);
        // chassis.curvature(-rightX, -leftY, false);
        //chassis.arcade(-rightX, -leftY, false, .3);
        chassis.arcade(leftY, rightX, false, .3);

        // update vertical displacement and horizontal displacement labels on brain screen (measured by rotation sensors)
        //update_VHD_Labels();

        // intake and outtake control functions
        intakeControl();
        outtakeControl();
        
        // set light to 100% and get distance and color readings
        colorSensor.set_led_pwm(100);
        int distance = distanceSensor.get_distance();
        double hue = colorSensor.get_hue();

        // logic for sorting mode toggle ------------------------------------------------
        bool CurrentB_ButtonState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (CurrentB_ButtonState && !LastB_ButtonState) {
            sortMode = 1 - sortMode;
        }
        LastB_ButtonState = CurrentB_ButtonState;
        // -------------------------------------------------------------------------------


        // color sorting functions ------------
        if (colorSortMode == 0) {
            red_colorSort(sortMode, distance, hue);
        }
        else if (colorSortMode == 1) {
            blue_colorSort(sortMode, distance, hue);
        }
        // manual_colorSort();


        // Matchload Pneumatics Toggle -------------------
        bool currentA = controller.get_digital(DIGITAL_A);
        if (currentA && !lastAButtonState) {
            matchloadPistonToggle = !matchloadPistonToggle;
            matchLoad.set_value(matchloadPistonToggle);
        }
        lastAButtonState = currentA;
        // -----------------------------------------------

        // Limiter Pneumatics Toggle ---------------------
        bool currentY = controller.get_digital(DIGITAL_Y);

        if (currentY && !lastYButtonState) {
            limiterPistonToggle = !limiterPistonToggle;
            limiter.set_value(limiterPistonToggle);
        }
        lastYButtonState = currentY;
        // -----------------------------------------------

        // Wing Mech Pneumatics Toggle -------------------
        bool currentX = controller.get_digital(DIGITAL_X);
        
        if (currentX && !lastXButtonState) {
            wingPistonToggle = !wingPistonToggle;
            wing.set_value(wingPistonToggle);
        }
        lastXButtonState = currentX;
        // -----------------------------------------------


        // logic for anti-jam control ----------------------------------------------------
        if (distance < 135) {
            if (allianceColor && ((hue <= 360 && hue >= 300) || (hue >= 0 && hue <= 35))) {
                isOurBlock = true;
            } else if (!allianceColor && (hue <= 250 && hue >= 180)) {
                isOurBlock = true;
            }
        }
        bool antiJamButtonPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
        antiJamControl(antiJamButtonPressed, isOurBlock);
        // --------------------------------------------------------------------------------

        // delay to save resources
        lv_timer_handler();
        pros::delay(30);
    }
}


// functions

void intake(int intakePower) {
    intakeMotor.move(intakePower);
}
void outtake(int outtakePower) {
    outtakeMotor.move(-outtakePower);
    middletakeMotor.move(outtakePower);
}
void middleTake(int middlePower) {  // is there a point of this? double check
    middletakeMotor.move(middlePower);
    outtakeMotor.move(-middlePower);
}
void sort(int sortPower) {
    sortMotor.move(sortPower);
}
void intakeControl() {
    if (controller.get_digital(DIGITAL_R1)) {
        intake(127);
    }
    else if (controller.get_digital(DIGITAL_R2)) {
        intake(-127);
    }
    else {
        intake(0);
    }
}
void outtakeControl() {
    if (controller.get_digital(DIGITAL_L1)) {
        outtake(127);
    }
    else if (controller.get_digital(DIGITAL_L2)) {
        outtake(-127);
    }
    else {
        outtake(0);
    }
}
void manual_colorSort() {
    if (controller.get_digital(DIGITAL_UP)) {
        sort(127);
    }
    else if (controller.get_digital(DIGITAL_DOWN)) {
        sort(-127);
    }
}
 
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