#include "robot/autonomous.hpp"
#include "robot/chassis_config.hpp" // IWYU pragma: keep
#include "robot/hardware.hpp" // IWYU pragma: keep
#include "robot/motors.hpp" // IWYU pragma: keep
#include "robot/auton_helpers.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include <cmath>


void correct_y(int target_y, int final_x, int final_t, int timeout, bool reversed, bool async) {
    long startTime = pros::millis();
    long elapsedTime = 0;

    int delta_y = chassis.getPose().y;
    while (elapsedTime < 1500) {
        if (reversed) {
            chassis.moveToPose(final_x, target_y-delta_y, final_t, timeout, {.forwards=true}, async);
        }
        else {
            chassis.moveToPose(final_x, target_y-delta_y, final_t, timeout, {}, async);
        }
        elapsedTime = pros::millis() - startTime;
    }   
}
void correct_x(int target_x, int final_y, int final_t, int timeout, bool reversed, bool async) {
    long startTime = pros::millis();
    long elapsedTime = 0;
    
    int delta_x = chassis.getPose().x;
    while (elapsedTime < 1500) {
        if (reversed) {
            chassis.moveToPose(target_x-delta_x, final_y, final_t, timeout, {.forwards=true}, async);
        }
        else {
            chassis.moveToPose(target_x-delta_x, final_y, final_t, timeout, {}, async);
        }
        elapsedTime = pros::millis() - startTime;
    }
}
void correct_t(int target_t, int final_x, int final_y, int timeout, bool reversed, bool async) {
    long startTime = pros::millis();
    long elapsedTime = 0;    
    
    int delta_t = chassis.getPose().theta;
    while (elapsedTime < 1000) {
        if (reversed) {
            chassis.moveToPose(final_x, final_y, target_t-delta_t, timeout, {.forwards=false}, async);
        }
        else {
            chassis.moveToPose(final_x, final_y, target_t-delta_t, timeout, {}, async);
        }
        elapsedTime = pros::millis() - startTime;
    }
}

// distance reset

using namespace std;
using namespace lemlib;

double angle_error(double angle_a, double angle_b) {

    double diff = fabs(angle_a - angle_b);

    double mod_diff = fmod(diff, 360.0);
    if (mod_diff < 0) {
        mod_diff += 360.0;
    }

    if (mod_diff > 180.0) {
        return 360.0 - mod_diff;
    } else {
        return abs(mod_diff);
    }
}

// Initializing 

Pose pose = Pose(-54.422, -9.185, 0);

double x = pose.x;
double y = pose.y;
double theta = pose.theta;

double prev_rotation = vertical_rotation.get_position();

const double FIELD_WIDTH = 3657.6; // mm
const double FIELD_HEIGHT = 3657.6; // mm

const double ANGLE_TOLERANCE = 5.0; // degrees

struct SensorOffsets {
    double left;
    double right;
    double back;
};


SensorOffsets sensor_offsets = {0.0, 0.0, 0.0};

double x_prev = x;
double y_prev = y;

const double WHEEL_CIRCUMFERENCE = Omniwheel::NEW_2 * M_PI; // mm

std::pair<double, double> reset_distance() {

    while (true) {

        double current_rotation = vertical_rotation.get_position();
        double delta_rotation = current_rotation - prev_rotation;
        double prev_rotation = current_rotation;

        
        double delta_degrees = delta_rotation / 100.0;
        double wheel_rotations = delta_degrees / 360.0;
        double delta_s = wheel_rotations * WHEEL_CIRCUMFERENCE;


        theta = imu.get_heading();

        x = x + delta_s * cos(theta);
        y = y + delta_s + sin(theta);

        //distance
        double DL = LeftDistanceSensor.get_distance();
        double DR = RightDistanceSensor.get_distance();
        double DB = BackDistanceSensor.get_distance();

        // left wall

        if (DL > 0 && fabs(angle_error(theta, 90.0) < ANGLE_TOLERANCE)) {
            x = DL - sensor_offsets.left;
        }
        if (DR > 0 && fabs(angle_error(theta, 270.0) < ANGLE_TOLERANCE)) {
            x = FIELD_WIDTH - DR + sensor_offsets.right;
        }
        if (DB > 0 && fabs(angle_error(theta, 180.0) < ANGLE_TOLERANCE)) {
            y = DB - sensor_offsets.back;
        }

        // in case of jumps or tipping (exponential smoothing)

        x = x_prev + 0.3 * (x - x_prev);
        y = y_prev + 0.3 * (y - y_prev);
        
        x_prev = x;
        y_prev = y;

        return {x,y};

        pros::delay(20);
    }
}

