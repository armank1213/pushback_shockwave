#include "robot/ui.hpp"
#include "robot/chassis_config.hpp"
#include "pros/rtos.hpp"

// UI state variables
int autonSelection = 0;
int colorSortMode = 0; // 0 for red, 1 for blue

// UI label objects
lv_obj_t *poseXYlabel = NULL; // x and y coords
lv_obj_t *poseTlabel = NULL; // theta (direction robot is facing)

// Button event handlers

void left_ButtonEvent(lv_event_t *e) {
    autonSelection = 0;
}

void right_ButtonEvent(lv_event_t *e) {
    autonSelection = 1;
}

void red_colorSort(lv_event_t *e) {
    colorSortMode = 0;
}

void blue_colorSort(lv_event_t *e) {
    colorSortMode = 1;
}

// Button creation functions

void left_Button(void) {
    lv_obj_t *button3 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(button3, left_ButtonEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *button3_label = lv_label_create(button3);
    lv_label_set_text(button3_label, "Left Auton");
    lv_obj_align(button3, LV_ALIGN_TOP_LEFT, 0,30);
}

void right_Button(void) {
    lv_obj_t *button4 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(button4, right_ButtonEvent, LV_EVENT_ALL, NULL);
    lv_obj_t *button4_label = lv_label_create(button4);
    lv_label_set_text(button4_label, "Right Auton");
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

// Label creation functions
void poseXY_Label(void) {
    poseXYlabel = lv_label_create(lv_screen_active());
    lv_obj_align(poseXYlabel,LV_ALIGN_TOP_MID,0,30);
}

void poseT_Label(void) {
    poseTlabel = lv_label_create(lv_screen_active());
    lv_obj_align(poseTlabel,LV_ALIGN_TOP_MID,0,60);
}

// Label update task
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

// Initialize UI
void initializeUI() {
    lv_obj_t *brain_screen = lv_obj_create(NULL);
    lv_screen_load(brain_screen);

    poseXY_Label();
    poseT_Label();
    left_Button();
    right_Button();
    bluecolorSortButton();
    redcolorSortButton();

    // Create a pros task that updates the vertical and horizontal rotation sensor measurements on the brain screen
    pros::Task VHP_TASK(update_VHP_Labels, nullptr, (uint32_t)TASK_PRIORITY_DEFAULT, (uint16_t)TASK_STACK_DEPTH_DEFAULT);
}

