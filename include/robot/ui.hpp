#pragma once

#include "liblvgl/lvgl.h" // IWYU pragma: keep
#include "robot/chassis_config.hpp" // IWYU pragma: keep

// UI state variables
extern int autonSelection;
extern int colorSortMode;
extern int driver;

// UI label objects
extern lv_obj_t *poseXYlabel;
extern lv_obj_t *poseTlabel;

// Button event handlers
void blueLeft_ButtonEvent(lv_event_t *e);
void blueRight_ButtonEvent(lv_event_t *e);
void redLeft_ButtonEvent(lv_event_t *e);
void redRight_ButtonEvent(lv_event_t *e);
void red_colorSort(lv_event_t *e);
void blue_colorSort(lv_event_t *e);

// Button creation functions
void blueLeft_Button(void);
void blueRight_Button(void);
void redLeft_Button(void);
void redRight_Button(void);
void redcolorSortButton(void);
void bluecolorSortButton(void);

// Label creation functions
void poseXY_Label(void);
void poseT_Label(void);

// Label update task
void update_VHP_Labels(void* param);

// Initialize UI
void initializeUI();

