#pragma once 

// Basic motor movement functions
void intake(int intakePower);
void outtake(int outtakePower);

// Control functions
void intakeControl();
void outtakeControl();

// antiJam control function
void antiJamControl(bool antiJamButtonPressed, bool isOurBlock);
