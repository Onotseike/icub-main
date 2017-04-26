#include <RobotDriver.h>

const char* R1DriverReal::R1PartName[NUM_R1_PARTS] = { "torso", "torso_tripod", "head", "left_arm", "left_wrist_tripod", "right_arm", "right_wrist_tripod", "left_hand", "right_hand" };

const char* R1DriverSim::R1PartName[NUM_R1_PARTS] = { "torso", "head", "left_upper_arm_and_pronosup", "left_wrist", "right_upper_arm_and_pronosup", "right_wrist", "left_hand", "right_hand" };

const char* iCubDriver::iCubPartName[NUM_ICUB_PARTS] = { "torso", "head", "left_arm", "right_arm", "left_leg", "right_leg" };