/*
 * navigation.h
 *
 *  Created on: Aug 27, 2025
 *      Author: monji
 */

#ifndef INC_NAVIGATION_H_
#define INC_NAVIGATION_H_

#include "Odometry.h"

enum Robot_state{
	DEAD_STOP,
	ACCELERATING,
	CONTROLLED_BREAK,
	EMERGENCY_BREAK,
	CRUISING // holding the same speed
}; // other states will be added, like doing a curve

extern enum Robot_state robot_state;
extern float ramping_acceleration; // !!! these accelerations marks the rate of change in velocity with respect to the distance traveled, not time!!!
extern float breaking_acceleration;
extern float emergency_breaking_acceleration;
extern float rotating_speed;
extern int8_t direction_R; // 1 if going forward, -1 if backwards
extern int8_t direction_L;
extern float cruise_speed_R; // the speed which the right wheel is accelerating towards
extern float cruise_speed_L;
extern float starting_phi;

int move_forward(float distance, float speed); // will return 0 if target has been reached successfully or 1 if it could not be reached
int move_backward(float distance, float speed); // will return 0 if target has been reached successfully or 1 if it could not be reached
float calculate_instant_target_speed_R(float x, float distance);
float calculate_instant_target_speed_L(float x, float distance);
float calculate_instant_target_phi(float x, float distance);
int rotate(float angle, int8_t is_radian);// is_radian should be set to 1 if the angle is expressed in radians and to 0 if in degrees
int orientate(float orientation, int8_t is_radian);// will return 0 if target has been reached successfully or 1 if it could not be reached
int go_to(float x, float y, float speed);// will return 0 if target has been reached successfully or 1 if some error happened in rotation, 2 if some error happened in moving forward
void emergency_break();

#endif /* INC_NAVIGATION_H_ */
