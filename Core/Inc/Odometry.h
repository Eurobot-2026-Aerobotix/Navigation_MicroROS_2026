/*
 * odometry.h
 *
 *  Created on: Aug 27, 2025
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include <math.h>
#include "motors.h"

#define SQR(x) ((x) * (x))
#define SGN(x) ( ( (x)<0 ) ? -1 : 1)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

extern float driven_wheels_spacing;
extern float dead_wheels_spacing;
extern float current_x;
extern float current_y;
extern float current_phi; // angle between x axis and the robot's center axis, in radians
extern float absolute_phi; // phi without modulo 2PI

void setup_wheels_spacing(float driven__wheels_spacing, float dead__wheels_spacing);
void update_position();
void reset_position();
void set_position(float current__x, float current__y, float current__phi, float absolute__phi);
float get_distance_to(float x, float y);
float get_angle_to(float x, float y);
float ticks_to_distance(int32_t ticks, float wheel_diameter);
float distance_to_ticks(int32_t distance, float wheel_diameter);
float rad_to_deg(float radian);
float deg_to_rad(float degrees);


#endif /* INC_ODOMETRY_H_ */
