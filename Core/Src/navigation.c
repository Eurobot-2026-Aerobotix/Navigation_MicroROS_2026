/*
 * navigation.c
 *
 *  Created on: Aug 27, 2025
 *      Author: monji
 */
#include "navigation.h"

enum Robot_state robot_state = DEAD_STOP;
float ramping_acceleration = 5;
float breaking_acceleration = -5;
float emergency_breaking_acceleration = -10;
float rotating_speed = 200;
int8_t direction_R;
int8_t direction_L;
float cruise_speed_R;
float cruise_speed_L;
float starting_phi;

int move_forward(float distance, float speed){
	direction_R = 1;
	direction_L = 1;

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float x1 = speed/ramping_acceleration;
	float x2 = speed/breaking_acceleration + distance;

	if (x2 <= x1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration*distance)/(breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed/ramping_acceleration;
		x2 = speed/breaking_acceleration + distance;
	}

	cruise_speed_R = speed;
	cruise_speed_L = speed;
	float target_point_x = current_x + distance*cos(current_phi);
	float target_point_y = current_y + distance*sin(current_phi);
	float distance_to_target = distance;
	float previous_distance_to_target = distance;
	float x = 0;

	robot_state = ACCELERATING;
	starting_phi = absolute_phi;
	float lane_correction_term;
	float phase_two_distance = 30;
	while (distance_to_target > phase_two_distance){
		x = distance - distance_to_target;
		if (robot_state == ACCELERATING && x > x1 ){
			robot_state = CRUISING;
		}else if (robot_state == CRUISING && x > x2){
			robot_state = CONTROLLED_BREAK;
		}else if (robot_state == EMERGENCY_BREAK){
			return 0;
		}

		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 10; // the coefficient is tunable
		if (x < 0.1*x1){
			lane_correction_term = 0;
		}
		set_target_speeds(calculate_instant_target_speed_R(x, distance)*(1 + lane_correction_term) ,calculate_instant_target_speed_L(x, distance)* (1 - lane_correction_term));
		HAL_Delay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = get_distance_to(target_point_x, target_point_y);

//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		uint8_t header1 = 0xAA;
		uint8_t header2 = 0xAB;
		uint8_t header3 = 0xAC;
		uint8_t header4 = 0xAD;
		float value1 = (float)current_speed_R;
		float value2 = (float)current_speed_R;
		float value3 = (float)target_speed_R;
		float value4 = (float)target_speed_L;
		uint8_t* bytes1 = (uint8_t*)&value1;
		uint8_t* bytes2 = (uint8_t*)&value2;
		uint8_t* bytes3 = (uint8_t*)&value3;
		uint8_t* bytes4 = (uint8_t*)&value4;
		HAL_UART_Transmit(&huart2, &header1, 1, 10);
		HAL_UART_Transmit(&huart2, bytes1, 4, 10);
		HAL_UART_Transmit(&huart2, &header2, 1, 10);
		HAL_UART_Transmit(&huart2, bytes2, 4, 10);
		HAL_UART_Transmit(&huart2, &header3, 1, 10);
		HAL_UART_Transmit(&huart2, bytes3, 4, 10);
		HAL_UART_Transmit(&huart2, &header4, 1, 10);
		HAL_UART_Transmit(&huart2, bytes4, 4, 10);
	}
	// completing the last part of distance separately, so the break happens on point
		// calculating the distance to move that results in the robot being the closest to target point, i can't explain the math in code unfortunately
	float x_closest = ( target_point_y - current_y + tan(current_phi)*current_x - tan(current_phi - PI/2)*target_point_x ) / ( tan(current_phi) - tan(current_phi - PI/2) );
	float y_closest = tan(current_phi)*(x_closest - current_x) + current_y;
	speed = (target_speed_R + target_speed_L)/ 2;
	if (fabs(speed) < phase_two_distance*(-breaking_acceleration) ){ // handling the case where the robot skips the first while loop, being too close to target_distance
		speed = - phase_two_distance * breaking_acceleration;
	}
	distance_to_target = get_distance_to(x_closest, y_closest);
	previous_distance_to_target = distance_to_target;
	while (distance_to_target >0.1 && distance_to_target <= previous_distance_to_target){
		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 110 ; // the coefficient is tunable
		set_target_speeds(speed*(1 + lane_correction_term) ,speed* (1 - lane_correction_term));
		speed = MAX(speed*0.95, 0);
		previous_distance_to_target = distance_to_target;
		HAL_Delay(2);
		distance_to_target = get_distance_to(x_closest, y_closest);

//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		uint8_t header1 = 0xAA;
		uint8_t header2 = 0xAB;
		uint8_t header3 = 0xAC;
		uint8_t header4 = 0xAD;
		float value1 = (float)current_speed_R;
		float value2 = (float)current_speed_R;
		float value3 = (float)target_speed_R;
		float value4 = (float)target_speed_L;
		uint8_t* bytes1 = (uint8_t*)&value1;
		uint8_t* bytes2 = (uint8_t*)&value2;
		uint8_t* bytes3 = (uint8_t*)&value3;
		uint8_t* bytes4 = (uint8_t*)&value4;
		HAL_UART_Transmit(&huart2, &header1, 1, 10);
		HAL_UART_Transmit(&huart2, bytes1, 4, 10);
		HAL_UART_Transmit(&huart2, &header2, 1, 10);
		HAL_UART_Transmit(&huart2, bytes2, 4, 10);
		HAL_UART_Transmit(&huart2, &header3, 1, 10);
		HAL_UART_Transmit(&huart2, bytes3, 4, 10);
		HAL_UART_Transmit(&huart2, &header4, 1, 10);
		HAL_UART_Transmit(&huart2, bytes4, 4, 10);
	}
	stop_motors();
	robot_state = DEAD_STOP;
	return 0;
}
int move_backward(float distance, float speed){
	speed = (speed < 0)? -speed: speed;
	distance = (distance < 0)? -distance: distance;
	direction_R = -1;
	direction_L = -1;

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float x1 = speed/ramping_acceleration;
	float x2 = speed/breaking_acceleration + distance;

	if (x2 <= x1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration*distance)/(breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed/ramping_acceleration;
		x2 = speed/breaking_acceleration + distance;
	}

	cruise_speed_R = -speed;
	cruise_speed_L = -speed;
	float target_point_x = current_x - distance*cos(current_phi);
	float target_point_y = current_y - distance*sin(current_phi);
	float distance_to_target = distance;
	float previous_distance_to_target = distance;
	float x = 0;

	robot_state = ACCELERATING;
	starting_phi = absolute_phi;
	float lane_correction_term;
	float phase_two_distance = 20;
	while (distance_to_target > phase_two_distance){
		x = distance - distance_to_target;
		if (robot_state == ACCELERATING && x > x1 ){
			robot_state = CRUISING;
		}else if (robot_state == CRUISING && x > x2){
			robot_state = CONTROLLED_BREAK;
		}else if (robot_state == EMERGENCY_BREAK){
			return 0;
		}

		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 10; // the coefficient is tunable
		if (x < 0.1*x1){
			lane_correction_term = 0;
		}
		set_target_speeds(calculate_instant_target_speed_R(x, distance)*(1 - lane_correction_term) ,calculate_instant_target_speed_L(x, distance)* (1 + lane_correction_term));
		HAL_Delay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = get_distance_to(target_point_x, target_point_y);
//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		uint8_t header1 = 0xAA;
		uint8_t header2 = 0xAB;
		uint8_t header3 = 0xAC;
		uint8_t header4 = 0xAD;
		float value1 = (float)current_speed_L;
		float value2 = (float)current_speed_R;
		float value3 = (float)target_speed_L;
		float value4 = (float)target_speed_R;
		uint8_t* bytes1 = (uint8_t*)&value1;
		uint8_t* bytes2 = (uint8_t*)&value2;
		uint8_t* bytes3 = (uint8_t*)&value3;
		uint8_t* bytes4 = (uint8_t*)&value4;
		HAL_UART_Transmit(&huart2, &header1, 1, 10);
		HAL_UART_Transmit(&huart2, bytes1, 4, 10);
		HAL_UART_Transmit(&huart2, &header2, 1, 10);
		HAL_UART_Transmit(&huart2, bytes2, 4, 10);
		HAL_UART_Transmit(&huart2, &header3, 1, 10);
		HAL_UART_Transmit(&huart2, bytes3, 4, 10);
		HAL_UART_Transmit(&huart2, &header4, 1, 10);
		HAL_UART_Transmit(&huart2, bytes4, 4, 10);
	}
	// completing the last part of distance separately, so the break happens on point
		// calculating the distance to move that results in the robot being the closest to target point, i can't explain the math in code unfortunately
	float x_closest = ( target_point_y - current_y + tan(current_phi)*current_x - tan(current_phi - PI/2)*target_point_x ) / ( tan(current_phi) - tan(current_phi - PI/2) );
	float y_closest = tan(current_phi)*(x_closest - current_x) + current_y;
	speed = (target_speed_R + target_speed_L)/ 2;
	if (fabs(speed) < phase_two_distance*(-breaking_acceleration) ){ // handling the case where the robot skips the first while loop, being too close to target_distance
		speed = - phase_two_distance * breaking_acceleration;
	}
	distance_to_target = get_distance_to(x_closest, y_closest);
	previous_distance_to_target = distance_to_target;
	while (distance_to_target >0.1 && distance_to_target <= previous_distance_to_target){
		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 20 ; // the coefficient is tunable
		set_target_speeds(speed*(1 - lane_correction_term) ,speed* (1 + lane_correction_term));
		speed = MIN(speed*0.975, 0);
		previous_distance_to_target = distance_to_target;
		HAL_Delay(2);
		distance_to_target = get_distance_to(x_closest, y_closest);

//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		uint8_t header1 = 0xAA;
		uint8_t header2 = 0xAB;
		uint8_t header3 = 0xAC;
		uint8_t header4 = 0xAD;
		float value1 = (float)current_speed_L;
		float value2 = (float)current_speed_R;
		float value3 = (float)target_speed_L;
		float value4 = (float)target_speed_R;
		uint8_t* bytes1 = (uint8_t*)&value1;
		uint8_t* bytes2 = (uint8_t*)&value2;
		uint8_t* bytes3 = (uint8_t*)&value3;
		uint8_t* bytes4 = (uint8_t*)&value4;
		HAL_UART_Transmit(&huart2, &header1, 1, 10);
		HAL_UART_Transmit(&huart2, bytes1, 4, 10);
		HAL_UART_Transmit(&huart2, &header2, 1, 10);
		HAL_UART_Transmit(&huart2, bytes2, 4, 10);
		HAL_UART_Transmit(&huart2, &header3, 1, 10);
		HAL_UART_Transmit(&huart2, bytes3, 4, 10);
		HAL_UART_Transmit(&huart2, &header4, 1, 10);
		HAL_UART_Transmit(&huart2, bytes4, 4, 10);
	}
	stop_motors();
	robot_state = DEAD_STOP;
	return 0;
}
float calculate_instant_target_speed_R(float x, float distance){
	if (robot_state == ACCELERATING){
		x = (x < 10)? 100/ramping_acceleration : x;
		if (direction_R == 1){
			return MIN(ramping_acceleration*x, cruise_speed_R);
		}else{
			return MAX(-ramping_acceleration*x, cruise_speed_R);
		}

	}else if (robot_state == CRUISING){
		return cruise_speed_R;

	}else if (robot_state == CONTROLLED_BREAK){
		if (direction_R == 1){
			return MAX(breaking_acceleration*(x-distance), 0);
		}else{
			return MIN(-breaking_acceleration*(x-distance), 0);
		}

	}else if (robot_state == EMERGENCY_BREAK){
		if (direction_R == 1){
			return MAX(target_speed_R + emergency_breaking_acceleration*x, 0);
		}else{
			return MIN(target_speed_R - emergency_breaking_acceleration*x, 0);
		}
	}
	return 0; // just to prevent debugger error, will be erased once a definitive else is added
}
float calculate_instant_target_speed_L(float x, float distance){
	if (robot_state == ACCELERATING){
		x = (x < 10)? 100/ramping_acceleration : x;
		if (direction_L == 1){
			return MIN(ramping_acceleration*x, cruise_speed_L);
		}else{
			return MAX(-ramping_acceleration*x, cruise_speed_L);
		}

	}else if (robot_state == CRUISING){
		return cruise_speed_L;

	}else if (robot_state == CONTROLLED_BREAK){
		if (direction_L == 1){
			return MAX(breaking_acceleration*(x-distance), 0);
		}else{
			return MIN(-breaking_acceleration*(x-distance), 0);
		}

	}else if (robot_state == EMERGENCY_BREAK){
		if (direction_L == 1){
			return MAX(target_speed_L + emergency_breaking_acceleration*x, 0);
		}else{
			return MIN(target_speed_L - emergency_breaking_acceleration*x, 0);
		}
	}
	return 0; // just to prevent debugger error, will be erased once a definitive else is added
}
float calculate_instant_target_phi(float x, float distance){
	if (robot_state == CRUISING || robot_state == ACCELERATING || robot_state == CONTROLLED_BREAK){
		return starting_phi;
	}
}
int rotate(float angle, int8_t is_radian){
	angle = is_radian? angle: deg_to_rad(angle);
	float arc_length = (dead_wheels_spacing/2) * angle;

	direction_R = SGN(arc_length);
	direction_L = -direction_R;

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float speed = rotating_speed;
	float distance = fabs(arc_length);
	float x1 = speed/ramping_acceleration;
	float x2 = speed/breaking_acceleration + distance;

	if (x2 <= x1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration*distance)/(breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed/ramping_acceleration;
		x2 = speed/breaking_acceleration + distance;
	}

	cruise_speed_R = speed * direction_R;
	cruise_speed_L = speed * direction_L;

	// I will track only the right wheel cause both wheels need to be in sync
	float distance_to_target = distance;
	float previous_distance_to_target = distance;
	float x;
	float target_phi = absolute_phi + angle;

	// Yaw centering, preventing that a wheel goes faster than the other and moves the rotation axis away from center
	float distance_traveled_R;
	float distance_traveled_L;
	const int32_t starting_encoder_count_R = encoder_count_R;
	const int32_t starting_encoder_count_L = encoder_count_L;
	float yaw_centering_term;

	float phase_two_angle = 15;

	robot_state = ACCELERATING;
	while ( fabs(target_phi - absolute_phi) > deg_to_rad(phase_two_angle)){
		x = distance - distance_to_target;
		if (robot_state == ACCELERATING && x > x1 ){
			robot_state = CRUISING;
		}else if (robot_state == CRUISING && x > x2){
			robot_state = CONTROLLED_BREAK;
		}else if (robot_state == EMERGENCY_BREAK){
			return 0;
		}

		distance_traveled_R = fabs(encoder_count_R - starting_encoder_count_R) * wheel_diameter_R * PI / CPR;
		distance_traveled_L = fabs(encoder_count_L - starting_encoder_count_L) * wheel_diameter_L * PI / CPR;

		yaw_centering_term = ( distance_traveled_R - distance_traveled_L ) * 0.02; // the coefficient is tunable
		if (x < 0.1*x1){
			yaw_centering_term = 0;
		}
		speed = calculate_instant_target_speed_R(x, distance);
		set_target_speeds(speed*(1 - yaw_centering_term) ,-speed* (1 + yaw_centering_term));
		HAL_Delay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = (dead_wheels_spacing/2) * fabs(target_phi - absolute_phi);

//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		uint8_t header1 = 0xAA;
		uint8_t header2 = 0xAB;
		uint8_t header3 = 0xAC;
		uint8_t header4 = 0xAD;
		float value1 = (float)current_speed_L;
		float value2 = (float)current_speed_R;
		float value3 = (float)target_speed_R;
		float value4 = (float)target_speed_L;
		uint8_t* bytes1 = (uint8_t*)&value1;
		uint8_t* bytes2 = (uint8_t*)&value2;
		uint8_t* bytes3 = (uint8_t*)&value3;
		uint8_t* bytes4 = (uint8_t*)&value4;
		HAL_UART_Transmit(&huart2, &header1, 1, 10);
		HAL_UART_Transmit(&huart2, bytes1, 4, 10);
		HAL_UART_Transmit(&huart2, &header2, 1, 10);
		HAL_UART_Transmit(&huart2, bytes2, 4, 10);
		HAL_UART_Transmit(&huart2, &header3, 1, 10);
		HAL_UART_Transmit(&huart2, bytes3, 4, 10);
		HAL_UART_Transmit(&huart2, &header4, 1, 10);
		HAL_UART_Transmit(&huart2, bytes4, 4, 10);
	}
	// completing the last part of distance separately, so the break happens on point

	speed = (target_speed_R - target_speed_L)/ 2; // one of the speeds is negative, that's why there is a minus
	if (fabs(speed) < (deg_to_rad(phase_two_angle) * dead_wheels_spacing/2)*(-breaking_acceleration) ){ // handling the case where the robot skips the first while loop, being too close to target_phi
		speed = (deg_to_rad(phase_two_angle) * dead_wheels_spacing/2)*(-breaking_acceleration) * direction_R;
	}
	distance_to_target = dead_wheels_spacing * fabs(target_phi - absolute_phi);
	previous_distance_to_target = distance_to_target;
	while (fabs(target_phi - absolute_phi) > deg_to_rad(0.2) && distance_to_target <= previous_distance_to_target + 10){
		distance_traveled_R = (encoder_count_R - starting_encoder_count_R) * wheel_diameter_R * PI / CPR;
		distance_traveled_L = (encoder_count_L - starting_encoder_count_L) * wheel_diameter_L * PI / CPR;
		yaw_centering_term = ( fabs(distance_traveled_R) - fabs(distance_traveled_L) ) * 0.05;

		set_target_speeds(speed*(1 - yaw_centering_term) ,-speed* (1 + yaw_centering_term));
		speed = MAX(fabs(speed)*0.98, 0) * direction_R;
		previous_distance_to_target = distance_to_target;
		HAL_Delay(2);
		distance_to_target = dead_wheels_spacing * fabs(target_phi - absolute_phi);

//		if (distance_to_target >= previous_distance_to_target+2){
//			stop_motors();
//			return 1;
//		}

		uint8_t header1 = 0xAA;
		uint8_t header2 = 0xAB;
		uint8_t header3 = 0xAC;
		uint8_t header4 = 0xAD;
		float value1 = (float)current_speed_L;
		float value2 = (float)current_speed_R;
		float value3 = (float)target_speed_R;
		float value4 = (float)target_speed_L;
		uint8_t* bytes1 = (uint8_t*)&value1;
		uint8_t* bytes2 = (uint8_t*)&value2;
		uint8_t* bytes3 = (uint8_t*)&value3;
		uint8_t* bytes4 = (uint8_t*)&value4;
		HAL_UART_Transmit(&huart2, &header1, 1, 10);
		HAL_UART_Transmit(&huart2, bytes1, 4, 10);
		HAL_UART_Transmit(&huart2, &header2, 1, 10);
		HAL_UART_Transmit(&huart2, bytes2, 4, 10);
		HAL_UART_Transmit(&huart2, &header3, 1, 10);
		HAL_UART_Transmit(&huart2, bytes3, 4, 10);
		HAL_UART_Transmit(&huart2, &header4, 1, 10);
		HAL_UART_Transmit(&huart2, bytes4, 4, 10);
	}
	stop_motors();
	robot_state = DEAD_STOP;
	return 0;
}
int orientate(float orientation, int8_t is_radian){
	orientation = (is_radian)? orientation : deg_to_rad(orientation);
	if (orientation != current_phi){
		float angle = orientation - current_phi;
		if (angle > PI){
			angle -= 2*PI;
		}else if (angle < -PI){
			angle += 2*PI;
		}
		return rotate(angle, 1);
	}
	return 0;
}
int go_to(float x, float y, float speed){
	if ( orientate( get_angle_to(x, y), 1 ) ){
		return 1;
	}
	HAL_Delay(500);
	return move_forward(get_distance_to(x, y), speed) * 2; // why *2 ? view in .h file
}
void emergency_break(){
	robot_state = EMERGENCY_BREAK;
	int32_t last_encoder_count_R = encoder_count_R;
	int32_t last_encoder_count_L = encoder_count_L;
	HAL_Delay(4);
	while (current_speed_R > 0.5 || current_speed_L > 0.5){
		float dx_R = ticks_to_distance(fabs(encoder_count_R - last_encoder_count_R), wheel_diameter_R);
		float dx_L = ticks_to_distance(fabs(encoder_count_L - last_encoder_count_L), wheel_diameter_L);

		set_target_speeds(calculate_instant_target_speed_R(dx_R, 0), calculate_instant_target_speed_L(dx_L, 0));

		last_encoder_count_R = encoder_count_R;
		last_encoder_count_L = encoder_count_L;
		HAL_Delay(4);
	}
	stop_motors();
}
