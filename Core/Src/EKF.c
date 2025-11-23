/*
 * EKF.c
 *
 *  Created on: Nov 23, 2025
 *      Author: monji
 */


#include "EKF.h"

void set_IMU_I2C_channel(I2C_HandleTypeDef *hi2c){
	bno055_init(hi2c, BNO055_I2C_ADDR_LO);
}

void update_position_EKF(){
	bno055_vector_t v = bno055_readEuler();
	debugger = v.z;
	send_values_to_usart(v.x, 0, 0, 0);
}
