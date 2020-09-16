/*
 * MPU6050_LIB.h
 *
 *  Created on: Aug 27, 2020
 *      Author: lenovo
 */

#ifndef INC_MPU6050_LIB_H_
#define INC_MPU6050_LIB_H_

#include "main.h"
#include <string.h>

extern float Ax, Ay, Az, Gx, Gy, Gz;
extern I2C_HandleTypeDef hi2c1;

//Initialization Functions
void MPU6050_Init (void); // Initialize MPU6050


//user functions
void MPU6050_Read_Accel (void);
void MPU6050_Read_Gyro (void);



#endif /* INC_MPU6050_LIB_H_ */
