/*
 *  bsp_motor.h
 *
 *  V1: Provide DC Motor & Actuator control functions
 *  V2: Provide Servo Motor control function
 * 
 *  Created on: Jan 11, 2023
 *      Author: Jackie Huynh & Michelle Tran
 * 
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "bsp_deployment.h"
#include <stdint.h>

//Servo Motor Error Enum
typedef enum{

	SERVO_OK 		 		= 0x00U,
	SERVO_FREQ_ERROR 		= 0x01U,
	SERVO_RANGE_ERROR_MIN	= 0x02U,
	SERVO_RANGE_ERROR_MAX	= 0x03U,
	SERVO_INSTANCE_ERROR	= 0x04U

} Servo_Error;

//DC Motor Error Enum
typedef enum{
    DC_MOTOR_OK             = 0x00U,
	DC_MOTOR_UNDER_RANGE	= 0x01U,
	DC_MOTOR_ABOVE_RANGE	= 0x02U,
    DC_MOTOR_INSTANCE_ERR   = 0x03U
} DCMotor_Error;

//DC Motor Direction Enum
typedef enum{
    CLOCKWISE               = 0x00U,
    COUNTER_CLOCKWISE       = 0x01u
} DCMotor_Direction;

//Actuator Enum
typedef enum{
    ACTUATOR_OK             = 0x00U,
    ACTUATOR_UNDER_RANGE    = 0x01U,
    ACTUATOR_ABOVE_RANGE    = 0x02U,
    ACTUATOR_INSTANCE_ERROR = 0x03U,
    ACTUATOR_FREQ_ERROR     = 0x04U
} Actuator_Error;

//Servo Motor Struct
typedef struct{

	float minDuty;
	float maxDuty;
	int8_t minAngle;
	int8_t maxAngle;

} Servo_Config_t;

typedef struct{

	TIM_HandleTypeDef*	htim;
	uint8_t				channel;
	Servo_Config_t*		config;

	/* These will be set in the INIT function */
	uint32_t 			minCnt;
	uint32_t 			maxCnt;

} Servo_Instance_t;

//DC Motor Struct
typedef struct{
	uint8_t Min_Speed;						//Percentage Based
	uint8_t Max_Speed;						//Percentage Based
} DCMotor_Config_t;

typedef struct{
	TIM_HandleTypeDef* 	DC_Timer;
	uint8_t 			IN1_Channel;
	uint8_t 			IN2_Channel;
	uint16_t			Min_Cnt;                      
	uint16_t			Max_Cnt;   
	DCMotor_Config_t* 	config;                   
} DCMotor_Instance_t;

//DC Motor W Encoder Structs
typedef struct{
	uint32_t 	Default_Counter;
	float 		Degree_Per_Pulse;
	int16_t 	Current_Angle;
	int16_t 	Min_Angle;
	int16_t 	Max_Angle;
} DCMotor_Encoder_Config_t;

typedef struct{
	TIM_HandleTypeDef*			Encoder_Timer;
	DCMotor_Encoder_Config_t* 	encConfig;
	DCMotor_Instance_t* 	  	motorInstance;
} DCMotor_Encoder_Instance_t;

//Actuator Struct
typedef struct{
    uint16_t 	Min_Pulse;                     //uS Based
    uint16_t 	Max_Pulse;                     //uS Based
    uint8_t 	Min_Length;                    //mm Based
    uint8_t 	Max_Length;                    //mm Based
	uint8_t		Desired_Min_Length;			   //mm Based
	uint8_t 	Desired_Max_Length;			   //mm Based
} Actuator_Config_t;

typedef struct 
{
	TIM_HandleTypeDef* Act_Timer;
	uint8_t Channel;
    Actuator_Config_t* config;

    /* Will be Initialized by function */
	uint16_t Min_Cnt;                     
	uint16_t Max_Cnt;                     
} Actuator_Instance_t;

/* List of Macros */
#define DESIRED_SERVO_FREQ  (50U)           //In Hz
#define DESIRED_ACT_FREQ    (50U)           //In Hz

//Servo Functions
Servo_Error Servo_Init(Servo_Instance_t* servo);
Servo_Error Drive_Servo(const Servo_Instance_t* servo, const int8_t angle);

//DC Motor Functions
DCMotor_Error DCMotor_Init(DCMotor_Instance_t* dcMotor);
DCMotor_Error Drive_DCMotor(const DCMotor_Instance_t* dcMotor, const uint8_t speed, DCMotor_Direction dir);
DCMotor_Error Stop_DCMotor(const DCMotor_Instance_t* dcMotor);

//DC Motor With Encoder Function
DCMotor_Error DCMotor_Encoder_Init(DCMotor_Encoder_Instance_t* encMotor);
DCMotor_Error Drive_DCMotor_Angle(const DCMotor_Encoder_Instance_t* encMotor, int16_t angle);

//Actuator Functions
Actuator_Error Actuator_Init(Actuator_Instance_t* act);
Actuator_Error Drive_Actuator(const Actuator_Instance_t* act, const uint8_t length);

#endif