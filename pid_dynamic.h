/*
 * pid_dynamic.h
 *
 *  Created on: Jan 1, 2024
 *      Author: Mathew
 */

#ifndef INC_PID_DYNAMIC_H_
#define INC_PID_DYNAMIC_H_

#include"stm32f4xx_hal.h"
#include "NOTCH_FILTER.h"
#include "IIR.h"


/* Structure of PID to hold runtime parameters*/
typedef struct{

	float Kp;
	float Ki;
	float Kd;

	uint32_t lastTime;

	float output;
	float minOutput;
	float maxOutput;
	float minIntegral;
	float maxIntegral;

	float lastInput;

	float Pterm;
	float Iterm;
	float Dterm;
	float DtermUnfilt;

	float dFilt1Output;
	float dFilt2Output;

	pt1Filter_t dFilt1;
	pt2Filter_t dFilt2;

	uint8_t hasNotchFilter;
	NotchFilter notchFilter;

	uint8_t initialized;
	uint8_t isOn;

} structPID;


void PID_Init(structPID *PID, float initKp, float initKi, float initKd, float sampleTime);

float PID_Compute(structPID *PID, float newInput, float newSetpoint, uint32_t newTime);

void SetTunings(structPID *PID, float newKp, float newKi, float newKd);

void SetOutputLimits(structPID *PID, float minOutput, float maxOutput, float minIntegral, float maxIntegral);

void PID_Set_On(structPID *PID);

void PID_Set_Off(structPID *PID);

void PID_Reset(structPID *PID, float newInput);

void PID_SetNotch(structPID *PID, NotchFilter *notchFilt);




#endif /* INC_PID_DYNAMIC_H_ */
