/*
 * TMC5062.h
 *
 *  Created on: 07.07.2017
 *      Author: LK
 *    Based on: TMC562-MKL.h (26.01.2012 OK)
 */

#ifndef TMC_IC_TMC5062_H_
#define TMC_IC_TMC5062_H_

#include "../../helpers/API_Header.h"
#include "TMC5062_Register.h"
#include "TMC5062_Mask_Shift.h"
#include "TMC5062_Constants.h"

#define TMC5062_FIELD_READ(tmc5062_ptr, channel, address, mask, shift) \
	FIELD_GET(tmc5062_readInt(tmc5062_ptr, channel, address), (mask), (shift))

#define TMC5062_FIELD_WRITE(tmc5062_ptr, channel, address, mask, shift, value) \
	tmc5062_writeInt((tmc5062_ptr), (channel), (address), FIELD_SET(tmc5062_readInt((tmc5062_ptr), (channel), (address)), (mask), (shift), (value)))

// Usage note: use one TypeDef per IC
typedef struct {
	int32 *shadowRegister;
	uint8 motors[TMC5062_MOTORS];

	// External frequency supplied to the IC (or 16MHz for internal frequency)
	uint32 chipFrequency;

	// Velocity estimation (for dcStep)
	uint32 measurementInterval;
	uint32 oldTick;
	int oldXActual[TMC5062_MOTORS];
	int velocity[TMC5062_MOTORS];

	int32 registerResetState[TMC5062_REGISTER_COUNT];
	const uint8 *registerAccess;
} TMC5062TypeDef;

void tmc5062_writeInt(TMC5062TypeDef *tmc5062, uint8 channel, uint8 address, int value);
int tmc5062_readInt(TMC5062TypeDef *tmc5062, uint8 channel, uint8 address);

void tmc5062_initConfig(TMC5062TypeDef *TMC5062, uint8 motorIndex0, uint8 motorIndex1, uint32 chipFrequency);
void tmc5062_periodicJob(TMC5062TypeDef *TMC5062, ConfigurationTypeDef *TMC5062_config, uint32 tick);
u8 tmc5062_reset(ConfigurationTypeDef *TMC5062_config);
u8 tmc5062_restore(ConfigurationTypeDef *TMC5062_config);

// Chopper settings
uint8 calculateTOFF(uint32 chopFreq, uint32 clkFreq);
// Diagnostics

// Stallguard
// Coolstep
// dcStep
uint8 dcStepActive(TMC5062TypeDef *tmc5062, uint8 channel);
// MSLUT
typedef struct {
	uint32 LUT_0;  // Bits   0 -  31
	uint32 LUT_1;  // Bits  32 -  63
	uint32 LUT_2;  // Bits  64 -  95
	uint32 LUT_3;  // Bits  96 - 127
	uint32 LUT_4;  // Bits 128 - 159
	uint32 LUT_5;  // Bits 160 - 191
	uint32 LUT_6;  // Bits 192 - 223
	uint32 LUT_7;  // Bits 224 - 255

	uint8 X1;     // Segment 1
	uint8 X2;     // Segment 2
	uint8 X3;     // Segment 3
	// Segment width. Determines bit meaning (Actual bit value = (Wn - 1) + bit)
	uint8 W0 : 2; // Segment [ 0 : X1-1]
	uint8 W1 : 2; // Segment [X1 : X2-1]
	uint8 W2 : 2; // Segment [X2 : X3-1]
	uint8 W3 : 2; // Segment [X3 :  255]

	uint8  START_SIN;    // Absolute current at MSLUT[0]
	uint8  START_SIN90;  // Absolute current at MSLUT[256]
} TMC5062_MicroStepTable;

uint8 setMicroStepTable(TMC5062TypeDef *tmc5062, uint8 channel, TMC5062_MicroStepTable *table);

// Encoder
uint32 setEncoderFactor(TMC5062TypeDef *tmc5062, uint8 channel, uint32 motorFullSteps, uint32 microSteps, uint32 encoderResolution);

#endif /* TMC_IC_TMC5062_H_ */
