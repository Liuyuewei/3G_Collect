/*
 * TMC5062.c
 *
 *  Created on: 06.07.2017
 *      Author: LK
 *    Based on: TMC562-MKL.h (26.01.2012 OK)
 */

#include "TMC5062.h"

// Default Register Values
#define R30 0x00071703  // IHOLD_IRUN
#define R32 0x00FFFFFF  // VHIGH
#define R3A 0x00010000  // ENC_CONST
#define R60 0xAAAAB554  // MSLUT[0]
#define R61 0x4A9554AA  // MSLUT[1]
#define R62 0x24492929  // MSLUT[2]
#define R63 0x10104222  // MSLUT[3]
#define R64 0xFBFFFFFF  // MSLUT[4]
#define R65 0xB5BB777D  // MSLUT[5]
#define R66 0x49295556  // MSLUT[6]
#define R67 0x00404222  // MSLUT[7]
#define R68 0xFFFF8056  // MSLUTSEL
#define R69 0x00F70000  // MSLUTSTART
#define R6C 0x000101D5  // CHOPCONF

/* Register access permissions:
 * 0: none (reserved)
 * 1: read
 * 2: write
 * 3: read/write
 * 7: read^write (seperate functions/values)
 */
const u8 tmc5062_defaultRegisterAccess[TMC5062_REGISTER_COUNT] = {
//	0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	3, 1, 1, 2, 7, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x00 - 0x0F
	2, 1, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, // 0x10 - 0x1F
	3, 3, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 0, 0, // 0x20 - 0x2F
	2, 2, 2, 2, 3, 1, 1, 0, 3, 3, 2, 1, 1, 0, 0, 0, // 0x30 - 0x3F
	3, 3, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 0, 0, // 0x40 - 0x4F
	2, 2, 2, 2, 3, 1, 1, 0, 3, 3, 2, 1, 1, 0, 0, 0, // 0x50 - 0x5F
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 3, 2, 2, 1, // 0x60 - 0x6F
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 3, 2, 2, 1  // 0x70 - 0x7F
};

const s32 tmc5062_defaultRegisterResetState[TMC5062_REGISTER_COUNT] = {
//	0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x00 - 0x0F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x20 - 0x2F
	R30, 0,   R32, 0,   0,   0,   0,   0,   0,   0,   R3A, 0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	R30, 0,   R32, 0,   0,   0,   0,   0,   0,   0,   R3A, 0,   0,   0,   0,   0, // 0x50 - 0x5F
	R60, R61, R62, R63, R64, R65, R66, R67, R68, R69, 0,   0,   R6C, 0,   0,   0, // 0x60 - 0x6F
	R60, R61, R62, R63, R64, R65, R66, R67, R68, R69, 0,   0,   R6C, 0,   0,   0  // 0x70 - 0x7F
};

static void measureVelocity(TMC5062TypeDef *tmc5062, uint32 tick);

// => SPI wrapper
extern uint8 tmc5062_readWrite(uint8 motor, uint8 data, uint8 lastTransfer);
// <= SPI wrapper

void tmc5062_writeInt(TMC5062TypeDef *tmc5062, uint8 channel, uint8 address, int value)
{
	if(channel >= TMC5062_MOTORS)
		return;

	tmc5062_readWrite(tmc5062->motors[channel], address | TMC5062_WRITE_BIT, FALSE);
	tmc5062_readWrite(tmc5062->motors[channel], value >> 24, FALSE);
	tmc5062_readWrite(tmc5062->motors[channel], value >> 16, FALSE);
	tmc5062_readWrite(tmc5062->motors[channel], value >>  8, FALSE);
	tmc5062_readWrite(tmc5062->motors[channel], value,       TRUE);

	// Shadow register
	if(tmc5062->shadowRegister)
		tmc5062->shadowRegister[address] = value;
}

int tmc5062_readInt(TMC5062TypeDef *tmc5062, uint8 channel, uint8 address)
{
	if(channel >= TMC5062_MOTORS)
		return 0;

	address &= ~TMC5062_WRITE_BIT;

	if(!TMC_IS_READABLE(tmc5062->registerAccess[address]))
		return (tmc5062->shadowRegister)? tmc5062->shadowRegister[address] : 0;

	tmc5062_readWrite(tmc5062->motors[channel], address, FALSE);
	tmc5062_readWrite(tmc5062->motors[channel], 0, FALSE);
	tmc5062_readWrite(tmc5062->motors[channel], 0, FALSE);
	tmc5062_readWrite(tmc5062->motors[channel], 0, FALSE);
	tmc5062_readWrite(tmc5062->motors[channel], 0, TRUE);

	int value = 0;

	tmc5062_readWrite(tmc5062->motors[channel], address, FALSE);
	value |= tmc5062_readWrite(tmc5062->motors[channel], 0, FALSE);
	value <<= 8;
	value |= tmc5062_readWrite(tmc5062->motors[channel], 0, FALSE);
	value <<= 8;
	value |= tmc5062_readWrite(tmc5062->motors[channel], 0, FALSE);
	value <<= 8;
	value |= tmc5062_readWrite(tmc5062->motors[channel], 0, TRUE);

	return value;
}

void tmc5062_initConfig(TMC5062TypeDef *tmc5062, uint8 motorIndex0, uint8 motorIndex1, uint32 chipFrequency)
{
	tmc5062->motors[0] = motorIndex0;
	tmc5062->motors[1] = motorIndex1;

	tmc5062->chipFrequency  = chipFrequency;

	tmc5062->measurementInterval = 25; // Default: 25 ms
	tmc5062->oldTick        = 0;
	tmc5062->oldXActual[0]  = 0;
	tmc5062->oldXActual[1]  = 0;
	tmc5062->velocity[0]    = 0;
	tmc5062->velocity[1]    = 0;

	tmc5062->registerAccess = &tmc5062_defaultRegisterAccess[0];

	for(int i = 0; i < TMC5062_REGISTER_COUNT; i++)
	{
		tmc5062->registerResetState[i]  = tmc5062_defaultRegisterResetState[i];
	}
}

void tmc5062_writeConfiguration(TMC5062TypeDef *tmc5062, ConfigurationTypeDef *TMC5062_config)
{
	uint8 *ptr = &TMC5062_config->configIndex;
	tmc5062->shadowRegister = (TMC5062_config->state == CONFIG_RESTORE) ? TMC5062_config->shadowRegister : tmc5062->registerResetState;

	while((*ptr < TMC5062_REGISTER_COUNT) && !TMC_IS_WRITABLE(tmc5062->registerAccess[*ptr]))
		(*ptr)++;

	if(*ptr < TMC5062_REGISTER_COUNT)
	{
		tmc5062_writeInt(tmc5062, 0, *ptr, tmc5062->shadowRegister[*ptr]);
		(*ptr)++;
	}
	else
	{
		TMC5062_config->state = CONFIG_READY;
	}
}

void tmc5062_periodicJob(TMC5062TypeDef *tmc5062, ConfigurationTypeDef *TMC5062_config, uint32 tick)
{
	if(TMC5062_config->state != CONFIG_READY)
	{
		tmc5062_writeConfiguration(tmc5062, TMC5062_config);
		return;
	}

	for(uint8 channel = 0; channel < TMC5062_MOTORS; channel++)
	{
		if(dcStepActive(tmc5062, channel))
		{
			// Measure if any channel has active dcStep
			measureVelocity(tmc5062, tick);
			break;
		}
	}
}

uint8 tmc5062_reset(ConfigurationTypeDef *TMC5062_config)
{
	if(TMC5062_config->state != CONFIG_READY)
		return 0;

	TMC5062_config->state        = CONFIG_RESET;
	TMC5062_config->configIndex  = 0;

	return 1;
}

uint8 tmc5062_restore(ConfigurationTypeDef *TMC5062_config)
{
	if(TMC5062_config->state != CONFIG_READY)
		return 0;

	TMC5062_config->state        = CONFIG_RESTORE;
	TMC5062_config->configIndex  = 0;

	return 1;
}

// Chopper settings
uint8 calculateTOFF(uint32 chopFreq, uint32 clkFreq)
{
	// Calculate TOff from the clock and chopper frequencies (see documentation
	// for details). We add 16 before dividing by 32 to have rounding instead of flooring.
	uint8 result = (((clkFreq / chopFreq / 4) - 12) + 16) / 32;

	result = MIN(15, result);
	result = MAX(1, result);

	return result;
}

// Diagnostics

// Stallguard

// Coolstep

// dcStep
uint8 dcStepActive(TMC5062TypeDef *tmc5062, uint8 channel)
{

	// vhighfs and vhighchm set?
	int chopConf = tmc5062_readInt(tmc5062, channel, TMC5062_CHOPCONF(channel));
	if((chopConf & (TMC5062_VHIGHFS_MASK | TMC5062_VHIGHCHM_MASK)) != (TMC5062_VHIGHFS_MASK | TMC5062_VHIGHCHM_MASK))
		return 0;

	// Velocity above dcStep velocity threshold?
	int vActual = tmc5062_readInt(tmc5062, channel, TMC5062_VACTUAL(channel));
	int vDCMin  = tmc5062_readInt(tmc5062, channel, TMC5062_VDCMIN(channel));

	return vActual >= vDCMin;
}

static void measureVelocity(TMC5062TypeDef *tmc5062, uint32 tick)
{
	int xActual;
	uint32 tickDiff;

	if((tickDiff = tick - tmc5062->oldTick) >= tmc5062->measurementInterval)
	{
		for(uint8 channel = 0; channel < TMC5062_MOTORS; channel++)
		{
			xActual = tmc5062_readInt(tmc5062, channel, TMC5062_XACTUAL(channel));

			// Position difference gets multiplied by 1000 to compensate ticks being in milliseconds
			int32 xDiff = (xActual - tmc5062->oldXActual[channel])* 1000;
			tmc5062->velocity[channel] = (xDiff) / ((float) tickDiff) * ((1<<24) / (float) tmc5062->chipFrequency);

			tmc5062->oldXActual[channel] = xActual;
		}
		tmc5062->oldTick = tick;
	}
}

// MSLUT
uint8 setMicroStepTable(TMC5062TypeDef *tmc5062, uint8 channel, TMC5062_MicroStepTable *table)
{
	if(channel >= TMC5062_MOTORS || table == 0 || tmc5062 == 0)
		return 0;

	tmc5062_writeInt(tmc5062, channel, TMC5062_MSLUT0(channel), table->LUT_0);
	tmc5062_writeInt(tmc5062, channel, TMC5062_MSLUT1(channel), table->LUT_1);
	tmc5062_writeInt(tmc5062, channel, TMC5062_MSLUT2(channel), table->LUT_2);
	tmc5062_writeInt(tmc5062, channel, TMC5062_MSLUT3(channel), table->LUT_3);
	tmc5062_writeInt(tmc5062, channel, TMC5062_MSLUT4(channel), table->LUT_4);
	tmc5062_writeInt(tmc5062, channel, TMC5062_MSLUT5(channel), table->LUT_5);
	tmc5062_writeInt(tmc5062, channel, TMC5062_MSLUT6(channel), table->LUT_6);
	tmc5062_writeInt(tmc5062, channel, TMC5062_MSLUT7(channel), table->LUT_7);

	uint32 tmp =   (table->X3 << 24) | (table->X2 << 16) | (table->X1 << 8)
			     | (table->W3 <<  6) | (table->W2 <<  4) | (table->W1 << 2) | (table->W0);
	tmc5062_writeInt(tmc5062, channel, TMC5062_MSLUTSEL(channel), tmp);

	tmp = (table->START_SIN90 << 16) | (table->START_SIN);
	tmc5062_writeInt(tmc5062, channel, TMC5062_MSLUTSTART(channel), tmp);

	return 1;
}

// Encoder
/* Calculate and set the encoder factor. Returns the deviation to the theoretical factor in
 * either binary or decimal form. To get the actual error value, divide the return type:
 * Binary:   Error = retVal / (2^16  * encoderResolution)
 * Decimal:  Error = retVal / (10000 * encoderResolution)
 * (Check the enc_sel_decimal bit in the ENCMODE register to find out which mode is used)
 */
uint32 setEncoderFactor(TMC5062TypeDef *tmc5062, uint8 channel, uint32 motorFullSteps, uint32 microSteps, uint32 encoderResolution)
{
	int numerator, denominator, remainder;
	int binaryError, decimalError;
	uint8 useDecimal, binaryRounded = FALSE, decimalRounded = FALSE;

	// Check for divisor 0
	// Return value here is inaccurate, this is just a protection against runtime/usage error.
	if(encoderResolution == 0)
		return 0;

	numerator = (motorFullSteps * microSteps) / encoderResolution;
	remainder = (motorFullSteps * microSteps) % encoderResolution;

	if((binaryError = (remainder << 16) % encoderResolution) == 0)
	{
		useDecimal = 0;
		denominator = (remainder << 16) / encoderResolution;
		remainder = 0;
	}
	else if((decimalError = (remainder * 10000) % encoderResolution) == 0)
	{
		useDecimal = 1;
		denominator = (remainder * 10000) / encoderResolution;
		remainder = 0;
	}
	else // No exact fraction possible -> calculate nearest value
	{
		// The integer division/modulo calculation results in flooring.
		// Compensate here to achieve rounding instead (smaller error)
		// (Cast to uint32 to avoid compiler warning - variable is positive anyways)
		if((uint32) binaryError > (encoderResolution/2))
		{
			binaryError -= encoderResolution;
			binaryRounded = TRUE;
		}

		if((uint32) decimalError > (encoderResolution/2))
		{
			decimalError -= encoderResolution;
			decimalRounded = TRUE;
		}

		if((abs(binaryError) * 10000) <= (abs(decimalError) << 16))
		{
			// Binary error smaller -> use binary
			useDecimal = 0;
			denominator = (remainder << 16) / encoderResolution + ((binaryRounded)? 1:0);
			remainder = binaryRounded;
		}
		else
		{
			// Decimal error smaller -> use decimal
			useDecimal = 1;
			denominator = (remainder * 10000) / encoderResolution + ((decimalRounded)? 1:0);
			remainder = decimalRounded;
		}
	}

	uint32 tmp = (numerator << 16) | (denominator & 0xFFFF);
	tmc5062_writeInt(tmc5062, channel, TMC5062_ENC_CONST(channel), tmp);
	TMC5062_FIELD_WRITE(tmc5062, channel, TMC5062_ENCMODE(channel), TMC5062_ENC_SEL_DECIMAL_MASK, TMC5062_ENC_SEL_DECIMAL_SHIFT, useDecimal);

	return remainder;
}

