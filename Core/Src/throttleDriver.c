#include "throttleDriver.h"

//resources used: https://wiki.st.com/stm32mcu/wiki/Getting_started_with_DAC
uint8_t dacBuffer[2];
uint8_t rxBuf[2];

// Step 1: Transmit

// Step 2: In TX callback, trigger a read
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_Master_Receive_IT(hi2c, DAC_ADDR, rxBuf, 2);
}

// Step 3: RX callback fires when response is received
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	// TODO what is this function supposed to do? 
}

void DAC_SetValue(uint16_t val) {
	// add some sort of RTOS check so buffer isn't overwritten by another call?
	// halts execution until I2C line is clear/ready. 
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}
	// Clamp to 12-bit max
	if (val > DAC_MAX_VALUE) {
		val = DAC_MAX_VALUE;
	}

	dacBuffer[0] = (uint8_t)((val >> 8) & 0x0F);
	//        C2=0,C1=0  PD1=0,PD0=0   upper 4 bits of value
	dacBuffer[1] = (uint8_t)(val & 0x00FF);
	//        lower 8 bits of value

	HAL_I2C_Master_Transmit_IT(&hi2c1, DAC_ADDR, dacBuffer, 2); // 0x63 << 1 = 0xC6
}

void throttleTest() {
	//start throttle at 0V, jump in increments of 1.22mV to 5V.
	accelerate(DAC_MAX_VALUE); 

	//decrement test
	decelerate(DAC_MAX_VALUE); 
}

void accelerate(int targetVal) {
	if (targetVal > DAC_MAX_VALUE) {
		targetVal = DAC_MAX_VALUE; 
	}

	if (targetVal < 0) {
		targetVal = 0; 
	}

	for (int dac_value = 0; dac_value <= targetVal; dac_value++) { // for loop gets overwritten
		DAC_SetValue(dac_value);
		HAL_Delay(STEP_DELAY_MS); 	// changed delay from 1 to 5 milliseconds 
									// change if its too fast (the LIM blows up)
	}
}

void decelerate(int targetVal) {
		if (targetVal > DAC_MAX_VALUE) {
		targetVal = DAC_MAX_VALUE; 
	}

	if (targetVal < 0) {
		targetVal = 0; 
	}

	for (int dac_value = targetVal; dac_value >= 0; dac_value--) {
		DAC_SetValue(dac_value);
		HAL_Delay(STEP_DELAY_MS); 	// changed delay from 1 to 5 milliseconds 
						// change if its too fast (the LIM blows up)
	}
}

void killThrottle() {
	DAC_SetValue(0);
}
