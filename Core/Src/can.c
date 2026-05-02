#include "can.h"
#include <math.h>

uint8_t	RxData250[8]; // 8 bytes
FDCAN_RxHeaderTypeDef RxHeader250;
uint8_t	RxData500[8]; // 8 bytes
FDCAN_RxHeaderTypeDef RxHeader500;


volatile int head250 = 0;
volatile int tail250 = 0;
volatile int head500 = 0;
volatile int tail500 = 0;


volatile uint32_t can1_rx_count = 0;
volatile uint32_t can2_rx_count = 0;


// debugging variables, delete later
volatile int x = 0;
volatile int y;
volatile int z;
volatile int a;

CAN_Message canQ250[CAN_QUEUE_SIZE];
CAN_Message canQ500[CAN_QUEUE_SIZE];



/* Calculating Functions */
double concatenate(uint8_t x, uint8_t y){
	return ((uint16_t)x << 8) | (uint16_t) y;
}

double rpm_to_ms(double rpm) {
    return (2 * M_PI * (.00395) * rpm) / 60.0;
}

/* Stores any random ID's seen */
//Logs for CAN testing
uint32_t unknownIDs[UNKNOWN_LOG_SIZE];
volatile int unknownIndex = 0;
int value = 0;

int isDuplicate(uint32_t id){
	for(int i = 0; i < UNKNOWN_LOG_SIZE; i++){
		if(unknownIDs[i] == id)
			return 1;
	}
	return 0;
}



void process_CAN250_msgs(VFD_CAN_Data *vfdData) {
	while (tail250 != head250) {
		CAN_Message currentMessage = canQ250[tail250];
		tail250 = (tail250 + 1) % CAN_QUEUE_SIZE;

		//other can peripherals can be added here as cases w/ their ID
		switch (currentMessage.id) {
		//KELLY VFD - 0x10F8109A (driving direction, speed in rpm

		//NOTE: ID 1713 is coming from the VFD

		case 0x10F8109A:
			// ID (decimal): 284692634
			vfdData -> drivingDirection = currentMessage.data[0];
			vfdData -> encoderSpeed = rpm_to_ms(concatenate(currentMessage.data[1], currentMessage.data[2]));
			vfdData -> errorCode = currentMessage.data[3];	//see table 1 of Kelly VFD datasheet
			break;
		//KELLY VFD - 0x10F8108D (battery voltage, motor current, motor temp, controller temp)
		case 0x10F8108D:
			// ID (decimal): 284692621
			vfdData-> batteryVoltage = (concatenate(currentMessage.data[1], currentMessage.data[0]) / 10);
			vfdData-> motorCurrent = (concatenate(currentMessage.data[3], currentMessage.data[2]));
			vfdData-> motorTemp = ((concatenate(currentMessage.data[5], currentMessage.data[4]))/10);
			vfdData-> controllerTemp = ((concatenate(currentMessage.data[7], currentMessage.data[6]))/10);
			break;
		default:
			if(!isDuplicate(currentMessage.id)){
				unknownIDs[unknownIndex] = currentMessage.id;
				unknownIndex = (unknownIndex +1) % UNKNOWN_LOG_SIZE;
			}
			break;
		}
	}
}

void process_CAN500_msgs(BMS_CAN_Data *bmsData, IMD_CAN_Data *imdData) {
	while (tail500 != head500) {
		CAN_Message currentMessage = canQ500[tail500];
		tail500 = (tail500 + 1) % CAN_QUEUE_SIZE;

		//other can peripherals can be added here as cases w/ their ID
		switch (currentMessage.id) {
		// IMD ID
		case 0x18FF01F4:
			//419365364
			imdData->insulationResistance = concatenate(currentMessage.data[1], currentMessage.data[0]);
			imdData->iso_status = currentMessage.data[2];
			imdData->imd_counter = currentMessage.data[3];
			imdData->imd_warnings = concatenate(currentMessage.data[5], currentMessage.data[4]);
			imdData->deviceActivity = currentMessage.data[6];
			break;

		case 0x10DEADBE:
			// ID (decimal): 283028926
			bmsData->lowestCellVoltage = concatenate(currentMessage.data[1], currentMessage.data[0]);
			bmsData->avgCellVoltage = concatenate(currentMessage.data[3], currentMessage.data[2]);
			bmsData->highestCellVoltage = concatenate(currentMessage.data[5], currentMessage.data[4]);
			bmsData->packSOC = currentMessage.data[6];
			bmsData->bmsTestCounter = currentMessage.data[7];
			break;
		default:
			if(!isDuplicate(currentMessage.id)){
				unknownIDs[unknownIndex] = currentMessage.id;
				unknownIndex = (unknownIndex +1) % UNKNOWN_LOG_SIZE;
			}
			break;
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) { // RxFifo0ITs using interrupt 0
	//overriding the builtin function that runs when a CAN interrupt is detected
	x = 3;
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) { //If Interrupt flag is 1 AND there is a new message in the queue...
		can1_rx_count++;

		// While loop to pop messages from the queue (useful in event of multiple msg arriving at once
		while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {

			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader250, RxData250);

		  	int next = (head250 + 1) % CAN_QUEUE_SIZE;

		  	if (next != tail250) {
		  		canQ250[head250].id = RxHeader250.Identifier;
		  		for (int i = 0; i < 8; i++) {
		  			canQ250[head250].data[i] = RxData250[i];
		  		}
		  		head250 = next;
		  	}
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) { // RxFifo1ITs using interrupt 1
	//overriding the builtin function that runs when a CAN interrupt is detected
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) { //If Interrupt flag is 1 AND there is a new message in the queue...
		can2_rx_count++;

		// While loop to pop messages from the queue (useful in event of multiple msg arriving at once
		while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0) {

			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader500, RxData500);

		  	int next = (head500 + 1) % CAN_QUEUE_SIZE;

		  	if (next != tail500) {
		  		canQ500[head500].id = RxHeader500.Identifier;
		  		for (int i = 0; i < 8; i++) {
		  			canQ500[head500].data[i] = RxData500[i];
		  		}
		  		head500 = next;
		  	}
		}
	}
}
