/*
 * sbus.h
 *
 *  Created on: Dec 18, 2023
 *      Author: kobaike
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

typedef enum{
	ANALOG_RX,
	ANALOG_RY,
	ANALOG_LX,
	ANALOG_LY,
	ANALOG_NUMBER,
}STICK_NUMBER;

typedef enum{
	SW_A,
	SW_B,
	SW_C,
	SW_D,
	SW_E,
	SW_F,
	SW_G,
	SW_H,
	VR,
	SWITCH_NUMBER,
} SW_NUMBER ;

typedef enum{
	LOW,
	NEUTRAL,
	HIGH,
} SWITCH_POSITION;

typedef enum{
	SBUS_SIGNAL_OK,
	SBUS_SIGNAL_LOST,
	SBUS_SIGNAL_FAILSAFE,
}SBUS_STATUS;

typedef enum{
	CH5 = 4,
	CH6,
	CH7,
	CH8,
	CH9,
	CH10,
}SBUS_CH_NUMBER ;

typedef enum{false=0,true=!false}bool;
bool enableReceive;
int failsafe_status;
int stickMaximumValue;
int stickNeutralValue;
int stickMinimumValue;
int stickResolution;
float volumeValue;
extern int CH[20];

int swSelectData[6];

int switchValue[12];
float stickValue[ANALOG_NUMBER];
extern uint8_t buf[50];

void convertReceiveData() {
		for(int i=ANALOG_RX;i<=ANALOG_LY;i++) {
			float buf = 0.0f;
			if(CH[i] > (stickNeutralValue + 20)) buf = ((CH[i] - stickNeutralValue) / (float)(stickMaximumValue - stickNeutralValue));
			else if(CH[i] < (stickNeutralValue - 20)) buf = -((CH[i] - stickNeutralValue) / (float)(stickMinimumValue - stickNeutralValue));
			else buf = 0.0f;

			buf = ((buf*100)/100.0f);
			if(buf > 1.0f)          buf = 1.0f;
			else if(buf < -1.0f)    buf = -1.0f;

			switch (i) {
				case 0:
					stickValue[ANALOG_RX] = buf;
					break;
				case 1:
					stickValue[ANALOG_LY] = -buf;
					break;
				case 2:
					stickValue[ANALOG_RY] = buf;
					break;
				case 3:
					stickValue[ANALOG_LX] = buf;
					break;
			}
		}


		for(int i = 0; i < 6; i++) {
			if(swSelectData[i] == VR) {
				float vr_buf = 0.0f;
				if(CH[4] > (stickNeutralValue + 20)) vr_buf = ((CH[4] - stickNeutralValue) / (float)(stickMaximumValue - stickNeutralValue));
				else if(CH[4] < (stickNeutralValue - 20)) vr_buf = -((CH[4] - stickNeutralValue) / (float)(stickMinimumValue - stickNeutralValue));
				else vr_buf = 0.0f;

				vr_buf = (int)(vr_buf*100)/100.0f;
				if(vr_buf > 1.0f) vr_buf = 1.0f;
				else if(vr_buf < -1.0f) vr_buf = -1.0f;
				volumeValue = vr_buf;
			}
		}

		for(int i = 0; i < 6; i++) {
			switch(CH[i + 4]) {
				case 0x078b:
				case 0x770:
					switchValue[swSelectData[i]] = HIGH;
					break;
				case 0x0400:
					switchValue[swSelectData[i]] = NEUTRAL;
					break;
				case 0x0090:
					switchValue[swSelectData[i]] = LOW;
					break;
				default:
					switchValue[i] = NEUTRAL;
					break;
			}
		}
	}

void resetAllData(){
	for(int i = 0; i < 16; i++) CH[i] = 0;
	for(int i = 0; i < 50; i++) buf[i] = 0;
	for(int i = 0; i < 4; i++) stickValue[i] = 0.0f;
	for(int i = 0; i < 12; i++) switchValue[i] = NEUTRAL;
	volumeValue = 0.0f;
}

void sbus(){
	stickMaximumValue = 0x0690;
	stickNeutralValue = 0x0400;
	stickMinimumValue = 0x0170;
	stickResolution   = stickMaximumValue - stickMinimumValue;

	enableReceive = false;
	failsafe_status = SBUS_SIGNAL_FAILSAFE;

	resetAllData();
}

void setup(int ch5,int ch6,int ch7,int ch8,int ch9,int ch10) {
	swSelectData[0] = ch5;
	swSelectData[1] = ch6;
	swSelectData[2] = ch7;
	swSelectData[3] = ch8;
	swSelectData[4] = ch9;
	swSelectData[5] = ch10;
}

void decordReceiveData() {
	if (buf[0] == 0x0F) {
		CH[0] = (buf[1] >> 0 | (buf[2] << 8)) & 0x07FF;
		CH[1] = (buf[2] >> 3 | (buf[3] << 5)) & 0x07FF;
		CH[2] = (buf[3] >> 6 | (buf[4] << 2) | buf[5] << 10) & 0x07FF;
		CH[3] = (buf[5] >> 1 | (buf[6] << 7)) & 0x07FF;
		CH[4] = (buf[6] >> 4 | (buf[7] << 4)) & 0x07FF;
		CH[5] = (buf[7] >> 7 | (buf[8] << 1) | buf[9] << 9) & 0x07FF;
		CH[6] = (buf[9] >> 2 | (buf[10] << 6)) & 0x07FF;
		CH[7] = (buf[10] >> 5 | (buf[11] << 3)) & 0x07FF;
		CH[8] = (buf[12] << 0 | (buf[13] << 8)) & 0x07FF;
		CH[9] = (buf[13] >> 3 | (buf[14] << 5)) & 0x07FF;
		CH[10] = (buf[14] >> 6 | (buf[15] << 2) | buf[16] << 10) & 0x07FF;
		CH[11] = (buf[16] >> 1 | (buf[17] << 7)) & 0x07FF;
		CH[12] = (buf[17] >> 4 | (buf[18] << 4)) & 0x07FF;
		CH[13] = (buf[18] >> 7 | (buf[19] << 1) | buf[20] << 9) & 0x07FF;
		CH[14] = (buf[20] >> 2 | (buf[21] << 6)) & 0x07FF;
		CH[15] = (buf[21] >> 5 | (buf[22] << 3)) & 0x07FF;

		if (buf[23] & (1 << 0)) {
			CH[16] = 1;
		} else {
			CH[16] = 0;
		}
		
		if (buf[23] & (1 << 1)) {
			CH[17] = 1;
		} else {
			CH[17] = 0;
		}

		// Failsafe
		failsafe_status = SBUS_SIGNAL_OK;
		if (buf[23] & (1 << 2)) {
			failsafe_status = SBUS_SIGNAL_LOST;
		}

		if (buf[23] & (1 << 3)) {
			failsafe_status = SBUS_SIGNAL_FAILSAFE;
		}

		//	SBUS_footer=buf[24];
		convertReceiveData();
	}
}


float getStickValue(int ch) {
	return stickValue[ch];
}

float getVolumeValue() {
	return volumeValue;
}

int getSwitchValue(int ch) {
	return switchValue[ch];
}

int getChannelValue(int ch) {
	return CH[ch];
}

int getFailSafe() {
	return failsafe_status;
}





#endif /* INC_SBUS_H_ */
