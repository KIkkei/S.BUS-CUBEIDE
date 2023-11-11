/*
 * sbus.cpp
 *
 *  Created on: Nov 10, 2023
 *      Author: kobaike
 */

#include <sbus.hpp>

	sbus::sbus(){
		stickMaximumValue = 0x0690;
		stickNeutralValue = 0x0400;
		stickMinimumValue = 0x0170;
		stickResolution   = stickMaximumValue - stickMinimumValue;

		enableReceive = false;
		failsafe_status = SBUS_SIGNAL_FAILSAFE;

		resetAllData();
	}

	void sbus::setup(int ch5,int ch6,int ch7,int ch8,int ch9,int ch10) {
		swSelectData[0] = ch5;
		swSelectData[1] = ch6;
		swSelectData[2] = ch7;
		swSelectData[3] = ch8;
		swSelectData[4] = ch9;
		swSelectData[5] = ch10;
	}

	void sbus::decordReceiveData() {
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

	void sbus::convertReceiveData() {
		for(int i=ANALOG_RX;i<=ANALOG_LY;i++) {
			float buf = 0.0f;
			if(CH[i] > (stickNeutralValue + 20)) buf = ((CH[i] - stickNeutralValue) / (float)(stickMaximumValue - stickNeutralValue));
			else if(CH[i] < (stickNeutralValue - 20)) buf = -((CH[i] - stickNeutralValue) / (float)(stickMinimumValue - stickNeutralValue));
			else buf = 0.0f;

			buf = static_cast<float>((buf*100)/100.0f);
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

	void sbus::resetAllData() {
		for(int i = 0; i < 16; i++) CH[i] = 0;
		for(int i = 0; i < 50; i++) buf[i] = 0;
		for(int i = 0; i < 4; i++) stickValue[i] = 0.0f;
		for(int i = 0; i < 12; i++) switchValue[i] = NEUTRAL;
		volumeValue = 0.0f;
	}

	float sbus::getStickValue(int ch) {
		return stickValue[ch];
	}

	float sbus::getVolumeValue() {
		return volumeValue;
	}

	int sbus::getSwitchValue(int ch) {
		return switchValue[ch];
	}

	int sbus::getChannelValue(int ch) {
		return CH[ch];
	}

	int sbus::getFailSafe() {
		return failsafe_status;
	}


