/*
 * sbus.h
 *
 *  Created on: Nov 10, 2023
 *      Author: kobaike
 */

#ifndef SBUS_HPP_
#define SBUS_HPP_

class sbus{
	public:
		sbus();

			enum STICK_NUMBER {
				ANALOG_RX,
				ANALOG_RY,
				ANALOG_LX,
				ANALOG_LY,
				ANALOG_NUMBER,
			};

			enum SWITCH_NUMBER {
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
			};

			enum SWITCH_POSITION {
				LOW,
				NEUTRAL,
				HIGH,
			};

			enum SBUS_STATUS {
				SBUS_SIGNAL_OK,
				SBUS_SIGNAL_LOST,
				SBUS_SIGNAL_FAILSAFE,
			};

			enum SBUS_CH_NUMBER {
				CH5 = 4,
				CH6,
				CH7,
				CH8,
				CH9,
				CH10,
			};

			void setup(int ch5,int ch6,int ch7,int ch8,int ch9,int ch10);

			float getStickValue(int ch);
			float getVolumeValue();
			int getSwitchValue(int ch);

			int getFailSafe();

			int getChannelValue(int ch);
			void decordReceiveData();
			void sbusdefaultSet();
		private:

			bool enableReceive;
			int failsafe_status;
			int stickMaximumValue;
			int stickNeutralValue;
			int stickMinimumValue;
			int stickResolution;
			float volumeValue;
			int CH[16];
			int buf[50];

			int swSelectData[6];

			int switchValue[12];
			float stickValue[ANALOG_NUMBER];


			void serialReceiveHandler();
			void convertReceiveData();
			void resetAllData();
};
#endif /* SBUS_HPP_ */
