#ifndef CAN_H
#define CAN_H

#include <vector>
#include <map>
#include <FlexCAN_T4.h>

extern std::vector<bool> is_can1_contact;
extern std::vector<bool> is_can2_contact;
extern std::vector<bool> is_can3_contact;

class CanControl{
	public:
		CanControl(uint8_t _canbus);

		int canbus;
		bool is_can_open;
		
		void init(int baudrate);
		void resetTable();
		void CANDataPull(uint32_t id ,uint8_t data[8]);
		void CANDataPush(uint32_t id ,uint8_t data[8]);
		void MsgStackClear();
		void CANMsgWrite(CAN_message_t msg);
		int8_t CANAllDataWrite();

	private:
		CAN_message_t MSG;
		
		bool _can1 = false;
		bool _can2 = false;
		bool _can3 = false;

		void CANReceiveInterrupt();
		
	protected:
		CanControl(){};
};

#endif