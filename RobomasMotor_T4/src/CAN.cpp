#include <CAN.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

std::map<uint32_t ,CAN_message_t> buffer_table1;
std::vector<CAN_message_t> msg_stack1;
std::map<uint32_t ,CAN_message_t> buffer_table2;
std::vector<CAN_message_t> msg_stack2;
std::map<uint32_t ,CAN_message_t> buffer_table3;
std::vector<CAN_message_t> msg_stack3;

std::vector<bool> is_can1_contact;
std::vector<bool> is_can2_contact;
std::vector<bool> is_can3_contact;

void canSniff1(const CAN_message_t &msg){
	is_can1_contact[msg.id] = true;
	buffer_table1[msg.id] = msg;
}

void canSniff2(const CAN_message_t &msg){
	is_can2_contact[msg.id] = true;
	buffer_table2[msg.id] = msg;
}

void canSniff3(const CAN_message_t &msg){
	is_can3_contact[msg.id] = true;
	buffer_table3[msg.id] = msg;
}

CanControl::CanControl(uint8_t _canbus){
	if(_canbus == 1){
		_can1 = true;
	}else if(_canbus == 2){
		_can2 = true;
	}else if(_canbus == 3){
		_can3 = true;
	}
	canbus = _canbus;
	is_can_open = false;
}


void CanControl::init(int baudrate){
	is_can_open = true;
	if(_can1){
		Can1.begin();
		Can1.setBaudRate(baudrate);
		Can1.setMaxMB(16);
  		Can1.enableFIFO();
  		Can1.enableFIFOInterrupt();
  		Can1.onReceive(canSniff1);
  		Can1.mailboxStatus();
	}else if(_can2){
		Can2.begin();
		Can2.setBaudRate(baudrate);
		Can2.setMaxMB(16);
  		Can2.enableFIFO();
  		Can2.enableFIFOInterrupt();
  		Can2.onReceive(canSniff2);
  		Can2.mailboxStatus();
	}else if(_can3){
		Can3.begin();
		Can3.setBaudRate(baudrate);
		Can3.setMaxMB(16);
  		Can3.enableFIFO();
  		Can3.enableFIFOInterrupt();
  		Can3.onReceive(canSniff3);
  		Can3.mailboxStatus();
	}
}

void CanControl::resetTable(){
	if(_can1){
		buffer_table1.clear();
	}else if(_can2){
		buffer_table2.clear();
	}else if(_can3){
		buffer_table3.clear();
	}
}

void CanControl::CANDataPull(uint32_t id ,uint8_t data[8]){
	CAN_message_t msg_;
	if(_can1){
		msg_ = buffer_table1[id];
		for(int i = 0; i<8; i++){
			data[i] = msg_.buf[i];
		}
		return;
	}
	if(_can2){
		msg_ = buffer_table2[id];
		for(int i = 0; i<8; i++){
			data[i] = msg_.buf[i];
		}
		return;
	}
	if(_can3){
		msg_ = buffer_table3[id];
		for(int i = 0; i<8; i++){
			data[i] = msg_.buf[i];
		}
		return;
	}
}

void CanControl::CANDataPush(uint32_t id, uint8_t data[8]){
	MSG.id = id;
	if(_can1){
		memcpy(MSG.buf,data,8);
		msg_stack1.push_back(MSG);
	}else if(_can2){
		memcpy(MSG.buf,data,8);
		msg_stack2.push_back(MSG);
	}else if(_can3){
		memcpy(MSG.buf,data,8);
		msg_stack3.push_back(MSG);
	}
	return;
}

void CanControl::MsgStackClear(){
	msg_stack1.clear();
	msg_stack2.clear();
	msg_stack3.clear();
}

void CanControl::CANMsgWrite(CAN_message_t msg){
	if(!is_can_open)return;
	if(_can1){
		Can1.write(msg);
	}else if(_can2){
		Can2.write(msg);
	}else if(_can3){
		Can3.write(msg);
	}
	return;
}

int8_t CanControl::CANAllDataWrite(){
	if(!is_can_open)return -1;
	int8_t s = 0;
	if(_can1){
		for(CAN_message_t buff : msg_stack1){
			Can1.write(buff);
			s++;
		}
	}else if(_can2){
		for(CAN_message_t buff : msg_stack2){
			Can2.write(buff);
			s++;
		}
	}else if(_can3){
		for(CAN_message_t buff : msg_stack3){
			Can3.write(buff);
			s++;
		}
	}
	msg_stack1.clear();
	msg_stack2.clear();
	msg_stack3.clear();
	return s;
}