#include "RobomasMotor.h"

RobomasMotor::RobomasMotor(CanControl *can_a ,double _cycle){
	this->canforDrive = can_a;
	this->cycle = _cycle / 1000;
	membaInit();
}

void RobomasMotor::membaInit(){
	this->DataM3508.clear();
	this->DataM2006.clear();
    this->DataGM6020.clear();
	this->PCDataM3508.clear();
	this->PCDataM2006.clear();
	this->PCDataGM6020.clear();
	for(int i=1; i<=8; i++) this->PCDataM3508[i].pos_flag = true;
	for(int i=1; i<=8; i++) this->PCDataM2006[i].pos_flag = true;
	for(int i=1; i<=7; i++) this->PCDataGM6020[i].pos_flag = true;
	for(int i=0; i<12; i++) Idpath[i] = NONMOTOR;
	for(int i=0;i<9;i++){
		this->is_useM3508[i] = false;
		this->is_usePositionControlM3508[i] = false;

		this->is_useM2006[i] = false;
		this->is_usePositionControlM2006[i] = false;
	}
	for(int i=0; i<8; i++){
		this->is_useGM6020[i] = false;
		this->is_usePositionControlGM6020[i] = false;
	}
	for(int id=0x201; id<=0x20b; id++){
		if(canforDrive->canbus == 1){
			is_can1_contact[id] = false;
		}
		if(canforDrive->canbus == 2){
			is_can2_contact[id] = false;
		}
		if(canforDrive->canbus == 3){
			is_can3_contact[id] = false;
		}
	}
}

void RobomasMotor::init(){
	canforDrive->init(CAN_BITRATE);
}

void RobomasMotor::setTargetRpmM3508(uint8_t id ,double _rpm){
	if( id <= 0 || id > 8)return;
	_rpm = (_rpm > MAXRPM_M3508) ? MAXRPM_M3508 : _rpm;
	_rpm = (_rpm < -MAXRPM_M3508) ? -MAXRPM_M3508 : _rpm;
	DataM3508[id].target = _rpm;
}

void RobomasMotor::setTargetRpmM2006(uint8_t id ,double _rpm){
	if( id <= 0 || id > 8)return;
	_rpm = (_rpm > MAXRPM_M2006) ? MAXRPM_M2006 : _rpm;
	_rpm = (_rpm < -MAXRPM_M2006) ? -MAXRPM_M2006 : _rpm;
	DataM2006[id].target = _rpm;
}

void RobomasMotor::setTargetRpmGM6020(uint8_t id ,double _rpm){
	if( id <= 0 || id > 7)return;
	_rpm = (_rpm > MAXRPM_GM6020) ? MAXRPM_GM6020 : _rpm;
	_rpm = (_rpm < -MAXRPM_GM6020) ? -MAXRPM_GM6020 : _rpm;
	DataGM6020[id].target = _rpm;
}

void RobomasMotor::setTargetPositionRelativeM3508(uint8_t id, int phase){
	if( id <= 0 || id > 8)return;
	PCDataM3508[id].targetpos = phase;
	PCDataM3508[id].pos_flag = true;
}

void RobomasMotor::setTargetPositionRelativeM2006(uint8_t id, int phase){
	if( id <= 0 || id > 8)return;
	PCDataM2006[id].targetpos = phase;
	PCDataM2006[id].pos_flag = true;
}

void RobomasMotor::setTargetPositionRelativeGM6020(uint8_t id, int phase){
	if( id <= 0 || id > 7)return;
	PCDataGM6020[id].targetpos = phase;
	PCDataGM6020[id].pos_flag = true;
}

void RobomasMotor::setTargetPositionAbsoluteM3508(uint8_t id, int pos){
	if( id <= 0 || id > 8)return;
	PCDataM3508[id].targetpos = pos;
	PCDataM3508[id].pos_flag = false;
}

void RobomasMotor::setTargetPositionAbsoluteM2006(uint8_t id, int pos){
	if( id <= 0 || id > 8)return;
	PCDataM2006[id].targetpos = pos;
	PCDataM2006[id].pos_flag = false;
}

void RobomasMotor::setTargetPositionAbsoluteGM6020(uint8_t id, int pos){
	if( id <= 0 || id > 7)return;
	PCDataGM6020[id].targetpos = pos;
	PCDataGM6020[id].pos_flag = false;
}

void RobomasMotor::setRpmPIDgain(uint8_t type, uint8_t id, PIDGain *gain){
	switch (type){
		case M3508:
			if( id <= 0 || id > 8)return;
			this->Idpath[id] = type;
			this->is_useM3508[id] = true;
			DataM3508[id].Gain = gain;
			break;
		case M2006:
			if( id <= 0 || id > 8)return;
			this->Idpath[id] = type;
			this->is_useM2006[id] = true;
			DataM2006[id].Gain = gain;
			break;
		case GM6020:
			if( id <= 0 || id > 7)return;
			this->Idpath[id+4] = type;
			this->is_useGM6020[id] = true;
			DataGM6020[id].Gain = gain;
			break;
		default:
			break;
	}
}

void RobomasMotor::setPositionPIDgain(uint8_t type, uint8_t id, PIDGain *gain){
	switch (type){
		case M3508:
			if( id <= 0 || id > 8)return;
			this->is_usePositionControlM3508[id] = true;
			PCDataM3508[id].Gain = gain;
			break;
		case M2006:
			if( id <= 0 || id > 8)return;
			this->is_usePositionControlM2006[id] = true;
			PCDataM2006[id].Gain = gain;
			break;
		case GM6020:
			if( id <= 0 || id > 7)return;
			this->is_usePositionControlGM6020[id] = true;
			PCDataGM6020[id].Gain = gain;
			break;
		default:
			break;
	}
}

void RobomasMotor::updateMotorDataM3508(uint8_t id){
	if( id <= 0 || id > 8)return;
	uint8_t dataf[8];
	canforDrive->CANDataPull(0x200+id,dataf);
	DataM3508[id].angle = (dataf[0]<<8) + dataf[1];
	PCDataM3508[id].pos = DataM3508[id].angle;
	DataM3508[id].rpm = (dataf[2]<<8) + dataf[3];
	DataM3508[id].ampare = (dataf[4]<<8) + dataf[5];
	DataM3508[id].temperature = dataf[6];
	phasedifferenceM3508(id);
	return;
}

void RobomasMotor::updateMotorDataM2006(uint8_t id){
	if( id <= 0 || id > 8)return;
	uint8_t dataf[8];
	canforDrive->CANDataPull(0x200+id,dataf);
	DataM2006[id].angle = (dataf[0]<<8) + dataf[1];
	PCDataM2006[id].pos = DataM2006[id].angle;
	DataM2006[id].rpm = (dataf[2]<<8) + dataf[3];
	DataM2006[id].ampare = (dataf[4]<<8) + dataf[5];
	DataM2006[id].temperature = dataf[6];
	phasedifferenceM2006(id);
	return;
}

void RobomasMotor::updateMotorDataGM6020(uint8_t id){
	if( id <= 0 || id > 7)return;
	uint8_t dataf[8];
	canforDrive->CANDataPull(0x204+id,dataf);
    DataGM6020[id].angle = (dataf[0]<<8) + dataf[1];
	PCDataGM6020[id].pos = DataGM6020[id].angle;
	DataGM6020[id].rpm = (dataf[2]<<8) + dataf[3];
	DataGM6020[id].ampare = (dataf[4]<<8) + dataf[5];
	DataGM6020[id].temperature = dataf[6];
	phasedifferenceGM6020(id);
	return;
}

void RobomasMotor::phasedifferenceM3508(uint8_t id){
	if(PCDataM3508[id].pos_flag != true){
		int16_t diff;
        diff = PCDataM3508[id].pos - PCDataM3508[id].prepos;
        if (diff < -(RESOLUTION / 2)){
          diff += RESOLUTION;
        }else if ((RESOLUTION / 2) < diff){
          diff -= RESOLUTION;
        }
        PCDataM3508[id].phase += diff;
	}else{
		PCDataM3508[id].phase = 0;
		PCDataM3508[id].pos_flag = false;
	}
	PCDataM3508[id].prepos = PCDataM3508[id].pos;
}

void RobomasMotor::phasedifferenceM2006(uint8_t id){
	if(PCDataM2006[id].pos_flag != true){
		int16_t diff;
        diff = PCDataM2006[id].pos - PCDataM2006[id].prepos;
        if (diff < -(RESOLUTION / 2)){
          diff += RESOLUTION;
        }else if ((RESOLUTION / 2) < diff){
          diff -= RESOLUTION;
        }
        PCDataM2006[id].phase += diff;
	}else{
		PCDataM2006[id].phase = 0;
		PCDataM2006[id].pos_flag = false;
	}
	PCDataM2006[id].prepos = PCDataM2006[id].pos;
}

void RobomasMotor::phasedifferenceGM6020(uint8_t id){
	if(PCDataGM6020[id].pos_flag != true){
		int16_t diff;
        diff = PCDataGM6020[id].pos - PCDataGM6020[id].prepos;
        if (diff < -(RESOLUTION / 2)){
          diff += RESOLUTION;
        }else if ((RESOLUTION / 2) < diff){
          diff -= RESOLUTION;
        }
        PCDataGM6020[id].phase += diff;
	}else{
		PCDataGM6020[id].phase = 0;
		PCDataGM6020[id].pos_flag = false;

	}
	PCDataGM6020[id].prepos = PCDataGM6020[id].pos;
}

bool RobomasMotor::check_is_contact(uint16_t canID){
	if(canforDrive->canbus == 1){
		if(is_can1_contact[canID] == false)return false;
		is_can1_contact[canID] = false;
	}
	if(canforDrive->canbus == 2){
		if(is_can2_contact[canID] == false)return false;
		is_can2_contact[canID] = false;
	}
	if(canforDrive->canbus == 3){
		if(is_can3_contact[canID] == false)return false;
		is_can3_contact[canID] = false;
	}
	return true;
}

void RobomasMotor::calculate(){
	for(int i=1; i<=11; i++){
		switch(this->Idpath[i]){
			case NONMOTOR:
				break;
			case M3508:
				if(is_usePositionControlM3508[i]){
					double integral = (PCDataM3508[i].targetpos - PCDataM3508[i].phase + PCDataM3508[i].preventdiff) / 2 * this->cycle;
					double targeterror = PCDataM3508[i].targetpos - PCDataM3508[i].phase;
					double calc_targetrpm = PCDataM3508[i].Gain->kp*targeterror + PCDataM3508[i].Gain->ki*integral + PCDataM3508[i].Gain->kd*(targeterror - PCDataM3508[i].preventdiff);
					if(calc_targetrpm >= MAXRPM_M3508) calc_targetrpm = MAXRPM_M3508;
					if(calc_targetrpm <= -MAXRPM_M3508) calc_targetrpm = -MAXRPM_M3508;
					this->setTargetRpmM3508(i,Map(calc_targetrpm, -MAXRPM_M3508, MAXRPM_M3508, -MAXRPM_M3508/5, MAXRPM_M3508/5));
				}
				break;
			case M2006:
				if(is_usePositionControlM2006[i]){
					double integral = (PCDataM2006[i].targetpos - PCDataM2006[i].phase + PCDataM2006[i].preventdiff) / 2 * this->cycle;
					double targeterror = PCDataM2006[i].targetpos - PCDataM2006[i].phase;
					double calc_targetrpm = PCDataM2006[i].Gain->kp*targeterror + PCDataM2006[i].Gain->ki*integral + PCDataM2006[i].Gain->kd*(targeterror - PCDataM2006[i].preventdiff);
					if(calc_targetrpm >= MAXRPM_M2006) calc_targetrpm = MAXRPM_M2006;
					if(calc_targetrpm <= -MAXRPM_M2006) calc_targetrpm = -MAXRPM_M2006;
					this->setTargetRpmM2006(i,Map(calc_targetrpm, -MAXRPM_M2006, MAXRPM_M2006, -MAXRPM_M2006/5, MAXRPM_M2006/5));
				}
				break;
			case GM6020:
				if(is_usePositionControlGM6020[i]){
					uint8_t id = i - 4;
					double integral = (PCDataGM6020[id].targetpos - PCDataGM6020[id].phase + PCDataGM6020[id].preventdiff) / 2 * this->cycle;
					double targeterror = PCDataGM6020[id].targetpos - PCDataGM6020[id].phase;
					double calc_targetrpm = PCDataGM6020[id].Gain->kp*targeterror + PCDataGM6020[id].Gain->ki*integral + PCDataGM6020[id].Gain->kd*(targeterror - PCDataGM6020[id].preventdiff);
					if(calc_targetrpm >= MAXRPM_GM6020) calc_targetrpm = MAXRPM_GM6020;
					if(calc_targetrpm <= -MAXRPM_GM6020) calc_targetrpm = -MAXRPM_GM6020;
					this->setTargetRpmGM6020(id,Map(calc_targetrpm, -MAXRPM_GM6020, MAXRPM_GM6020, -MAXRPM_GM6020/5, MAXRPM_GM6020/5));
				}
				break;
			default:
				break;
		}
	}
	static int diff[12] = {};
	static double integral[12];
	static double targeterror[12];
	for(int i=1; i<=11; i++){
		switch(this->Idpath[i]){
			case NONMOTOR:
				break;
			case M3508:
				if(is_useM3508[i]){
					if(!check_is_contact(0x200+i)) continue;
					updateMotorDataM3508(i);
					targeterror[i] = DataM3508[i].target - DataM3508[i].rpm;
					integral[i] += (targeterror[i] + diff[i]) / 2 * this->cycle;
					this->DataM3508[i].order_ampare = DataM3508[i].Gain->kp*targeterror[i] + DataM3508[i].Gain->ki*integral[i] + DataM3508[i].Gain->kd*(targeterror[i] - diff[i]);
					if(this->DataM3508[i].order_ampare >= MAXANPARE_M3508) this->DataM3508[i].order_ampare = MAXANPARE_M3508;
					if(this->DataM3508[i].order_ampare <= -MAXANPARE_M3508) this->DataM3508[i].order_ampare = -MAXANPARE_M3508;
					diff[i] = targeterror[i];
				}
				break;
			case M2006:
				if(is_useM2006[i]){
					if(!check_is_contact(0x200+i)) continue;
					updateMotorDataM2006(i);
					targeterror[i] = DataM2006[i].target - DataM2006[i].rpm;
					integral[i] += (targeterror[i] + diff[i]) / 2 * this->cycle;
					this->DataM2006[i].order_ampare = DataM2006[i].Gain->kp*targeterror[i] + DataM2006[i].Gain->ki*integral[i] + DataM2006[i].Gain->kd*(targeterror[i] - diff[i]);
					if(this->DataM2006[i].order_ampare >= MAXANPARE_M2006) this->DataM2006[i].order_ampare = MAXANPARE_M2006;
					if(this->DataM2006[i].order_ampare <= -MAXANPARE_M2006) this->DataM2006[i].order_ampare = -MAXANPARE_M2006;
					diff[i] = targeterror[i];
				}
				break;
			case GM6020:
				if(is_useGM6020[i]){
					uint8_t id = i - 4;
					if(!check_is_contact(0x204+id)) continue;
					updateMotorDataGM6020(id);
					targeterror[id] = DataGM6020[id].target - DataGM6020[id].rpm;
					integral[id] += (targeterror[id] + diff[id]) / 2 * this->cycle;
					DataGM6020[id].order_voltage = DataGM6020[id].Gain->kp*targeterror[id] + DataGM6020[id].Gain->ki*integral[id] + DataGM6020[id].Gain->kd*(targeterror[id] - diff[id]);
					if(DataGM6020[id].order_voltage >= MAXVOLTAGE_GM6020) DataGM6020[id].order_voltage = MAXVOLTAGE_GM6020;
					if(DataGM6020[id].order_voltage <= -MAXVOLTAGE_GM6020) DataGM6020[id].order_voltage = -MAXVOLTAGE_GM6020;
					diff[id] = targeterror[id];
				}
				break;
			default:
				break;
		}
	}
}

void RobomasMotor::Control(){
	this->calculate();
	static CAN_message_t msg1;
	static CAN_message_t msg2;
	static CAN_message_t msg3;

	bool is_msg1_data = false;
	bool is_msg2_data = false;
	bool is_msg3_data = false;

	msg1.id = 0x1FF;//M3508,M2006(ID:5,6,7,8) GM6020(ID:1,2,3,4)
	msg2.id = 0x200;//M3508,M2006(ID:1,2,3,4)
	msg3.id = 0x2FF;//GM6020(ID:5,6,7)
	
	msg1.len = 8;
	msg2.len = 8;
	msg3.len = 8;

	for(int i=0;i<8;i++){
		msg1.buf[i] = 0;
		msg2.buf[i] = 0;
		msg3.buf[i] = 0;
	}
	
	for(int i=1;i<=11;i++){
		switch(this->Idpath[i]){
			case NONMOTOR:
				break;
			case M3508:
				if(i <= 4){
					is_msg2_data = true;
					msg2.buf[(i-1)*2] = ((int16_t)this->DataM3508[i].order_ampare) >> 8;
					msg2.buf[(i-1)*2+1] = ((int16_t)this->DataM3508[i].order_ampare) & 0xFF;
				}else if(i <= 8){
					is_msg1_data = true;
					msg1.buf[(i-5)*2] = ((int16_t)this->DataM3508[i].order_ampare) >> 8;
					msg1.buf[(i-5)*2+1] = ((int16_t)this->DataM3508[i].order_ampare) & 0xFF;					
				}
				break;
			case M2006:
				if(i <= 4){
					is_msg2_data = true;
					msg2.buf[(i-1)*2] = ((int16_t)this->DataM2006[i].order_ampare) >> 8;
					msg2.buf[(i-1)*2+1] = ((int16_t)this->DataM2006[i].order_ampare) & 0xFF;
				}else if(i <= 8){
					is_msg1_data = true;
					msg1.buf[(i-5)*2] = ((int16_t)this->DataM2006[i].order_ampare) >> 8;
					msg1.buf[(i-5)*2+1] = ((int16_t)this->DataM2006[i].order_ampare) & 0xFF;
				}
				break;
			case GM6020:
				uint8_t id = i - 4; 
				if(id <= 4){
					is_msg1_data = true;
					msg1.buf[(id-1)*2] = ((int16_t)this->DataGM6020[id].order_voltage) >> 8;
					msg1.buf[(id-1)*2+1] = ((int16_t)this->DataGM6020[id].order_voltage) & 0xFF;
				}else if(i <= 7){
				is_msg3_data = true;
					msg3.buf[(id-5)*2] = ((int16_t)this->DataGM6020[id].order_voltage) >> 8;
					msg3.buf[(id-5)*2+1] = ((int16_t)this->DataGM6020[id].order_voltage) & 0xFF;
				}
				break;
			default:
				break;
		}
	}
	if(is_msg1_data)canforDrive->CANMsgWrite(msg1);
	if(is_msg2_data)canforDrive->CANMsgWrite(msg2);
	if(is_msg3_data)canforDrive->CANMsgWrite(msg3);
}

double RobomasMotor::Map(double num, double from_min, double from_max, double to_min, double to_max){

	return (num - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

void RobomasMotor::dispMotorDataM3508(uint8_t id){
	Serial.print(id);
	Serial.print(" angle:");
	Serial.print(DataM3508[id].angle);
	Serial.print("\trpm:");
	Serial.print(DataM3508[id].rpm);
	Serial.print("\tampare:");
	Serial.print(DataM3508[id].ampare);
	Serial.print("\ttemperature:");
	Serial.print(DataM3508[id].temperature);
	Serial.print("\torder_ampare:");
	Serial.print(DataM3508[id].order_ampare);
	Serial.print("\tkp:");
	Serial.print(DataM3508[id].Gain->kp);
	Serial.print("\tki:");
	Serial.print(DataM3508[id].Gain->ki);
	Serial.print("\tkd:");
	Serial.println(DataM3508[id].Gain->kd);
}

void RobomasMotor::dispMotorDataM2006(uint8_t id){
	Serial.print(id);
	Serial.print(" angle:");
	Serial.print(DataM2006[id].angle);
	Serial.print("\trpm:");
	Serial.print(DataM2006[id].rpm);
	Serial.print("\tampare:");
	Serial.print(DataM2006[id].ampare);
	Serial.print("\ttemperature:");
	Serial.print(DataM2006[id].temperature);
	Serial.print("\torder_ampare:");
	Serial.print(DataM2006[id].order_ampare);
	Serial.print("\tkp:");
	Serial.print(DataM2006[id].Gain->kp);
	Serial.print("\tki:");
	Serial.print(DataM2006[id].Gain->ki);
	Serial.print("\tkd:");
	Serial.println(DataM2006[id].Gain->kd);
}

void RobomasMotor::dispMotorDataGM6020(uint8_t id){
	Serial.print(id);
	Serial.print(" angle:");
	Serial.print(DataGM6020[id].angle);
	Serial.print("\trpm:");
	Serial.print(DataGM6020[id].rpm);
	Serial.print("\tampare:");
	Serial.print(DataGM6020[id].ampare);
	Serial.print("\ttemperature:");
	Serial.print(DataGM6020[id].temperature);
	Serial.print("\torder_voltage:");
	Serial.print(DataGM6020[id].order_voltage);
	Serial.print("\tkp:");
	Serial.print(DataGM6020[id].Gain->kp);
	Serial.print("\tki:");
	Serial.print(DataGM6020[id].Gain->ki);
	Serial.print("\tkd:");
	Serial.println(DataGM6020[id].Gain->kd);
}

void RobomasMotor::dispMotorPositionControlDataM3508(uint8_t id){
	Serial.print(id);
	Serial.print(" prepos:");
	Serial.print(PCDataM3508[id].prepos);
	Serial.print("\tpos:");
	Serial.print(PCDataM3508[id].pos);
	Serial.print("\tpos_flag:");
	Serial.print(PCDataM3508[id].pos_flag);
	Serial.print("\tphase:");
	Serial.print(PCDataM3508[id].phase);
	Serial.print("\ttargetpos:");
	Serial.print(PCDataM3508[id].targetpos);
	Serial.print("\tpreventdiff:");
	Serial.print(PCDataM3508[id].preventdiff);
	Serial.print("\tkp:");
	Serial.print(PCDataM3508[id].Gain->kp);
	Serial.print("\tki:");
	Serial.print(PCDataM3508[id].Gain->ki);
	Serial.print("\tkd:");
	Serial.println(PCDataM3508[id].Gain->kd);
}

void RobomasMotor::dispMotorPositionControlDataM2006(uint8_t id){
	Serial.print(id);
	Serial.print(" prepos:");
	Serial.print(PCDataM2006[id].prepos);
	Serial.print("\tpos:");
	Serial.print(PCDataM2006[id].pos);
	Serial.print("\tpos_flag:");
	Serial.print(PCDataM2006[id].pos_flag);
	Serial.print("\tphase:");
	Serial.print(PCDataM2006[id].phase);
	Serial.print("\ttargetpos:");
	Serial.print(PCDataM2006[id].targetpos);
	Serial.print("\tpreventdiff:");
	Serial.print(PCDataM2006[id].preventdiff);
	Serial.print("\tkp:");
	Serial.print(PCDataM2006[id].Gain->kp);
	Serial.print("\tki:");
	Serial.print(PCDataM2006[id].Gain->ki);
	Serial.print("\tkd:");
	Serial.println(PCDataM2006[id].Gain->kd);
}

void RobomasMotor::dispMotorPositionControlDataGM6020(uint8_t id){
	Serial.print(id);
	Serial.print(" prepos:");
	Serial.print(PCDataGM6020[id].prepos);
	Serial.print("\tpos:");
	Serial.print(PCDataGM6020[id].pos);
	Serial.print("\tpos_flag:");
	Serial.print(PCDataGM6020[id].pos_flag);
	Serial.print("\tphase:");
	Serial.print(PCDataGM6020[id].phase);
	Serial.print("\ttargetpos:");
	Serial.print(PCDataGM6020[id].targetpos);
	Serial.print("\tpreventdiff:");
	Serial.print(PCDataGM6020[id].preventdiff);
	Serial.print("\tkp:");
	Serial.print(PCDataGM6020[id].Gain->kp);
	Serial.print("\tki:");
	Serial.print(PCDataGM6020[id].Gain->ki);
	Serial.print("\tkd:");
	Serial.println(PCDataGM6020[id].Gain->kd);
}

void RobomasMotor::dispUsingMotorM3508(){
	Serial.print("using MotorM3508\t");
	for(int i=1; i<=8; i++) Serial.print(is_useM3508[i]);Serial.print(" ");
	Serial.println();
}

void RobomasMotor::dispUsingMotorM2006(){
	Serial.print("using MotorM2006\t");
	for(int i=1; i<=8; i++) Serial.print(is_useM2006[i]);Serial.print(" ");
	Serial.println();
}

void RobomasMotor::dispUsingMotorGM6020(){
	Serial.print("using MotorGM6020\t");
	for(int i=1; i<=7; i++) Serial.print(is_useGM6020[i]);Serial.print(" ");
	Serial.println();
}

int16_t RobomasMotor::getRpmM3508(uint8_t id){
	return DataM3508[id].rpm;
}

int16_t RobomasMotor::getRpmM2006(uint8_t id){
	return DataM2006[id].rpm;
}

int16_t RobomasMotor::getRpmGM6020(uint8_t id){
	return DataGM6020[id].rpm;
}

int16_t RobomasMotor::getAngleM3508(uint8_t id){
	return DataM3508[id].angle;
}

int16_t RobomasMotor::getAngleM2006(uint8_t id){
	return DataM2006[id].angle;
}

int16_t RobomasMotor::getAngleGM6020(uint8_t id){
	return DataGM6020[id].angle;
}

int64_t RobomasMotor::getPhaseM3508(uint8_t id){
	return PCDataM3508[id].phase;
}

int64_t RobomasMotor::getPhaseM2006(uint8_t id){
	return PCDataM2006[id].phase;
}

int64_t RobomasMotor::getPhaseGM6020(uint8_t id){
	return PCDataGM6020[id].phase;
}
