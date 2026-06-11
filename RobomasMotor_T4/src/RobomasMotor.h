/****************************************************************************************************************************************/
/* RobomasMotorライブラリはRobomasモーターを制御するためのライブラリです．
*  ユーザーからCANの存在を意識させないような構造になっており，メインコードではCANのライブラリをincludeする必要はありません．
*  ロボマスモーター以外でもCANバスを使う場合はメインコードでCANのオブジェクトを生成し，RobomasMotorのコンストラクタにアドレスを渡してください．
*  
*  IDを指定すればモーターのRPM，角度，電流，温度を取得することができます．
*  速度制御を実装しており,PIDのパラメータを設定し目標速度を指定すると,任意の速度で回転させることができます．
*  角度制御は未実装ですので誰かやってください
*  (追記)
*  GM6020用の角度制御を実装しました。(久末晃平)
*  (追追記)
*  PID計算のタイマー割込み、CANの受信割り込みの使用により制御精度が向上しました。(久末晃平)
*  M3508,GM6020,M2006の角度制御(位置制御)を実装しました。<-制御可能範囲が無くなりました。(松岡青空)
*  
*/
/****************************************************************************************************************************************/
#ifndef _ROBOMASMOTOR_H_
#define _ROBOMASMOTOR_H_

#include "CAN.h"
#include <map>

#define NONMOTOR 0
#define M3508 1
#define M2006 2
#define GM6020 3

#define M3508M2006 1

const double CAN_BITRATE = 1e6;			//ロボマスモーター（ESC）の初期レート
#define MAXRPM_M3508 10000				//M3508の最大目標RPM
#define MAXRPM_GM6020 526*8 			//GM6020の最大目標RPM
#define MAXRPM_M2006 10000				//M2006の最大目標RPM
#define MAXANPARE_M3508 16384			//M3508の最大出力電流
#define MAXVOLTAGE_GM6020 30000			//GM6020の最大出力電圧
#define MAXANPARE_M2006 10000			//M2006の最大出力電流

#define RESOLUTION 8192					//ロボマスモーター内部のエンコーダの１回転あたりの分解能
#define GEARRATE (1.0/36)
#define tPI (4.0 * acos(0.0))

typedef struct{
	float kp;
	float ki;
	float kd;
}PIDGain;					//モータに送る電流量（電圧）を決める際のPID、位置制御（角度制御）をする際の目標RPMを決定するPIDの設定用の構造体

typedef struct{
	float gains[4];			//LQR制御のゲイン
}LQRGain;					//モータに送る電流量（電圧）を決める際のPID、位置制御（角度制御）をする際の目標RPMを決定するPIDの設定用の構造体

typedef struct{       //必要なセンサーデータの種類や数に応じて追加する
	double senser1;			//センサー1の値
  	double senser2;			//センサー2の値
}SenserData;					//センサーデータをまとめている構造体

typedef struct{
	int16_t angle;			//モータ内部にあるエンコーダの機械的角度
	int16_t rpm;			//モータの回転速度
	int16_t ampare;			//モータに流れるトルク電流
	int16_t temperature;	//モータの温度
	double order_ampare;	//モータに流したいトルク電流
	double target;			//目標回転速度
	PIDGain *Gain;			//回転数制御を行う上で使用するPIDゲイン
}MotorData;					//M3508,M2006のモータ個々のデータをまとめている構造体

typedef struct{
	int16_t angle;			//モータ内部にあるエンコーダの機械的角度
	int16_t rpm;			//モータの回転速度
	int16_t ampare;			//モータに流れるトルク電流
	int16_t temperature;	//モータの温度
	double order_voltage;	//モータに流したい電圧値
	double target;			//目標回転速度
	PIDGain *Gain;			//回転数制御を行う上で使用するPIDゲイン
}MotorDataGM6020;			//GM6020のモータ個々のデータをまとめている構造体

typedef struct{
	int16_t prepos;
	int16_t pos;
	bool pos_flag;
	int64_t phase;
	int64_t targetpos;
	int preventdiff;
	PIDGain *Gain;			//角度制御(位置制御)を行う上で使用するPIDゲイン
}MotorPositionControlData;	//M3508,M2006,GM6020の角度制御(位置制御)に必要なモータ個々のデータをまとめている構造体

typedef struct{
	double mot_difang;       	//モータの角速度
  	double mot_absang;		//モータの絶対角度
	double gyro_difang;		//ジャイロの角速度
  	double gyro_absang;	//ジャイロの絶対角度
  	double iTQ;				//モータに流したいトルク電流
	LQRGain *Gain;			//回転数制御を行う上で使用するLQRゲイン
  	SenserData *Senser;		//センサーデータのポインタ
}MotorLQRData;				//M3508,M2006のモータ個々のデータをまとめている構造体

class RobomasMotor : public CanControl{
	public:
		
		RobomasMotor(CanControl *can ,double _cycle); //CanContorlクラスのアドレス，制御周期(ms)

		void init(); //初期化用関数

		void setTargetRpmM3508(uint8_t id, double _rpm); 		//指定したIDのM3508の目標回転速度の設定
		void setTargetRpmM2006(uint8_t id, double _rpm);		//指定したIDのM2006の目標回転速度の設定
		void setTargetRpmGM6020(uint8_t id, double _rpm); 		//指定したIDのGM6020の目標回転速度の設定

		//相対的角度制御(相対的位置制御)：基準位置から指定した角度（距離）へモーターを動かす制御
		//絶対的角度制御(絶対的位置制御)：起動した時の角度（位置）から指定した角度分（距離分）モーターを動かす制御
		void setTargetPositionRelativeM3508(uint8_t id, int phase);		//指定したIDのM3508の相対的角度制御(相対的位置制御)の目標角度(目標位置)の設定
		void setTargetPositionRelativeM2006(uint8_t id, int phase);		//指定したIDのM2006の相対的角度制御(相対的位置制御)の目標角度(目標位置)の設定
		void setTargetPositionRelativeGM6020(uint8_t id, int phase);	//指定したIDのGM6020の相対的角度制御(相対的位置制御)の目標角度(目標位置)の設定
		void setTargetPositionAbsoluteM3508(uint8_t id, int pos);		//指定したIDのM3508の絶対的角度制御(絶対的位置制御)の目標角度(目標位置)の設定
		void setTargetPositionAbsoluteM2006(uint8_t id, int pos);		//指定したIDのM2006の絶対的角度制御(絶対的位置制御)の目標角度(目標位置)の設定
		void setTargetPositionAbsoluteGM6020(uint8_t id, int pos);		//指定したIDのGM6020の絶対的角度制御(絶対的位置制御)の目標角度(目標位置)の設定

		void setSenserDataM2006(uint8_t id, SenserData *senser);		//指定したIDのM2006のセンサーデータの設定

		void setRpmPIDgain(uint8_t type, uint8_t id, PIDGain *gain);
		void setPositionPIDgain(uint8_t type, uint8_t id, PIDGain *gain);
		void setLQRgain(uint8_t type, uint8_t id, LQRGain *gain);

		void calculate();
		// void calculateRpm();

		void  Control();	//モータにデータを送る

		int16_t getRpmM3508(uint8_t id);		//指定したIDのM3508の回転数を受け取れる
		int16_t getRpmM2006(uint8_t id);		//指定したIDのM2006の回転数を受け取れる
		int16_t getRpmGM6020(uint8_t id);		//指定したIDのGM6020の回転数を受け取れる
		int16_t getAngleM3508(uint8_t id);		//指定したIDのM3508のモータ内部エンコーダでの機械的角度を受け取れる
		int16_t getAngleM2006(uint8_t id);		//指定したIDのM2006のモータ内部エンコーダでの機械的角度を受け取れる
		int16_t getAngleGM6020(uint8_t id);		//指定したIDのGM6020のモータ内部エンコーダでの機械的角度を受け取れる
		int64_t getPhaseM3508(uint8_t id);		//指定したIDのM3508の初期位置からの変位量(一回転当たり8192)を受け取れる
		int64_t getPhaseM2006(uint8_t id);		//指定したIDのM2006の初期位置からの変位量(一回転当たり8192)を受け取れる
		int64_t getPhaseGM6020(uint8_t id);		//指定したIDのGM6020の初期位置からの変位量(一回転当たり8192)を受け取れる

/////////////////////display data/////////////////////////////////////////////////////////
		void dispMotorDataM3508(uint8_t id);	//指定したIDのM3508のデータを表示する
		void dispMotorDataM2006(uint8_t id);	//指定したIDのM2006のデータを表示する
		void dispMotorDataGM6020(uint8_t id);	//指定したIDのGM6020のデータを表示する

		void dispMotorPositionControlDataM3508(uint8_t id);		//指定したIDのM3508の角度制御(位置制御)データを表示する
		void dispMotorPositionControlDataM2006(uint8_t id);		//指定したIDのM2006の角度制御(位置制御)データを表示する
		void dispMotorPositionControlDataGM6020(uint8_t id);	//指定したIDのGM6020の角度制御(位置制御)データを表示する
		
		void dispUsingMotorM3508();				//M3508の使用されているモータを表示する
		void dispUsingMotorM2006();				//M2006の使用されているモータを表示する
		void dispUsingMotorGM6020();			//GM6020の使用されているモータを表示する

		void dispUsingLQRMotorM2006();	//M2006のLQR制御を使用しているモータを表示する
//////////////////////////////////////////////////////////////////////////////////////////

	private:
		CanControl* canforDrive;		//CANのオブジェクトアドレス

		std::map<uint8_t,MotorData> DataM3508;		//モータの速度制御データを辞書型にまとめている M3508用
		std::map<uint8_t,MotorData> DataM2006;		//モータの速度制御データを辞書型にまとめている M2006用
		std::map<uint8_t,MotorDataGM6020> DataGM6020;	//モータの速度制御データを辞書型にまとめている GM6020用
		std::map<uint8_t,MotorPositionControlData> PCDataM3508;		//モータの角度制御(位置制御)のデータを辞書型にまとめている M3508用
		std::map<uint8_t,MotorPositionControlData> PCDataM2006;		//モータの角度制御(位置制御)のデータを辞書型にまとめている M2006用
		std::map<uint8_t,MotorPositionControlData> PCDataGM6020;	//モータの角度制御(位置制御)のデータを辞書型にまとめている GM6020用
		std::map<uint8_t,MotorLQRData> LQRDataM2006;		//モータの速度制御データを辞書型にまとめている M2006用

		bool is_useM3508[9];					//M3508の使用フラグ
		bool is_useM2006[9];					//M2006の使用フラグ
		bool is_useGM6020[8];					//GM6020の使用フラグ
		bool is_usePositionControlM3508[9];		//M3508の角度制御(位置制御)　使用フラグ
		bool is_usePositionControlM2006[9];		//M2006の角度制御(位置制御)　使用フラグ
		bool is_usePositionControlGM6020[8];	//GM6020の角度制御(位置制御)　使用フラグ
		bool is_useLQRControlM2006[9];			//M2006のLQR制御使用フラグ

		double cycle;			//制御周期

		uint8_t Idpath[12];

		void membaInit();			//変数初期化関数

		void updateMotorDataM3508(uint8_t id); 		//M3508のデータを更新
		void updateMotorDataM2006(uint8_t id); 		//M2006のデータを更新
		void updateMotorDataGM6020(uint8_t id); 	//GM6020のデータを更新

		void phasedifferenceM3508(uint8_t id);		//M3508の回転による変位を計算
		void phasedifferenceM2006(uint8_t id);		//M2006の回転による変位を計算
		void phasedifferenceGM6020(uint8_t id);		//GM6020の回転による変位を計算

		bool check_is_contact(uint16_t canID); //canID : CANのアドレス(例:0x201)
		
		double Map(double num, double from_min, double from_max, double to_min, double to_max);
};

#endif