#include <pigpio.h>
#include <iostream>
#include <csignal>
#include <cmath>
#include <iomanip>

// D/A converter関連
#define SPI_CHANNEL 0  // CE0 を使用
#define SPI_SPEED 1000000  // 1MHz
#define VREF 5.34  // MCP4922 の基準電圧 (V)


// GPIO settings
#define ENC_A  17  // A 相 (Encoder 1)
#define ENC_B  27  // B 相 (Encoder 1)

#define ENC_2A 22  // A 相 (Encoder 2)
#define ENC_2B 5   // B 相 (Encoder 2)

#define ENC_MA 6   // A 相 (DC motor)
#define ENC_MB 13  // B 相 (DC motor)

#define FR_DCM 26  // DC Forward/Reversed

#define Check 4 // for signal checking

// エンコーダカウント
volatile int encoder_count  = 0; // encoder 1
volatile int encoder_count2 = 0; // encoder 2
volatile int encoder_countm = 0; // encoder of DC motor

// A/B 相の状態（初期化時に設定）
volatile int lastState  = 0; // encoder 1
volatile int lastState2 = 0; // encoder 2
volatile int lastStatem = 0; // encoder of DC motor

// クアドラチャカウントテーブル（標準的な方法）
const int state_table[16] = {0,  1, -1, 0,  
                             -1, 0,  0, 1,  
                              1, 0,  0, -1,  
                              0, -1, 1, 0};

// DMAを使ったサンプル取得のコールバック
void sampleCallback(const gpioSample_t *samples, int numSamples) {
    for (int i = 0; i < numSamples; i++) {
        int level = samples[i].level;

        // --- Encoder 1 ---
        int A1 = (level & (1 << ENC_A)) ? 1 : 0;
        int B1 = (level & (1 << ENC_B)) ? 1 : 0;
        int state1 = (A1 << 1) | B1;
        encoder_count += state_table[(lastState << 2) | state1];
        lastState = state1;

        // --- Encoder 2 ---
        int A2 = (level & (1 << ENC_2A)) ? 1 : 0;
        int B2 = (level & (1 << ENC_2B)) ? 1 : 0;
        int state2 = (A2 << 1) | B2;
        encoder_count2 += state_table[(lastState2 << 2) | state2];
        lastState2 = state2;

        // --- Encoder of DC Motor ---
        int AM = (level & (1 << ENC_MA)) ? 1 : 0;
        int BM = (level & (1 << ENC_MB)) ? 1 : 0;
        int statem = (AM << 1) | BM;
        encoder_countm += state_table[(lastStatem << 2) | statem];
        lastStatem = statem;
    }
}

// Ctrl+C ハンドラー
void signalHandler(int signum) {
    std::cout << "\nStopping encoder reading..." << std::endl;
    gpioSetGetSamplesFunc(nullptr, 0); // DMA停止
    gpioTerminate();
    exit(signum);
}

// Voltage calculation
void setVoltage(int spiHandle, int channel, double voltage) {
    if (voltage < 0.0) voltage = 0.0;
    if (voltage > VREF) voltage = VREF;

//    int data = static_cast<int>(std::round((voltage / VREF) * 4095));
    int data = (int)((voltage / VREF) * 4095);  // 12bit データ (0-4095)
//    int command = (channel == 0) ? 0x1000 : 0x9000; // CH_A: 0x1, CH_B: 0x9

    int command = (channel == 0) ? 0x3000 : 0xB000; // CH0: 0x3, CH1: 0xB
    int value = command | (data & 0x0FFF); // 送信データ

    char buffer[2];
    buffer[0] = (value >> 8) & 0xFF; // 上位バイト
    buffer[1] = value & 0xFF;        // 下位バイト

    // ここを修正
    spiWrite(spiHandle, buffer, 2);
//    std::cout << "Command: "<<command<<", value: "<<value<<", data: "<<data<<std::endl;
//    std::cout << " buf1: " <<buffer[0] << ", buf2: "<<buffer[1]<<std::endl;
}

int main() {
    // pigpio 初期化
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio!" << std::endl;
        return 1;
    }

    // SPI settings
    int spiHandle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if (spiHandle < 0) {
        std::cerr << "SPI オープン失敗!" << std::endl;
        gpioTerminate();
        return 1;
    }

    // GPIO 設定（入力モード）
    gpioSetMode(ENC_A, PI_INPUT);
    gpioSetMode(ENC_B, PI_INPUT);
    
    gpioSetMode(ENC_2A, PI_INPUT);
    gpioSetMode(ENC_2B, PI_INPUT);

    gpioSetMode(ENC_MA, PI_INPUT);
    gpioSetMode(ENC_MB, PI_INPUT);

    gpioSetMode(FR_DCM, PI_OUTPUT);

    gpioSetMode(Check,PI_OUTPUT);

    // **初期状態の取得**
    lastState  = (gpioRead(ENC_A)  << 1) | gpioRead(ENC_B);
    lastState2 = (gpioRead(ENC_2A) << 1) | gpioRead(ENC_2B);
    lastStatem = (gpioRead(ENC_MA) << 1) | gpioRead(ENC_MB);

    // A/B 相の変化を DMA で取得（1回だけ呼び出す）
    gpioSetGetSamplesFunc(sampleCallback, 
        (1 << ENC_A) | (1 << ENC_B) |
        (1 << ENC_2A) | (1 << ENC_2B) |
        (1 << ENC_MA) | (1 << ENC_MB));

    // Ctrl+C ハンドラー設定
    signal(SIGINT, signalHandler);

    std::cout << "Reading encoder with DMA... (Press Ctrl+C to exit)\n";

    double rang1{},rang2{},rangm{};
    double prang1{},prang2{},prangm{};
    double pprang1{},pprangm{};
    double drang1{},drang2{},drangm{};
    double tpi;

    double gains[6]{};
    double iTQ{};

    double Kt{},Kc{},Summ{};
    double Vtg{};
//    int intVtg{};

    tpi = 4.0*acos(0.0);
    
    // Optimized gains
//    gains[0] = -0.3097;
//    gains[1] = -10.39;
//    gains[2] = -0.5575;
//    gains[3] = -1.762;

    //現状最強on250311
//gains[0] = -0.2929;
//gains[1] = -4.119;
//gains[2] = -0.5216;
//gains[3] = -0.4496;

gains[0] = -0.1546;
gains[1] = -7.982;
gains[2] = -0.4626;
gains[3] = -1.183;

    //    gains[0] = 1.7709;
//    gains[1] = -6.7644E+1;
//    gains[2] = 1.1488E+2;
//    gains[3] = 1.8139;
//    gains[4] = -0.47798;
//    gains[5] = 9.7232;

//    Kt = 1.638;
//    Kc = 9.235E-1;
//    Summ = Kt*Kc;

    bool conts;
    int flag{};
    conts = true;

    while (conts) {
        // 角度計算
        double angle  = -(encoder_count  / 4096.0)*tpi;// * 360.0; // Encoder 1
        double angle2 = -(encoder_count2 / 4096.0)*tpi;// * 360.0; // Encoder 2
        double anglem = -(encoder_countm / 4096.0)*tpi/28.0;// * 360.0; // Encoder of DC motor


        std::cout << "\rEnc1: " <<std::setw(10)<< angle << " rad  "
                  << "Enc2: " <<std::setw(10)<< angle2 << " rad  "
                  << "Mot: " <<std::setw(10)<< anglem << " rad  "
                  << "dth1: " <<std::setw(10)<< drang1 << " rad/s "
                  << "dth2: " <<std::setw(10)<< drang2 << " rad/s "
                  << "dth0: " <<std::setw(10)<< drangm << " rad/s "
                  << " Vol: " << Vtg<< " V "
                  << " Torq: " << iTQ<<" Nm "
                  << std::flush;


        gpioDelay(4000); // 5 ms 更新

//        if(Check>0){
//            gpioWrite(Check,0);
//        }else{
//            gpioWrite(Check,1);
//        }

        pprang1 = prang1;
        pprangm = prangm;
        // Prev values
        prang1 = rang1;
//        prang2 = rang2;
        prangm = rangm;

        // Current values
//        rang1 = anglem + angle2;
        rang1 = anglem + angle;
//        rang2 = rang1  + angle2;
        rangm = anglem;

        // Derivatives
        drang1 = (rang1-pprang1)/8.0E-3;
//        drang1 = (rang1-prang1)/5.0E-3;
//        drang2 = (rang2-prang2)/4.0E-3;
        drangm = (rangm - pprangm)/8.0E-3;
//        drangm = (rangm-prangm)/5.0E-3;

        // Optimal input by LQR (torque Nm)
//        iTQ = -angle;
        iTQ = -(gains[0]*rangm + gains[1]*rang1 + gains[2]*drangm + gains[3]*drang1);
//        iTQ *= 3.0E+1;
//        iTQ = -(gains[0]*rangm  + gains[1]*rang1  + gains[2]*rang2 
//              + gains[3]*drangm + gains[4]*drang1 + gains[5]*drang2);

// for N-mode on 2025.3.17

//        Vtg = 0.15*iTQ + 0.273*drangm;


                Vtg = 2.8E+1*0.0518*iTQ + 0.1638*drangm;
//        Vtg *= 2.8E+1;
//        Vtg = iTQ/Summ;
        if(flag == 1){
            Vtg = 0.0;
        }
        // ここのメモ書きは誤り（実験で確認） → 見ている方向が逆なので，ここは反転のはず
        if(Vtg>0){
            gpioWrite(FR_DCM,1);
        }else{
            gpioWrite(FR_DCM,0);
        }

        Vtg = 0.9;

        if(flag==1){
            Vtg = 0.0;
            conts = false;
        }
        setVoltage(spiHandle, 0, fabs(Vtg));

        if(abs(anglem)>tpi*0.5){
            flag = 1;
        }

    }

    spiClose(spiHandle);
    gpioTerminate();

    return 0;
}
