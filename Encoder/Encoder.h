/* --- エンコーダから回転角を取得して速度を計算するクラス --- */
 
#ifndef __Encoder_h
#define __Encoder_h
 
/**
 * Includes.
 */
#include "mbed.h"
#include "QEI.h"
 
/**
 * Defines.
 */
#define PI 3.14159265358979
 
#define LPF 0.2 //0~1 速度計算,LPF係数
 
class Encoder {
 
public:
 
    /**
     *
     * A エンコーダA相ピン変化割り込みピン
     * B エンコーダB相ピン変化割り込みピン
     * pulseNum 一回転辺りのパルス数
     * aSample 回転角取得周期[s]
     * vSample 回転速度取得周期[s]
    */      
    Encoder(PinName A , PinName B , int pulseNum , float aSample, float vSample);
 
    /* 初期値の設定 */
    void initParam(void);
 
    /* 回転角を取得するタイマー割り込み関数　*/
    void calcAngle(void);
    
    /* 回転速度を取得するタイマー割り込み関数　*/
    void calcVelocity(void);
 
    /* 回転速度を計算する関数 */
    float getVelocity(void); //回転速度[rad/s]の取得
 
    int getPulses(void); //パルス数の取得
 
    float getAngle(void); //回転角度[rad]の取得
 
    float getRevolution(void); //回転数の取得
 
private:
 
    QEI qei;
 
    Ticker AngleTicker; // 回転角度計算タイマー割り込み
    Ticker VelocityTicker; //回転速度タイマー割り込み
 
    int pulseNum_;
    
    float aSample_;
    float vSample_;
 
    float  Angle;   // 回転角度 [rad]
    float  pAngle;  // 前回の回転角度 [rad]
 
    float  Velocity;  // 回転速度 [rad/s]
    float  pVelocity; // 前回の回転速度 [rad/s]
 
 
};
 
#endif 