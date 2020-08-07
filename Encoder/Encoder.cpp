#include "Encoder.h"
 
Encoder::Encoder(PinName A , PinName B ,  int pulseNum ,  float aSample, float vSample) : 
                             qei(A , B , NC , pulseNum , QEI::X4_ENCODING) {
 
    pulseNum_ = pulseNum;
    aSample_  = aSample;
    vSample_  = vSample;
 
    initParam();
 
    /* 回転角度計算タイマー割り込み開始 */
    AngleTicker.attach(this , &Encoder::calcAngle , aSample_);
    /* 回転速度計算タイマー割り込み開始 */
    VelocityTicker.attach(this , &Encoder::calcVelocity , vSample_);
}
 
void Encoder::initParam(void) {
 
    qei.reset();
    /* 現在の回転角を原点に設定 */
    Angle = pAngle = (float)qei.getPulses()/(float)pulseNum_*2.0*PI;
 
    Velocity  = 0.0;
    pVelocity = 0.0;
}
 
void Encoder::calcAngle(void) {
    /* エンコーダから回転角を取得 */
    Angle = (float)qei.getPulses()/(float)pulseNum_*2.0*PI;
}
 
void Encoder::calcVelocity(void) {
    float dAngle;
 
    dAngle = Angle - pAngle ; // 角度の差分を計算
    pAngle = Angle ; // 前回の回転角を更新
 
    //while(dAngle > PI){ dAngle -= 2*PI;} //角度の差分の範囲を-π~πにする
    //while(dAngle <-PI){ dAngle += 2*PI;}
 
    /* 速度を計算(回転角の微分 dAngle/Sample_)して、一次LPFに通す
       現在の速度 = LPF係数 × 前回の速度 + 現在の速度 × (1 - LPF係数) */
    Velocity = LPF * pVelocity + (dAngle/vSample_) * (1.0-LPF);
    pVelocity = Velocity; //前回の速度を更新
}
 
float Encoder::getVelocity(void) {
    return Velocity;
}
 
int Encoder::getPulses(void) {
    return qei.getPulses();
}
 
float Encoder::getAngle(void) {
 
    return Angle;
}
 
float Encoder::getRevolution(void) {
 
    return qei.getPulses() / pulseNum_;
}