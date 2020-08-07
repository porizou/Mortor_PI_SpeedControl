#include "mbed.h"
#include "rtos.h"

#include "Encoder.h"
#include "SDFileSystem.h"
#include "motordriver.h"

#define V_CONTROL_CYCLE 0.01 //速度制御周期[s]

#define Kp 0.03 //比例ゲイン
#define Ki 0.578 //積分ゲイン

#define Vmax 8.2 //モータの電源電圧[V]

DigitalOut led(LED1);

SDFileSystem sd(p5, p6, p7, p8, "sd"); // the pinout on the mbed Cool Components workshop board

Serial pc(USBTX, USBRX); // tx, rx

Encoder Enc(p21, p22, 60, 0.001, 0.01);
Motor motor(p25 , p23 , p24 , 10000);

float accError = 0.0; //偏差の積分値
float prevError = 0.0; //前回の偏差

float Vref = 0.0; //速度の目標値[rad/s]
float V = 0.0;


/* 速度制御スレッド */
void VelocityThread(void const *argument) {

    V = Enc.getVelocity(); //速度の値を取得[rad/s]
    
    float Error = Vref - V; //偏差の計算
    
    accError += (Error + prevError) / 2.0 * V_CONTROL_CYCLE; //偏差の積分値の計算
    
    float Vout = Kp * Error + Ki * accError; //出力電圧を計算
    
    motor.rotate(Vout / Vmax); //モータドライバに出力
    
    prevError = Error;
}

int main() {
    
    //pc.baud(115200);
    
    RtosTimer Velocity(VelocityThread); //速度制御
    
    Velocity.start(V_CONTROL_CYCLE * 1000);
    //motor.rotate(1.0);
    while(1) {
        Vref = 200;
        
        pc.printf("%f,%f\n\r",Vref, Enc.getVelocity());
        led = !led;
        wait(0.001);
    }
}
