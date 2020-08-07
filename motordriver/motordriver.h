#ifndef __motordriver_h
#define __motordriver_h

/**
 * Includes
 */
#include "mbed.h"

class Motor {

public:
    /*
     * pwm --- PwmOut pin
     * fwd --- DigitalOut pin モーター正転
     * rev --- DigitalOut pin モーター逆転
     */
    Motor(PinName pwm , PinName fwd , PinName rev , float frequency);

    /* --- モーター回転 --- */
    void rotate(float speed);

    /* --- モーターHブレーキ ---*/
    void brake(void);

private:
    
    PwmOut     pwm_;
    DigitalOut fwd_;
    DigitalOut rev_;
    float frequency_; //PWM周波数

};


#endif /* __motordriver_h */





















             #ifndef __motordriver_h
#define __motordriver_h

/**
 * Includes
 */
#include "mbed.h"

class Motor {

public:
    /*
     * pwm --- PwmOut pin
     * fwd --- DigitalOut pin モーター正転
     * rev --- DigitalOut pin モーター逆転
     */
    Motor(PinName pwm , PinName fwd , PinName rev , float frequency);

    /* --- モーター回転 --- */
    void rotate(float speed);

    /* --- モーターHブレーキ ---*/
    void brake(void);

private:
    
    PwmOut     pwm_;
    DigitalOut fwd_;
    DigitalOut rev_;
    float frequency_; //PWM周波数

};


#endif /* __motordriver_h */





















             