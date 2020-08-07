#include "motordriver.h"

Motor::Motor(PinName pwm , PinName fwd , PinName rev , float frequency):
             pwm_(pwm) , fwd_(fwd) , rev_(rev) , frequency_(frequency) {

    pwm_.period(1/frequency_); // PWM周期の設定
    pwm_ = 0.0;

    fwd_ = 0;
    rev_ = 0;

}

void Motor::rotate(float speed) {
    /* speedを-1.0~1.0の範囲に設定 */
    if(speed > 1.0) speed =  1.0;
    if(speed <-1.0) speed = -1.0;
    
    fwd_ = (speed > 0.0);
    rev_ = (speed < 0.0);
    pwm_ = fabs(speed);
}

void Motor::brake(void) {
    fwd_ = 1;
    rev_ = 1;
    pwm_ =  1.0;
}
