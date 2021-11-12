asm(".global _printf_float");

#include <encoder1.h>
#include <encoder3.h>
#include <mbed.h>
#include "KRA_PID.h"

DigitalOut dir(PA_7);
DigitalOut dir1(PA_3);
PwmOut pwm1(PA_1);
PwmOut pwm0(PA_6);
// DigitalOut dir(PA_6);

DigitalOut led(LED1);

DigitalIn sw(PA_10);

Encoder1 encoder1(360);
//Encoder3 encoder3(360);

Thread thread(osPriorityNormal, 1024);

KRA_PID mypid(0.1,0,39200,0,0.9);


void encoder() {
  encoder1.start();
  //encoder3.start();
  pwm0.period_ms(10);

  while (true) {
    encoder1.update();
//    encoder3.update();

/*    printf("angle1: %lld, %lld\n", encoder1.getRawSumPulse(),
           encoder3.getRawSumPulse());
           */
    // printf("%d\n", TIM1->CNT);
    ThisThread::sleep_for(10ms);
  }
}

void TunePID()
{
  encoder1.start();
//  encoder3.start();
  pwm0.period_ms(10);
  mypid.setgain(10,0.2,0);
  mypid.setgoal(10000);
  led = 1;
  float output=0;
  do
  {
    encoder1.update();
//    encoder3.update();
    
    output = mypid.calPID(encoder1.getRawSumPulse());
    pwm0 = abs(output);
    dir = mypid.Out_sign;
    printf("goal = 10000,now=%lld,error=%f",encoder1.getRawSumPulse(),mypid.error);
    printf("pwmoutput:%f ,dir:%d\n",output,mypid.Out_sign);
    wait_us(100000);

  } while (!mypid.judgePID());
  pwm0 = 0;
  printf("task ended\n");
  led = 0;
}

int main() {
  printf("program started\n");
  sw.mode(PullUp);

  while (sw) {
  }
  TunePID();
  while (true) {
  }
}