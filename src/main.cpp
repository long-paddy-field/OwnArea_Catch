asm(".global _printf_float");

#include <mbed.h>
#include "KRA_PID.h"


#define LIMIT1_1 PA_0
#define LIMIT1_2 PA_4
#define LIMIT2_1 PF_1
#define LIMIT2_2 PF_0

#define ENC_1
#define ENC_3

//各種コンストラクタの宣言
DigitalOut dir(PA_7);
DigitalOut dir1(PA_3);
PwmOut pwm1(PA_1);
PwmOut pwm0(PA_6);

DigitalOut led(LED1);
DigitalIn sw(PA_10);

InterruptIn LS1_1(LIMIT1_1);
InterruptIn LS1_2(LIMIT1_2);
InterruptIn LS2_1(LIMIT2_1);
InterruptIn LS2_2(LIMIT2_2);

Thread thread(osPriorityNormal, 1024);

KRA_PID mypid_1(0.1,0,39200,0,0.9);
KRA_PID mypid_2(0.1,0,39200,0,0.9);

CAN can(PA_11,PA_12,500000);

void TestEncoder();//QEIのテスト
void TestPID();//PIDのテスト
void waitstart();//startボタンが押されるまで待機
void setorigin();//リミットスイッチが押されるまで負方向に＋押されたらそこをゼロに
void setterminal();//リミットスイッチが押されるまで制方向に＋押されたらそこを読み取る
void movepid(float goal);//goalまでPID制御で進む
void valve_request();//電磁弁に動けと送信
void encoder();//エンコーダ―読み取り
void tunepid();//pidテスト用


int main() {
  printf("program started\n");
  sw.mode(PullUp);

  while (sw) {
  }

//task 0:スタートボタンが押されるまで待機
  printf("now waitinig...\n");
  waitstart();

//task 1:初期位置の計測
  printf("task 1 started\n");
  
  printf("task 1 ended.\n");

//task 2:ビスコの位置まで移動
  printf("task 2 started\n");
  
  printf("task 2 ended\n");

//task 3:観覧車で回収
  printf("task 3 started\n"); 
  printf("task 3 ended");

//task 4:滑り台まで移動
  printf("task 4 started\n");
  printf("task 4 ended\n");

  while (true) {
  }
}

void waitstart()
{
  CANMessage msg(0x0,CANStandard);
  while(!can.read(msg))
  {
    wait_us(100000);
  }
}

void setorigin()
{

}

void setterminal()
{

}

void movepid()
{

}

void valve_request()
{

}