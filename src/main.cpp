asm(".global _printf_float");

#include <mbed.h>
#include "KRA_PID.h"
#include "QEI.h"

#define LIMIT1_1 PA_0
#define LIMIT1_2 PA_4
#define LIMIT2_1 PF_1
#define LIMIT2_2 PF_0


//各種コンストラクタの宣言
DigitalOut dir1(PA_7);
DigitalOut dir2(PA_3);
PwmOut pwm1(PA_1);
PwmOut pwm2(PA_6);

DigitalOut led(LED1);
DigitalIn sw(PA_10);

DigitalIn LS1_1(LIMIT1_1);
DigitalIn LS1_2(LIMIT1_2);
DigitalIn LS2_1(LIMIT2_1);
DigitalIn LS2_2(LIMIT2_2);
Thread thread(osPriorityNormal, 1024);
KRA_PID mypid_1(0.1,0,39200,0,0.9);
KRA_PID mypid_2(0.1,0,360,0,0.9);
Timer time1;
Timer time2;
 
QEI encoder1(PA_8,PA_9,NC,360,&time1);
QEI encoder2(PB_4,PB_5,NC,360,&time2);
CAN can(PA_11,PA_12,500000);

void TestEncoder();//QEIのテスト
void TestPID();//PIDのテスト
void waitstart();//startボタンが押されるまで待機
void setorigin();//リミットスイッチが押されるまで負方向に＋押されたらそこをゼロに
void setterminal();//リミットスイッチが押されるまで制方向に＋押されたらそこを読み取る
void movepid1(float goal);//goalまでPID制御で進む
void movepid2(float goal);
void valve_request();//電磁弁に動けと送信
void encoder();//エンコーダ―読み取り
void tunepid();//pidテスト用

float bisco_location[7]={0,0,0,0,0,0,0};
float FW_angle[3]={0,0,0};
unsigned char data_msg2 = 0;

int main() {
  printf("program started\n");
  sw.mode(PullUp);
  pwm1.period_ms(1);
  pwm2.period_ms(1);
  mypid_1.setgain(20,0.1,0);
  mypid_2.setgain(20,0.1,0);
  while (sw) {
  }

//task 0:スタートボタンが押されるまで待機
  printf("now waitinig...\n");
  waitstart();

  while(1)
  {
//task 1:初期位置の計測
  printf("task 1 started\n");
  setorigin();
  setterminal();
  printf("task 1 ended.\n");

  for(int loop = 1;loop <= 6;loop++)
  {
  //task 2:ビスコの位置まで移動
    printf("task 2 started\n");
    movepid1(bisco_location[loop-1]);
    printf("task 2 ended\n");

  //task 3:観覧車で回収
    printf("task 3 started\n"); 
    movepid2(FW_angle[0]);
    valve_request();
    wait_us(500000);
    movepid2(FW_angle[1]);
    printf("task 3 ended");

  //task 4:滑り台まで移動
    printf("task 4 started\n");
    movepid1(bisco_location[6]);
    printf("task 4 ended\n");

  //task 5:おろす
    printf("task 5 started\n");
    movepid2(FW_angle[2]);
    valve_request();
    wait_us(500000);
    movepid2(FW_angle[1]);
    printf("task 5 ended\n");
  }

  }

}

void waitstart()
{
  CANMessage msg1(0x0,CANStandard);
  while(!can.read(msg1))
  {
    wait_us(100000);
  }
}

void setorigin()
{
  while(LS1_1)
  {
    dir1 = 0;
    pwm1 = 0.5;
    wait_us(100000);
  }
  pwm1 = 0;
  encoder1.qei_reset();
}

void setterminal()
{
  while(LS1_2)
  {
    dir1 = 1;
    pwm1 = 0.5;
    printf("NUM=%f\n",encoder1.getSumangle());
    wait_us(100000);
  }
  pwm1 = 0;
  mypid_1.In_max = encoder1.getSumangle();
  printf("terminal point is %f\n",mypid_1.In_max);
}

void movepid1(float goal)
{
  mypid_1.setgoal(goal);
  do
  {
    float pid_output = mypid_1.calPID(encoder1.getSumangle());
    pwm1 = abs(pid_output);
    dir1 = pid_output > 0 ? 1 : 0;
    wait_us(100000);
  } while (!mypid_1.judgePID());
  pwm1=0;
}

void movepid2(float goal)
{
  mypid_2.setgoal(goal);
  do
  {
    float pid_output = mypid_2.calPID(encoder2.getSumangle());
    pwm2 = abs(pid_output);
    dir2 = pid_output > 0 ? 1 : 0;
    wait_us(100000);
  } while (!mypid_2.judgePID());
  pwm2=0;
}

void valve_request()
{
  CANMessage msg2(0x1,&data_msg2,1,CANData,CANStandard);
  can.write(msg2);
}