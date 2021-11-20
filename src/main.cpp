asm(".global _printf_float");

#include <mbed.h>
#include "KRA_PID.h"
#include "QEI.h"
#include "controller.h"

#define LIMIT1_1 PA_0
#define LIMIT1_2 PA_4
#define LIMIT2_1 PF_1
#define LIMIT2_2 PF_0
#define SHOOTING 0
#define SLIDE 10000
//各種コンストラクタの宣言
DigitalOut dir1(PA_7);
PwmOut dir2(PA_3);
PwmOut pwm1(PA_6);
PwmOut pwm2(PA_1);

DigitalOut led(LED1);
DigitalIn sw(PA_10);
DigitalIn mode_sw(PB_0);

DigitalIn LS1_1(LIMIT1_1);
DigitalIn LS1_2(LIMIT1_2);
DigitalIn LS2_1(LIMIT2_1);
DigitalIn LS2_2(LIMIT2_2);

Timer time1;
Timer time2;
KRA_PID mypid_1(0.1,0,10000,0,0.9);
KRA_PID mypid_2(0.1,0,160,0,0.3);

QEI encoder1(PA_8,PA_9,NC,360,&time1,QEI::X4_ENCODING);
QEI encoder2(PB_4,PB_5,NC,500,&time2,QEI::X4_ENCODING);
CAN can(PA_11,PA_12,500000);
unsigned char data2 = 0;

CANMessage msg1(0x1,CANStandard);
CANMessage msg2(0x2,&data2);
CANMessage msg3(0x3,CANStandard);
CANMessage msg4(0x4,CANStandard);

void TestEncoder();//QEIのテスト
void TestPID();//PIDのテスト
void setorigin();//リミットスイッチが押されるまで負方向に＋押されたらそこをゼロに
void setterminal();//リミットスイッチが押されるまで制方向に＋押されたらそこを読み取る
void movepid1(float goal);//goalまでPID制御で進む
void movepid2(float goal);

int time_counter = 0;
float bisco_location[8]={0,0,0,0,0,0,0,0};//0~5:ビスコ　6:シューティングボックス 7:コンベア
float FW_angle[3]={0,0,0};//[0]:ビスコ [1]:真ん中 [2]:滑り台
float angle_origin = 0;
int location_pointer = 0;
int angle_pointer = 1;
int now_goal1 = 0;//0~5:ビスコ　
int now_goal2 = 1;//0:ビスコ 1:正午 2:滑り台
float pid_output1 = 0;
float pid_output2 = 0;

Controller controller(can,0x334);

int main()
{
  printf("program started\n");
  sw.mode(PullUp);
  LS1_1.mode(PullUp);
  LS1_2.mode(PullUp);
  LS2_1.mode(PullUp);
  LS2_2.mode(PullUp);
  pwm1.period_ms(1);
  pwm2.period_ms(1);
  dir2.period_us(50);
  mypid_1.setgain(20,0.1,0);
  mypid_2.setgain(2.5,0.1,0);
  
/*
  while(sw)
  {

  }
  while(1)
  {
    if(!LS2_1||!LS2_2)
    {
      pwm2 = 0;
    }
    mypid_2.setgoal(30);
    pid_output2 = mypid_2.calPID(encoder2.getSumangle());
    if(abs(mypid_2.error) < 0.01)
    {
      //rocked_anch_phaze
      printf("rocked_anti_phaze\n");
      pwm2 = 1;
      dir2 = 0.5;
   }else{
      printf("PID controll\n");
      pwm2 = abs(pid_output2);
      dir2 = pid_output2 > 0 ? 0 : 1;
    }
    printf("now angle:%f ,output:%f, error:%f\n",encoder2.getSumangle(),pid_output2,mypid_2.error);
    printf("pid_input%f:,pid_goal:%f\n",mypid_2.pid_input,mypid_2.pid_goal);
    wait_us(100000);
  }
*/
  while(!can.read(msg1))
  {
    
  }  if(mode_sw)
  {
    //青の時
    FW_angle[0] = -100;
    FW_angle[1] = 0;
    FW_angle[2] = 100;
  }else{
    //赤の時
    FW_angle[0] = -100;
    FW_angle[1] = 0;
    FW_angle[2] = 100;
  }
//phaze 0:スタートボタンが押されるまで待機
  printf("now waiting msg1...\n");
  while(!can.read(msg1))
  {
    wait_us(100000);
  }
//phaze 1:本格稼働
  while(!can.read(msg4))
  {
    if(!LS1_1||!LS1_2)
    {
      pwm2 = 0;
      dir2 = 0;
      while(1)
      {

      }
    }
    mypid_1.setgoal(bisco_location[location_pointer]);
    mypid_2.setgoal(FW_angle[angle_pointer]);
    pid_output1 = mypid_1.calPID(encoder1.getSumangle());
    pid_output2 = mypid_2.calPID(encoder2.getSumangle());    
    if(can.read(msg3)&&msg3.data[0])
    {
      if(controller.axes.x == 100)
      {
        if(controller.buttons[5] == 1)
        {
          pwm1 = 0.7;
          dir1 = 0;
        }else
        {
          location_pointer++;
          if(location_pointer > 5)
          {
            location_pointer -= 6;
          }
        }
      }else if(controller.axes.x == -100)
      {
        if(controller.buttons[5] == 1)
        {
          pwm1 = 0.7;
          dir1 = 1;
        }else
        {
          location_pointer--;
          if(location_pointer < 0)
          {
            location_pointer += 6;
          }  
        }
      }else if(controller.axes.y == 100)
      {
        if(controller.buttons[5] == 1)
        {
          if(mode_sw == 0)
          {
            dir2 = 0;
          }else{
            dir2 = 1;
          }
          pwm2 = 0.2;
        }else
        {
          location_pointer = 7;
        }
      }else if(controller.axes.y == -100)
      {
        if(controller.buttons[5] == 1)
        {
          if(mode_sw == 0)
          {
            dir2 = 1;
          }else{
            dir2 = 0;
          }
          pwm2 = 0.2;
        }else
        {
          location_pointer = 6;
        }
      }else if(controller.buttons[4] == 1)
      {
        if(controller.buttons[5] == 1)
        {
          msg2.data[0] = 1;
          can.write(msg2);
        }else
        {
          movepid2(FW_angle[0]);
          msg2.data[0] = 1;
          can.write(msg2);
          wait_us(1000000);
          movepid2(FW_angle[1]);
        }

      }else if(controller.buttons[6] == 1)
      {
        if(controller.buttons[5] == 1)
        {
          msg2.data[0] = 0;
          can.write(msg2);
        }else
        {
          movepid2(FW_angle[2]);
          msg2.data[0] = 1;
          can.write(msg2);
          wait_us(1000000);
          movepid2(FW_angle[1]);
        }     
      }
    }
    time_counter++;
    wait_us(100000);
  }
  movepid2(angle_origin);
  pwm1 = 0;
  pwm2 = 0;
  return 0;
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
    dir2 = pid_output > 0 ? 0 : 1;
    printf("goal:100 now:%f output:%f\n",encoder2.getSumangle(),pid_output);
    wait_us(100000);
  } while (!mypid_2.judgePID());
  pwm2=0;
}