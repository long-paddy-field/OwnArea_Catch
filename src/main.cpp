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
Thread thread(osPriorityNormal, 1024);
KRA_PID mypid_1(0.1,0,39200,0,0.9);
KRA_PID mypid_2(0.1,0,160,0,0.3);
Timer time1;
Timer time2;

QEI encoder1(PA_8,PA_9,NC,360,&time1,QEI::X4_ENCODING);
QEI encoder2(PB_4,PB_5,NC,500,&time2,QEI::X4_ENCODING);
CAN can(PA_11,PA_12,500000);
unsigned char data2 = 0;
unsigned char data6 = 0;//0は閉じる　1は開ける
CANMessage msg1(0x1,CANStandard);
CANMessage msg2(0x2,&data2);
CANMessage msg4(0x4,CANStandard);
CANMessage msg5(0x5,CANStandard);
CANMessage msg6(0x6,&data6);
CANMessage msg8(0x8,CANStandard);
CANMessage msg9(0x9,CANStandard);

void TestEncoder();//QEIのテスト
void TestPID();//PIDのテスト
void setorigin();//リミットスイッチが押されるまで負方向に＋押されたらそこをゼロに
void setterminal();//リミットスイッチが押されるまで制方向に＋押されたらそこを読み取る
void movepid1(float goal);//goalまでPID制御で進む
void movepid2(float goal);
void valve_request(bool status);//電磁弁に動けと送信
void encoder();//エンコーダ―読み取り
void tunepid();//pidテスト用
void endprotocol();//終わり処理

Ticker checkend;
float bisco_location[7]={0,0,0,0,0,0,0};
int location_pointer = 1;
float FW_angle[3]={0,0,0};//[0]:ビスコ [1]:真ん中 [2]:滑り台
int now_goal1 = 1;//1~6:ビスコ　7:滑り台
int now_goal2 = 1;//0:ビスコ 1:正午 2:滑り台
float pid_output1 = 0;
float pid_output2 = 0;

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

  if(mode_sw)
  {
    //青の時
    bisco_location[0] = 0;
    bisco_location[1] = 4000;
    bisco_location[2] = 8000;
    bisco_location[3] = 12000;
    bisco_location[4] = 16000;
    bisco_location[5] = 20000;
    bisco_location[6] = 24000;
    FW_angle[0] = -100;
    FW_angle[1] = 0;
    FW_angle[2] = 100;
  }else{
    //赤の時
    bisco_location[0] = 0;
    bisco_location[1] = 4000;
    bisco_location[2] = 8000;
    bisco_location[3] = 12000;
    bisco_location[4] = 16000;
    bisco_location[5] = 20000;
    bisco_location[6] = 24000;
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
//phaze 1:PI制御のセッティング
  printf("Let's begin setting!\n");
  setorigin();
  setterminal();
  can.write(msg2);
  printf("End setting PID!\n");
//phaze 2:シフトボタンで
  printf("start setting location\n");
  while(1)
  {
    if(can.read(msg4))
    {
      movepid1(bisco_location[msg4.data[0]]);
      now_goal1 = msg4.data[0];
      printf("now location_num %d",msg4.data[0]);
    }else if(can.read(msg5))
    {
      printf("End setting Location\n");
      break;
    }
    wait_us(100000);
  }
//phaze 3:本格稼働
  bool pause = false;
  int task_num = 0;
  checkend.attach(callback(&endprotocol),0.1);
  while(!can.read(msg8))
  {
    if(!LS1_1||!LS1_2)
    {
      pwm2 = 0;
      dir2 = 0;
      while(1)
      {

      }
    }

    if(can.read(msg9))
    {
      pause = !pause;
    }
    mypid_2.setgoal(FW_angle[now_goal2]);
    if(!pause)
    {
      switch (task_num)
      {
        case 0:
        now_goal2 = 1;
        if(mypid_2.judgePID())
        {
          task_num++;
        }
        break;
        
        case 1:
        now_goal2 = 1;
        mypid_1.setgoal(bisco_location[now_goal1-1]);
        pid_output1 = mypid_1.calPID(encoder1.getSumangle());
        pwm1 = abs(pid_output1);
        dir1 = pid_output1 > 0 ? 1 : 0;
        if(mypid_1.judgePID())
        {
          task_num++;
          pwm1 = 0;
        }
        break;
        
        case 2:
        now_goal2 = 0;
        if(mypid_2.judgePID())
        {
          task_num++;
          msg6.data[0] = 0;
          can.write(msg6);
          wait_us(500000);
        }
        break;
        
        case 3:
        now_goal2 = 1;
        if(mypid_2.judgePID())
        {
          task_num++;
        }
        break;
        
        case 4:
        now_goal2 = 1;
        mypid_1.setgoal(bisco_location[6]);
        pid_output1 = mypid_1.calPID(encoder1.getSumangle());
        pwm1 = abs(pid_output1);
        dir1 = pid_output1 > 0 ? 1 : 0;
        if(mypid_1.judgePID())
        {
          task_num++;
          pwm1 = 0;
        }
        break;
        
        case 5:
        now_goal2 = 2;
        if(mypid_2.judgePID())
        {
          task_num = 0;
          msg6.data[0] = 1;
          can.write(msg6);
          now_goal1++;
          if(now_goal1 > 5)
          {
            now_goal1 = now_goal1 - 5;
          }
        }
        break;
      }
    }
    
    pid_output2 = mypid_2.calPID(encoder2.getSumangle());
    pwm2 = abs(pid_output2);
    dir2 = pid_output2 > 0 ? 0 : 1;
    wait_us(100000);
  }
//phaze 4:終了処理
  printf("end protocol activated\n");
  movepid2(FW_angle[2]);
  pwm1 = 0;
  pwm2 = 0;
  printf("all program completed\n");
  return 0;
}
/*
int main() {
  printf("program started\n");
  sw.mode(PullUp);
  pwm1.period_ms(1);
  pwm2.period_ms(1);
  mypid_1.setgain(20,0.1,0);
  mypid_2.setgain(2.5,0.1,0);
  while (sw) {
  }
  movepid2(100);
  while(1)
  {
    
  }
//phaze 0:スタートボタンが押されるまで待機
  printf("now waitinig...\n");
  while(can.read(msg1))
  {
    wait_us(100000);
  }

//phaze 1:PI制御のパラメータを設定
  setorigin();
  setterminal();
  can.write(msg2);

//phaze 2:シフトに反応してスタート位置を決定
  while(1)
  {
    while(can.read(msg5))
    {
      wait_us(100000);
    }
   
  }
//phaze 3:稼働開始
  checkend.attach(callback(&endprotocol),0.1);
  while(1)
  {
    //指定位置まで移動
    movepid1(bisco_location[location_pointer-1]);
    //観覧車動かす
    movepid2(FW_angle[0]);
    valve_request(0);
    wait_us(500000);
    movepid2(FW_angle[1]);
    //滑り台まで移動
    movepid1(bisco_location[6]);
    //観覧車動かす
    movepid2(FW_angle[2]);
    valve_request(1);
    wait_us(500000);
    movepid2(FW_angle[1]);

    location_pointer++;
    if(location_pointer > 7)
    {
      location_pointer -= 6;
    }
  }

}
*/
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
    dir2 = pid_output > 0 ? 0 : 1;
    printf("goal:100 now:%f output:%f\n",encoder2.getSumangle(),pid_output);
    wait_us(100000);
  } while (!mypid_2.judgePID());
  pwm2=0;
}

void valve_request(bool status)
{
  msg6.data[0] = (status == true) ? 0 : 1;
  can.write(msg6);
}

void endprotocol()
{
  if(can.read(msg8))
  {
    checkend.detach();
    movepid2(FW_angle[1]);
    pwm1 = 0;
    pwm2 = 0;
    while(1)
    {

    }
  }
}