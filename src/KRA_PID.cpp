#include "KRA_PID.h"
#include "mbed.h"
#include <math.h>

KRA_PID::KRA_PID(float _Tspan,float _in_min,float _in_max,float _out_min,float _out_max)
:Tspan(_Tspan),In_min(_in_min),In_max(_in_max),Out_min(_out_min),Out_max(_out_max)
{
    Out_sign = 0;

    Kp = 0;//比例ゲイン
    Ki = 0;//積分ゲイン
    Kd = 0;//微分ゲイン

    pid_input = 0;//入力値
    pid_goal = 0;//目標値
    pid_output = 0;//出力値

    error = 0;//偏差
    acc = 0;//累積和
    dif = 0;//差分

    Prev_L = 0;//dt秒前の位置
    Prev_E = 0;//dt秒前の偏差
    Prev_O = 0;//dt秒前の出力
}

void KRA_PID::setgain(float _p,float _i,float _d)
{
    //比例ゲイン、積分ゲイン、微分ゲインの設定
    Kp=_p;
    Ki=_i;
    Kd=_d;
}

void KRA_PID::setgoal(float argument)
{
    //目標値の設定
    pid_goal = standardize(argument);
}

float KRA_PID::standardize(float argument)
{
    //最小を0,最大を1になるよう縮尺を変更
    return (argument - In_min)/(In_max - In_min);
}

float KRA_PID::calPID(float argument)
{
    //他の値に合わせ入力も標準化
    pid_input = standardize(argument);

    //偏差の計算
    error = pid_goal-pid_input;

    //積分量の計算
    if(!(Prev_O > 1 && error > 0) && !(Prev_O < -1 && error < 0))
    {
        acc += error * Tspan;
    }

    //微分量の計算
    dif = (error-Prev_E)/Tspan;

    //出力を計算
    pid_output = Kp*error + Ki*acc + Kd*dif;

    //パラメータの更新
    Prev_O = pid_output;
    Prev_E = error;
    Prev_L = pid_input;
    
    calsign(pid_output);
    if(pid_output>1)
    {
        return Out_max;
    }else if(pid_output<-1)
    {
        return -1*Out_max;
    }else{
        return pid_output*Out_max;
    }
}

bool KRA_PID::judgePID()
{
    if(abs(Prev_E)<0.05)
    {
        return true;
    }else{
        return false;
    }
}

void KRA_PID::calsign(float argument)
{
    if(argument>0)
    {
        Out_sign = 1;
    }else
    {
        Out_sign = 0;
    }
}