#ifndef _KRA_PID_
#define _KRA_PID_
#include"mbed.h"

class KRA_PID
{
    public:
    KRA_PID(float _Tspan,float _in_min,float _in_max,float _out_min,float _out_max);

    float Tspan;//dtの長さ
    float In_min;//入力の下限
    float In_max;//入力の上限
    float Out_min;//出力の下限
    float Out_max;//出力の上限
    int Out_sign;//出力値の符号

    bool judgePID();//目標値に達したか判定
    void setgain(float,float,float);//ゲインの設定
    void setgoal(float);//目標値の設定
    float calPID(float);//出力値を計算
    void calsign(float);//出力の正負を真偽地に変換

    float error;//偏差
    float acc;//累積和
    float dif;//差分

    private:
    float Kp;//比例ゲイン
    float Ki;//積分ゲイン
    float Kd;//微分ゲイン

    float pid_input;//入力値
    float pid_goal;//目標値
    float pid_output;//出力値

    float Prev_L;//dt秒前の位置
    float Prev_E;//dt秒前の偏差
    float Prev_O;//dt秒前の出力
    float standardize(float);
};
#endif