/**************************************************************************
Copyright: OneTwoLab 
Author: Jenson
Date: 2021-12-10
Demo: https://www.bilibili.com/video/BV1ZL4y1J7JL
**************************************************************************/

#include "PID_v1.3.h"
#include <stdint.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>
#include <IRremote.h>  //红外遥控头文件
 
int RECV_PIN = 2;    //管脚定义
 
IRrecv irrecv(RECV_PIN);
decode_results results;


enum PosType{
  PosTypeCENTER=0,
  PosTypeA,
  PosTypeB,
  PosTypeC,
  PosTypeD,
  PosTypeCIRCLE
};

PosType posType = PosTypeCENTER;
//////////////////////////////////////////////////////////////////////////////

#define X1 A0 
#define X2 A2 
#define Y1 A3 
#define Y2 A1 
#define Xresolution 252 
#define Yresolution 190
#define SvrXPin 7
#define SvrYPin 8 

int    Interval  = 25;
double  Setpoint, Input, Output;
double  Setpoint1, Input1, Output1;
Servo  servoX;
Servo servoY;
#define ANGLE_MIN 60
#define ANGLE_MAX 120
//
//float  Kp  = 0.3;
//float Ki  = 0.06;
//float Kd  = 0.1;
//float Kp1 = 0.25;
//float Ki1 = 0.06;
//float Kd1 = 0.08;

float  Kp  = 0.32;
float Ki  = 0.05;
float Kd  = 0.095;
float Kp1 = 0.27;
float Ki1 = 0.05;
float Kd1 = 0.085;
double  R = 0;

int NoDataFrame = 0;
int InitFrame = 0;
bool SysInit = false;

int ServoXBlanceOut = 88; // x轴舵机初始水平输出 控制y轴运动
int ServoYBlanceOut = 85; // y轴舵机初始水平输出 控制x轴运动

PID Pid( &Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT,0.3);
PID Pid1( &Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, P_ON_E, DIRECT,0.3);

struct TCPoint{
  int x;
  int y;
};

TCPoint prePoint = {0, 0}; //上次位置

SimpleKalmanFilter kalmanfilter = SimpleKalmanFilter(1, 1, 0.4);
SimpleKalmanFilter kalmanfilter1 = SimpleKalmanFilter(1, 1, 0.4);

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); //初始化红外遥控
  Setpoint  = 120;
  Setpoint1 = 86;

  Input = 0;
  Input1 = 0;
  
  //x轴转动舵机
  servoX.attach( SvrXPin );
  //y轴转动舵机
  servoY.attach( SvrYPin );
  
  Output  = ServoYBlanceOut;
  Output1 = ServoXBlanceOut;
  servoY.write( Output );
  servoX.write( Output1 );
  
  Pid.SetMode( AUTOMATIC );
  Pid.SetOutputLimits( ANGLE_MIN, ANGLE_MAX );
  Pid1.SetMode( AUTOMATIC );
  Pid1.SetOutputLimits( ANGLE_MIN, ANGLE_MAX );

  Pid.SetSampleTime( Interval );
  Pid1.SetSampleTime( Interval );

  posType = PosTypeCENTER;
}

void loop()
{

  if (irrecv.decode(&results)){
      long code=results.value;  //红外接收到遥控器数值
      Serial.println(code, HEX);

      if(code!=-1){
        //确定
        if(code == 0x8D72738C)
        {
          posType = PosTypeCENTER;
        }
        else if(code == 0x8D72A956)
        {
          //播放  
          posType = PosTypeCIRCLE;
        }
        else if(code == 0x8D72837C)
        {
          //向右顺时针
          int type = ((int)posType+1);
          if(type > (int)PosTypeD){
            type = (int)PosTypeA;
          }
          posType = (PosType)type;
        }
        else if(code == 0x8D729966)
        {
          //向左逆时针
          int type = ((int)posType-1);
          if(type < (int)PosTypeA){
            type = (int)PosTypeD;
          }
          posType = (PosType)type;
        }
        
        if(posType == PosTypeCIRCLE)
        {
          enter_circle_mode();  
        }
      }
      irrecv.resume(); // 接收下一个值
   }

   if(!SysInit)
   {
      InitFrame ++;
      SysInit = InitFrame >5;
   }
   
   
   TCPoint point = get_tcpoint();
   if(point.x > 0 && point.y > 0)
   {     
      servoX.attach(SvrXPin);
      servoY.attach(SvrYPin);

      double oldOut = Output;
      double oldOut1 = Output1;
      Input = point.x;
      Input1 = point.y;

      //卡尔曼滤波
      Input = kalmanfilter.updateEstimate(Input);
      Input1 = kalmanfilter1.updateEstimate(Input1);

      Pid.Compute();
      Pid1.Compute();

      servoY.write( ReserveOutput(Output, ServoYBlanceOut) );
      servoX.write(Output1);

      setPosition();
      
      NoDataFrame = 0;
      printLog(point.y);
      
      if(SysInit){
        prePoint = point;
      }
      
   }
   else
   {
        NoDataFrame ++;
        if(NoDataFrame >=5)
        {
          Output  = ServoYBlanceOut;
          Output1 = ServoXBlanceOut;
          servoX.write( ServoXBlanceOut );
          servoY.write( ServoYBlanceOut );
          NoDataFrame = 5;
        }
   }
}

void printLog(double newVal)
{
   Serial.print("Input ");  
   Serial.print(Input1);
   
//   Serial.print(", Input1 ");  
//   Serial.println(Input1);

     Serial.print(", CurrDv ");
     Serial.print( Pid.GetCurrDv());
     
     Serial.print(", Output ");
     Serial.println( Output1);

//      Serial.print(", OriginalDv ");
//     Serial.print( myPID1.GetOriginalDv());
//     Serial.print(", CurrDv ");
//     Serial.println( myPID1.GetCurrDv());

     
//     Serial.print(", ");
//     Serial.println( ReserveOutput(Output, ServoYBlanceOut));
  //   Serial.print(" Output1 = ");
  //   Serial.println( Output1);
}

//反转输出角度
double ReserveOutput(double output, double center)
{
  return output + 2.0*(center-output);
}

//读取触摸位置
TCPoint get_tcpoint(){
   int X,Y; 
   pinMode(Y1,INPUT); 
   pinMode(Y2,INPUT);  
   digitalWrite(Y2,LOW);
   pinMode(X1,OUTPUT);
   digitalWrite(X1,HIGH);
   pinMode(X2,OUTPUT);
   digitalWrite(X2,LOW);
   delay(1);
   X = (analogRead(Y1))/(1024/Xresolution); 
    
   pinMode(X1,INPUT);
   pinMode(X2,INPUT);
   digitalWrite(X2,LOW);
   pinMode(Y1,OUTPUT);
   digitalWrite(Y1,HIGH);
   pinMode(Y2,OUTPUT);
   digitalWrite(Y2,LOW);
   delay(1);
   Y = (analogRead(X1))/(1024/Yresolution); 

   return TCPoint{X, Y};
}

void enter_circle_mode()
{
  Setpoint  = 105 + 44 * cos( R );
  Setpoint1 = 86 + 42 * sin( R );
  R = 0;
}

void setPosition()
{
  if(posType == PosTypeCENTER)
  {
    Setpoint  = 105;
    Setpoint1 = 86;
  }
  else if(posType == PosTypeA)
  {
    Setpoint  = 60;
    Setpoint1 = 47;
  }
  else if(posType == PosTypeB)
  {
    Setpoint  = 160;
    Setpoint1 = 47;
  }
  else if(posType == PosTypeC)
  {
    Setpoint  = 160;
    Setpoint1 = 128;
  }
  else if(posType == PosTypeD)
  {
    Setpoint  = 60;
    Setpoint1 = 126;
  }
  else if(posType == PosTypeCIRCLE)
  {
      Setpoint  = 105 + 60 * cos( R );
      Setpoint1 = 86 + 50 * sin( R );
      R = R + 0.16;
  }

}
