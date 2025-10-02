/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:10:45
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

/*硬件设备成员对象序列*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Motor AppMotor;

/*f(x) int */
static boolean
function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

/*运动方向控制序列*/
enum SmartRobotCarMotionControl
{
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it        //(9)
};               //direction方向:（1）、（2）、 （3）、（4）、（5）、（6）

/*模式控制序列*/
enum SmartRobotCarFunctionalModel
{
  TraceBased_mode,        /*循迹模式*/
};

/*控制管理成员*/
struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
 
Application_xxx Application_SmartRobotCarxxx0;

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  while (Serial.read() >= 0)
  {
    /*清空串口缓存...*/
  }
  delay(2000);
  Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
}

/*ITR20001 检测小车是否离开地面*/
static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void)
{
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() > Application_FunctionSet.TrackingDetection_V)
  {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  }
  else
  {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}
/*
  直线运动控制：
  direction：方向选择 前/后
  directionRecord：方向记录（作用于首次进入该函数时更新方向位置数据，即:yaw偏航）
  speed：输入速度 （0--255）
  Kp：位置误差放大比例常数项（提高位置回复状态的反映，输入时根据不同的运动工作模式进行修改）
  UpperLimit：最大输出控制量上限
*/
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw; //偏航
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }
  //if (en != directionRecord)
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }
  //加入比例常数Kp
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }
  if (direction == Forward) //前进
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
                                           /*direction_B*/ direction_just, /*speed_B*/ L, /*controlED*/ control_enable);
  }
  else if (direction == Backward) //后退
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
                                           /*direction_B*/ direction_back, /*speed_B*/ R, /*controlED*/ control_enable);
  }
}
/*
  运动控制:
  1# direction方向:前行（1）、后退（2）、 左前（3）、右前（4）、后左（5）、后右（6）
  2# speed速度(0--255)
*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;
  //需要进行直线运动调整的控制模式（在以下工作运动模式小车前后方向运动时容易产生位置偏移，运动达不到相对直线方向的效果，因此需要加入控制调节）
    Kp = 10;
    UpperLimit = 255;

  switch (direction)
  {
  case /* constant-expression */
      Forward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //前进时进入方向位置逼近控制环处理
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }

    break;
  case /* constant-expression */ Backward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //后退时进入方向位置逼近控制环处理
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 2;
    }

    break;
  case /* constant-expression */ Left:
    /* code */
    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Right:
    /* code */
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftForward:
    /* code */
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftBackward:
    /* code */
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightForward:
    /* code */
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightBackward:
    /* code */
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ stop_it:
    /* code */
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    break;
  default:
    directionRecord = 10;
    break;
  }
}
void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void)
{
  { /*R循迹状态更新*/
    TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    TrackingDetectionStatus_R = function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
    TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    TrackingDetectionStatus_M = function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
    TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    TrackingDetectionStatus_L = function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E);
    //ITR20001 检测小车是否离开地面
    ApplicationFunctionSet_SmartRobotCarLeaveTheGround();
  }
}





void ApplicationFunctionSet::ApplicationFunctionSet_Tracking(bool isStop)
{
    static unsigned long lastTime = 0;
    static float integral = 0;
    static float prevError = 0;
    static bool timestamp = true;
    static bool blindDetection = true;
    static unsigned long motorRL_time = 0;

    if (isStop) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }

    if (Application_SmartRobotCarxxx0.Functional_Mode != TraceBased_mode)
        return;

    if (Application_FunctionSet.Car_LeaveTheGround == false) 
    {
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        return;
    }

    float sensor_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    float sensor_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    float sensor_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    Serial.println("\nsol : ");
    Serial.print(sensor_L);
    Serial.println("\norta : ");
    Serial.print(sensor_M);    
    Serial.println("\nsağ : ");
    Serial.print(sensor_R);

    float error = 0;
    if (function_xxx(sensor_M, TrackingDetection_S, TrackingDetection_E))
    {
        error = 0; // orta çizgi üzerindeyiz
    }
    else if (function_xxx(sensor_R, TrackingDetection_S, TrackingDetection_E))
    {
        error = 1.0; // sağa sapma
    }
    else if (function_xxx(sensor_L, TrackingDetection_S, TrackingDetection_E))
    {
        error = -1.0; // sola sapma
    }
    else // çizgiyi kaybettik
    {
        error = prevError; // son hatayı koru
    }

    // Zaman farkı
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001; // sıfır bölmeye karşı
    lastTime = currentTime;

    // PID terimleri
    integral += error * dt;                       // Integral
    float derivative = (error - prevError) / dt;  // Türev
    prevError = error;

    // PID parametreleri (deneme ile ayarlanmalı)
    float Kp = 120.0;
    float Ki = 0.0;
    float Kd = 60.0;
    int baseSpeed = 100;  // Motor temel hızı
    int UpperLimit = 255;

    // Motor hızını PID ile ayarla
    int speedCorrection = Kp * error + Ki * integral + Kd * derivative;
    int leftSpeed = baseSpeed - speedCorrection;
    int rightSpeed = baseSpeed + speedCorrection;

    leftSpeed = constrain(leftSpeed, 0, UpperLimit);
    rightSpeed = constrain(rightSpeed, 0, UpperLimit);

    // Motor kontrolü
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 0); // yön sabit Forward
    AppMotor.DeviceDriverSet_Motor_control(direction_just, rightSpeed, direction_just, leftSpeed, control_enable);

    // Blind detection ve stop mekanizması
    if (!function_xxx(sensor_L, TrackingDetection_S, TrackingDetection_E) &&
        !function_xxx(sensor_M, TrackingDetection_S, TrackingDetection_E) &&
        !function_xxx(sensor_R, TrackingDetection_S, TrackingDetection_E))
    {
        if (timestamp)
        {
            timestamp = false;
            motorRL_time = millis();
            ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        }

        unsigned long m = millis();
        if ((function_xxx((m - motorRL_time), 0, 200) || function_xxx((m - motorRL_time), 1600, 2000)) && blindDetection)
        {
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
        }
        else if ((function_xxx((m - motorRL_time), 200, 1600)) && blindDetection)
        {
            ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
        }
        else if ((function_xxx((m - motorRL_time), 3000, 3500)))
        {
            blindDetection = false;
            ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        }
    }
    else
    {
        timestamp = true;
        blindDetection = true;
    }
}
