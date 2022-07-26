
//新车电机测试
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <TimerOne.h>
#include<SoftwareSerial.h>//必要的头文件
#include <ArduinoJson.h>
//舵机对象
Servo transverse; //横向
Servo vertical; //纵向
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//修改对比测试
//ICD对象
LiquidCrystal_I2C lcd(0x27, 16, 2);

//循迹引脚
int D1 = 45;
int D2 = 44;
int D3 = 43;
int D4 = 42;
int D5 = 41;
int D6 = 40;
int D7 = 39;
int D8 = 38;

//各电机的转向控制器
#define IN1 A7
#define IN2 A6
#define IN3 A5
#define IN4 A4
#define IN5 A3
#define IN6 A2
#define IN7 A1
#define IN8 A0

//各电机的使能端
#define Enabled1 7
#define Enabled2 6
#define Enabled3 9
#define Enabled4 8

//各电机的检测相位
#define Encoder_1A 19
#define Encoder_2A 18
#define Encoder_3A 2
#define Encoder_4A 3
#define Encoder_1B 16
#define Encoder_2B 17
#define Encoder_3B 5
#define Encoder_4B 4

//各个电机的脉冲计数器
double Val1 = 0, Val2 = 0, Val3 = 0, Val4 = 0;
double last_Val1 = 0, last_Val2 = 0, last_Val3 = 0, last_Val4 = 0;

//各电机的pwm输入值0为初速度
double pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;

//各个电机的目标rpm
double target1, target2, target3, target4;

//各个电机的距离记数器
double distance1 = 0, distance2 = 0, distance3 = 0, distance4 = 0;
double last_distance1 = 0, last_distance2 = 0, last_distance3 = 0, last_distance4 = 0;

//各个电机的速度记数器
double velocity1 = 0, velocity2 = 0, velocity3 = 0, velocity4 = 0;

//各个电机的角度记数器
double angle1 = 0, angle2 = 0, angle3 = 0, angle4 = 0;

//各个电机的速度修正器
double vel1 = 0, vel2 = 0, vel3 = 0, vel4 = 0;

//PID变量
double Km1 = 4, Ki1 = 1.2, Kd1 = 0;
double Kp2 = 4, Ki2 = 1.2, Kd2 = 0;
double Kp3 = 4, Ki3 = 1.2, Kd3 = 0;
double Kp4 = 4, Ki4 = 1.2, Kd4 = 0;

//各PID的误差值
double error1, error2, error3, error4;
double last_error1, last_error2, last_error3, last_error4;

int mark = 0; //定时器标志
//按键
int G = 26,Y = 27,R = 28;
int num = 0; //当前循迹所在位置
int last_num = 0; //上一次所在位置
//各任务的进度标志位
int m1 = 0,m2 = 0;
int flag_mission = 0; //任务标志位
int Line_num = 0; //记录经过的黑线
//循迹传感器当前低电平数量
int state = 0,last_state = 0;

/*………………………………OpenMV……………………………
openmv_state接收Openmv传回的信息判断车位是否为空
location记录当前位置若车位不为空则+1检查下一个车位
…………………………………………………………………………*/
char openmv_state = 0; //位置是否为空
int  location = 1; //定位

/*……………………………………………………
函数类型：初始化函数
函数功能：初始化各部分硬件
输入：无
……………………………………………………*/
void LCD_Init() { // 初始化LCD
  lcd.init();                  
  lcd.backlight();             //设置LCD背景等亮
  Wire.begin();
}

void tracking_Init() { //8路循迹初始化
  pinMode (D1, INPUT); //设置引脚为输入引脚
  pinMode (D2, INPUT); 
  pinMode (D3, INPUT);
  pinMode (D4, INPUT);
  pinMode (D5, INPUT);
  pinMode (D6, INPUT);
  pinMode (D7, INPUT); 
  pinMode (D8, INPUT); 
}

void Servo_Init() {  //舵机初始化
  transverse.attach(27); //横向
  vertical.attach(26); //纵向
}

void Motor_Init() {
  pinMode(IN1, OUTPUT);  //转向端
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  pinMode(Enabled1, OUTPUT);   //使能端
  pinMode(Enabled2, OUTPUT);
  pinMode(Enabled3, OUTPUT);
  pinMode(Enabled4, OUTPUT);

  //编码器AB相初始化
  pinMode(Encoder_1A, INPUT);
  pinMode(Encoder_2A, INPUT);
  pinMode(Encoder_3A, INPUT);
  pinMode(Encoder_4A, INPUT);
  pinMode(Encoder_1B, INPUT);
  pinMode(Encoder_2B, INPUT);
  pinMode(Encoder_3B, INPUT);
  pinMode(Encoder_4B, INPUT);
}

void Interrupt_Init() { //中断程序初始化
  //定时器初始化
  Timer1.initialize( 5000 );   //定时器设定5ms(单位us
  Timer1.attachInterrupt( Interrupt_service );   //设置定时器触发函数

  //编码器A相初始化
  attachInterrupt(digitalPinToInterrupt(Encoder_1A), counter1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_2A), counter2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_3A), counter3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_4A), counter4, CHANGE);
  interrupts(); //启动中断
}
      //……………………Finish……………………//
//……………………定时器中断调用以下函数……………………//
void Interrupt_service() {    //中断服务函数
  if (mark == 2000 && m1 == 1) {
    mark = 0;
    m1++;
  }

  mark++;
  Openmv();
  distance_detection();
  velocity_detection();
  angle_detection();
  last_Val1 = Val1;
  last_Val2 = Val2;
  last_Val3 = Val3;
  last_Val4 = Val4;
}

void velocity_detection() { //速度检测 单位为cm/s
  velocity1 = (Val1 - last_Val1) * 22 / 1170.0 / 0.05;
  velocity2 = (Val2 - last_Val2) * 22 / 1170.0 / 0.05;
  velocity3 = (Val3 - last_Val3) * 22 / 1170.0 / 0.05;
  velocity4 = (Val4 - last_Val4) * 22 / 1170.0 / 0.05;
}

void angle_detection() {  //角度检测
  angle1 += (Val1 - last_Val1) / 1170 * 360;
  angle2 += (Val2 - last_Val2) / 1170 * 360;
  angle3 += (Val3 - last_Val3) / 1170 * 360;
  angle4 += (Val4 - last_Val4) / 1170 * 360;
}

void distance_detection() { //距离检测  单位为cm
  distance1 += (Val1 - last_Val1) / 2000 * 22;
  distance2 += (Val2 - last_Val2) / 2000 * 22;
  distance3 += (Val3 - last_Val3) / 2000 * 22;
  distance4 += (Val4 - last_Val4) / 2000 * 22;
}

void Reset() {  //重置计数器
  angle1 = angle2 = angle3 = angle4 = 0;
  distance1 = distance2 = distance3 = distance4 = 0;
  Val1=Val2=Val3=Val4=0;
}

void Openmv() { //Openmv通信
  if (Serial3.available() > 0) {
    openmv_state = Serial3.read();
  }
}
//……………………定时中断调用以上函数……………………//

//……………………外部中断调用以下函数……………………//
void counter1() {   //电机1脉冲数检测
  if (digitalRead(Encoder_1A) == HIGH)
  {
    if (digitalRead(Encoder_1B) == LOW)
      Val1--;  //根据另外一相电平判定方向
    else
      Val1++;
  }
  else
  {
    if (digitalRead(Encoder_1B) ==  LOW)
      Val1++; //根据另外一相电平判定方向
    else
      Val1--;
  }
}

void counter2() {   //电机2脉冲数检测
  if (digitalRead(Encoder_2A) == HIGH)
  {
    if (digitalRead(Encoder_2B) == LOW)
      Val2--;  //根据另外一相电平判定方向
    else
      Val2++;
  }
  else
  {
    if (digitalRead(Encoder_2B) == LOW)
      Val2++; //根据另外一相电平判定方向
    else
      Val2--;
  }
}

void counter3() {   //电机3脉冲数检测
  if (digitalRead(Encoder_3A) == HIGH)
  {
    if (digitalRead(Encoder_3B) == LOW)
      Val3++;  //根据另外一相电平判定方向
    else
      Val3--;
  }
  else
  {
    if (digitalRead(Encoder_3B) == LOW)
      Val3--; //根据另外一相电平判定方向
    else
      Val3++;
  }
}

void counter4() {   //电机4脉冲数检测
  if (digitalRead(Encoder_4A) == HIGH)
  {
    if (digitalRead(Encoder_4B) == LOW)
      Val4++;  //根据另外一相电平判定方向
    else
      Val4--;
  }
  else
  {
    if (digitalRead(Encoder_4B) == LOW)
      Val4--; //根据另外一相电平判定方向
    else
      Val4++;
  }
}

/*……………………………………………………
函数类型：四个电机的方向控制
函数功能：电机正转与反转 (Seed>0时前进
输入：pwm值 Speed
……………………………………………………*/
void go1(int Speed) {
  if (Speed > 0 && Speed <= 255) {
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, HIGH);
    analogWrite(Enabled1, abs(Speed));
  } else if (Speed >= -255 && Speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(Enabled1, abs(Speed));
  } else if (Speed == 0) {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 0);
    analogWrite(Enabled1, 0);
  }
}

void go2(int Speed) {
  if (Speed >= 0 && Speed <= 255) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(Enabled2, abs(Speed));
  } else if (Speed >= -255 && Speed <= 0) {
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
    analogWrite(Enabled2, abs(Speed));
  } else if (Speed == 0) {
    digitalWrite(IN2, 0);
    digitalWrite(IN1, 0);
    analogWrite(Enabled2, 0);
  }
}

void go3(int Speed) {
  if (Speed >= 0 && Speed <= 255) {
    digitalWrite(IN8, HIGH);
    digitalWrite(IN7, LOW);
    analogWrite(Enabled3, abs(Speed));
  } else if (Speed >= -255 && Speed <= 0) {
    digitalWrite(IN7, HIGH);
    digitalWrite(IN8, LOW);
    analogWrite(Enabled3, abs(Speed));
  } else if (Speed == 0) {
    digitalWrite(IN8, 0);
    digitalWrite(IN7, 0);
    analogWrite(Enabled3, 0);
  }
}

void go4(int Speed) {
  if (Speed >= 0 && Speed <= 255) {
    digitalWrite(IN6, HIGH);
    digitalWrite(IN5, LOW);
    analogWrite(Enabled4, abs(Speed));
  } else if (Speed >= -255 && Speed <= 0) {
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
    analogWrite(Enabled4, abs(Speed));
  } else if (Speed == 0) {
    digitalWrite(IN5, 0);
    digitalWrite(IN6, 0);
    analogWrite(Enabled4, 0);
  }
}
//……………………Finish……………………//

/*……………………………………………………
函数类型：电机组合动作
函数功能：前进、后退、左右转等……
输入：Speed、angle、distance
……………………………………………………*/
void Enabled() {  //使能函数
  go1(pwm1);
  go2(pwm2);
  go3(pwm3);
  go4(pwm4);
}

void forward(int Speed) { //前进
  pwm1 = pwm2 = pwm3 = pwm4 = Speed;
  Enabled();
}

void turn_R() {  //右转
  pwm1 = pwm2 = 175;
  pwm3 = pwm4 = -175;
  Enabled();
}

void turn_L() {  //左转
  pwm1 = pwm2 = -150;
  pwm3 = pwm4 = 150;
  Enabled();
}

void brake(){   //前进刹车
  pwm1 = pwm2 = -255;
  pwm3 = pwm4 = -255;
  Enabled();
}

void brake_F(){  //后退刹车
  pwm1 = pwm2 = 255;
  pwm3 = pwm4 = 255;
  Enabled();
}

void brake_L() { //旋转刹车左
  pwm1 = pwm2 = -255;
  pwm3 = pwm4 = 255;
  delay(10);
}

void brake_R() { //旋转刹车右
  pwm1 = pwm2 = 255;
  pwm3 = pwm4 = -255;
  delay(10);
}

void Stop() {   //停车
  pwm1 = pwm2 = 0;
  pwm3 = pwm4 = 0;
  Enabled();
}

void Specify_angle_L_90() { //左转90 偏差10-15
  while (1) {
    if (Val1<=-1400 && Val2<=-1400 && Val3>=1400 && Val4>=1400) {
      brake_L(); delay(50);
      break;
    }
    turn_L();
  }
}

void Specify_angle_L_180() { //左转90 偏差10-15
  while (1) { //2975
    if (Val1<=-2750 && Val2<=-2750 && Val3>=2750 && Val4>=2750) {
      brake_L(); delay(50);
      break;
    }
    turn_L();
  }
}

void Specify_distance(int distance) { //指定距离
  while (1) {
    if (distance1 > distance && distance2 > distance && distance3 > distance && distance4 > distance) {
      brake(); delay(50);
      break;
    }
    forward(75);
  }
}

//……………………Finish……………………//


/*……………………………………………………
函数类型：循迹运动
函数功能：循迹采样、调速
输入：无
……………………………………………………*/
uint8_t Read(){ //读取循迹数值
  uint8_t tmp;
  tmp = 0;
  tmp |= (digitalRead(D1)  << 0);
  tmp |= (digitalRead(D2)  << 1);
  tmp |= (digitalRead(D3)  << 2);
  tmp |= (digitalRead(D4)  << 3);
  tmp |= (digitalRead(D5)  << 4);
  tmp |= (digitalRead(D6)  << 5);
  tmp |= (digitalRead(D7)  << 6);
  tmp |= (digitalRead(D8)  << 7);
  return tmp;
}

int tracking_error(){ //根据读取的循迹数值返回误差值
  num = Read();
  switch (num)
  { //8765 4321
    /*…………左…………*/
    case 0xFE:  //普通巡线时与90度转弯时都有可能遇到
      return -35;  //1111 1110
    case 0xFC:  //普通巡线时与90度转弯时都有可能遇到
      return -30;  //1111 1100
    case 0xFD:
      return -25;  //1111 1101
    case 0xF9:
      return -20;  //1111 1001
    case 0xFB:
      return -15;  //1111 1011
    case 0xF3:
      return -10;  //1111 0011
    case 0xF7:
      return -5;  //1111 0111
    /*…………右…………*/
    case 0xEF:
      return 5;   //1110 1111
    case 0xCF:
      return 10;   //1100 1111
    case 0xC7:
      return 10;   //11000111
    case 0x9F:
      return 25;   //1001 1111
    case 0xDF:
      return 20;   //1101 1111
    case 0xBF:
      return 30;   //1011 1111
    case 0x3f:  //普通巡线时与90度转弯时都有可能遇到
      return 30;   //0011 1111
    case 0x80:  //普通巡线时与90度转弯时都有可能遇到
      return 35;  //0111 1111

    /*…………Y字路口…………*/
    case 0x7E:    //0111 1110
    case 0xbd:    //1011 1101
    case 0x3e:    //0011 1110
    case 0x7c:    //0111 1100
    case 0x3c:    //0111 1100
    case 0x38:    //0011 1000
    case 0x1c:    //0001 1100
    case 0x0e:    //0000 1110
    case 0x70:    //0111 0000
    case 0x6c:    //0110 1100
    case 0x36:    //0011 0110
    case 0x1A:    //0001 1010
    case 0x58:    //0101 1000
    case 0x64:    //0110 0100
    case 0x26:    //0010 0110
      turn_L(); return 0;
  /*…………左直角转弯…………*///左11100000
    case 0xF8:  //1111 1000
    case 0xF0:  //1111 0000
    case 0xE0:  //1110 0000
    case 0xc0:  //1100 0000
      turn_L();delay(300);
      return -100;
    /*…………右直角转弯…………*/
    case 0x07:  //0000 0111
    case 0x0F:  //0000 1111
    case 0x1F:  //0001 1111
    case 0x7F:  //0111 1111
      turn_R();delay(300);
      return 100;
    case 0xE7:
      return 0;    //1110 0111 正中间
    case 0xc3:
      return 0;    //1100 0011
    case 0xff:
      return 300;  //1111 1111 走空
    case 0x00:
      if(m1==2)m1++;
      return 200;  //0000 0000 全在线
  }
}

int Tracking_num() {
  int num = 0;
  if (digitalRead(D1) == LOW)num++;
  if (digitalRead(D2) == LOW)num++;
  if (digitalRead(D3) == LOW)num++;
  if (digitalRead(D4) == LOW)num++;
  if (digitalRead(D5) == LOW)num++;
  if (digitalRead(D6) == LOW)num++;
  if (digitalRead(D7) == LOW)num++;
  if (digitalRead(D8) == LOW)num++;
  return num;
}

void xunji(){ //根据误差值进行调速
  state = Tracking_num();
  static float robotSpeed=50; //初始速度
  float KP = 6;    //比例系数
  float KD = 15;     //微分系数

  static float err;
  static float last_err;
  err = tracking_error(); //读取误差值
  
  if(err==300 || err==200)forward(120);
  if ( err <= 100 && err >= -100 ) {
      err = constrain(err, -25, 25); //前往寻找卸货车位时减小误差值区间
    if (state >= 3 && last_state >= 3 && m1 == 1)forward(100);
    float val = (err * KP) + (err - last_err) * KD; //转弯环
    last_err = err; //保存误差
    int motor_left = constrain((robotSpeed + val), -100, 100);
    int motor_right = constrain((robotSpeed - val), -100, 100);
    pwm1 = pwm2 = motor_left * 2.55;  //输出参数为百分制
    pwm3 = pwm4 = motor_right * 2.55; //这里乘以2.55作为255的比例数
    Enabled();
    delay(1); //循环采样时间
  }
  last_err = err;
}
//……………………Finish……………………//


/*……………………………………………………
函数类型：其他硬件功能
函数功能：舵机转角、按键识别、RGB灯使用等
输入：无
……………………………………………………*/
void servo(int pos1,int pos2){
  transverse.write(pos1);
  vertical.write(pos2);
}


/*……………………………………………………
函数名：Print(int mod)
函数功能：数据查看器
输入：0-4 分别代表不同数据
0:速度  1:脉冲数  2:pwm
3:pid更改量  4:pid误差值  5:角度
6:距离
……………………………………………………*/
void Print(int mod) { //查看曲线
  if (mod == 0) {
    Serial.print(velocity1);
    Serial.print(",");
    Serial.print(velocity2);
    Serial.print(",");
    Serial.print(velocity3);
    Serial.print(",");
    Serial.println(velocity4);
  } else if (mod == 1) {
    Serial.print(Val1);
    Serial.print(",");
    Serial.print(Val2);
    Serial.print(",");
    Serial.print(Val3);
    Serial.print(",");
    Serial.println(Val4);
  } else if (mod == 2) {
    Serial.print(pwm1);
    Serial.print(",");
    Serial.print(pwm2);
    Serial.print(",");
    Serial.print(pwm3);
    Serial.print(",");
    Serial.println(pwm4);
  } else if (mod == 3) {
    Serial.print(vel1);
    Serial.print(",");
    Serial.print(vel2);
    Serial.print(",");
    Serial.print(vel3);
    Serial.print(",");
    Serial.println(vel4);
  } else if (mod == 4) {
    Serial.print(error1);
    Serial.print(",");
    Serial.print(error2);
    Serial.print(",");
    Serial.print(error3);
    Serial.print(",");
    Serial.println(error4);
  } else if (mod == 5) {
    Serial.print(angle1);
    Serial.print(",");
    Serial.print(angle2);
    Serial.print(",");
    Serial.print(angle3);
    Serial.print(",");
    Serial.println(angle4);
  } else if (mod == 6)  {
    Serial.print(distance1);
    Serial.print(",");
    Serial.print(distance2);
    Serial.print(",");
    Serial.print(distance3);
    Serial.print(",");
    Serial.println(distance4);
  }
}

void setup() {
  // put your setup code here, to run once:
  Motor_Init();
  LCD_Init();
  Interrupt_Init();
  tracking_Init();
//  Servo_Init();
  Serial.begin(9600);
}

void mission1(){
  
}

void loop() {
  // put your main code here, to run repeatedly:
  xunji();
  
}
