#include <Ultrasonic.h>
#include <Servo.h>
//#include <MPU6050.h>
//#include <Adafruit_PWMServoDriver.h>
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
Ultrasonic ultrasonic(35, 37);
//MPU6050 mpu(0x68);
// define for multi
#define A 48
#define B 46
#define C 44
#define D 42
#define data A0
//#define for driver1
#define i1 22
#define i2 2//////////////////
//define for driver2
#define i3 26
#define i4 4
//define for driver3
#define i21 24
#define i22 3
//#define for driver4
#define i23 28
#define i24 5
//for led on shild
//#define led1 A7
//#define led2 A8

//sw dip on shild ;
int sw = A2;
//srf
int dis;
//for tcrt
int linee [16] ,  blk = 500 ;//blk=black kam     linee=for analog input tcrt
int linemax [16];//for max input tcrt;
bool lineD[16] ;//for digital input tcrt
//for gy521
bool dmpReady = false;
uint8_t deviceStatus; //device status , 0 = success ,
uint16_t packetSize; //expected DMP packet size (defult 42) -- ?
uint16_t fifoCount; //count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO buffer storage
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
bool ticket = 1;
float yaw , output_pid, roll;
int halat, degree;
int Speed;
void motor(int pwm1, int pwm2) {
  // pwm1 for left motors
  // pwm2 for right motors
  pwm1 += output_pid;
  pwm2 -= output_pid;
  if (pwm1 >= 254) pwm1 = 254;
  if (pwm2 >= 254) pwm2 = 254;
  if (pwm1 <= -254) pwm1 = -254;
  if (pwm2 <= -254) pwm2 = -254;
  if (pwm1 >= 0 && pwm2 >= 0)
  {
    digitalWrite(i1, 1);
    digitalWrite(i3, 1);
    analogWrite(i2, 255 - pwm1);
    analogWrite(i4, 255 - pwm1);


    digitalWrite(i21, 1);
    digitalWrite(i23, 1);
    analogWrite(i22, 255 - pwm2);
    analogWrite(i24, 255 - pwm2);

  }
  else if (pwm1 < 0 && pwm2 > -1)
  {
    digitalWrite(i1, 0);
    digitalWrite(i3, 0);
    analogWrite(i2,  -1 * ( pwm1));
    analogWrite(i4, -1 * ( pwm1));

    digitalWrite(i21, 1);
    digitalWrite(i23, 1);
    analogWrite(i22, 255 - pwm2);
    analogWrite(i24, 255 - pwm2);
  }
  else if (pwm1 > -1  && pwm2 < 0)
  {
    digitalWrite(i1, 1);
    digitalWrite(i3, 1);
    analogWrite(i2, 255 - (-(pwm2)));
    analogWrite(i4, 255 - (-(pwm2)));

    digitalWrite(i21, 0);
    digitalWrite(i23, 0);
    analogWrite(i22, pwm1);
    analogWrite(i24, pwm1);
  }
  else if (pwm1 < 0 && pwm2 < 0)
  {
    digitalWrite(i1, 0);
    digitalWrite(i3, 0);
    analogWrite(i2, 255 - (-(pwm2)));
    analogWrite(i4, 255 - (-(pwm2)));

    digitalWrite(i21, 0);
    digitalWrite(i23, 0);
    analogWrite(i22, -(pwm1));
    analogWrite(i24, -(pwm1));
  }
}
void mult(int addad)
{
  digitalWrite(D, (addad / 8) % 2);
  digitalWrite(C, (addad / 4) % 2);
  digitalWrite(B, addad / 2 % 2);
  digitalWrite(A, addad % 2);
}
void adadgiri_tcrt () {
  mult (1 );
  linee [1] = analogRead((data));
  mult (15 );
  linee [2] = analogRead((data));
  mult (2 );
  linee [3] = analogRead((data));
  mult (14);
  linee [4] = analogRead((data));
  mult (6);
  linee [5] = analogRead((data));
  mult (11);
  linee [6] = analogRead((data));
  mult (12);
  linee [7] = analogRead((data));
  mult (7);
  linee [8] = analogRead((data));
  mult (0);
  linee [9] = analogRead((data));
  mult (4);
  linee [10] = analogRead((data));
  mult (3);
  linee [11] = analogRead((data));
  mult (8);
  linee [12] = analogRead((data));
  mult (5);
  linee [13] = analogRead((data));
  mult (10);
  linee [14] = analogRead((data));
  mult (9);
  linee [15] = analogRead((data));
  for ( int y = 1; y < 16; y++) {
    if ( linee [y] > blk ) {
      lineD[y] = 1;
      chop(linee[y], 50);

    }//((linemax[y]*7)/10)
    else  {
      lineD[y] = 0;
      chop(linee[y], 50);
    }

  }
  Serial.println();


  /*Serial.print(linee[11]);
    Serial.print("\t");
    Serial.print(linee[12]);
    Serial.print("\t");
    Serial.print(linee[13]);
    Serial.print("\t");
    Serial.println();
    delay(200);
  */
}
//void gy521()
//{
//  mpu.resetFIFO();
//  //if DMP not ready don't do anything
//  if (!dmpReady)
//  {
//    if (ticket == 1)
//    {
//      Serial.println("MAIN LOOP: DMP disabled");
//      ticket = 0 ;
//    }
//    else
//      return;
//  }
//  else
//  {
//    if (fifoCount == 1024)
//    {
//      mpu.resetFIFO();
//      Serial.println("FIFO overflow");
//    }
//    else
//    {
//      //waiting until get enough
//      while (fifoCount < packetSize)
//        fifoCount = mpu.getFIFOCount();
//
//      mpu.getFIFOBytes(fifoBuffer, packetSize);
//      fifoCount -= packetSize ;
//
//      if (fifoCount > 2) {
//        ////// clear fifo buffer
//      }
//
//      mpu.dmpGetQuaternion(&q, fifoBuffer);
//      mpu.dmpGetGravity(&gravity, &q);
//      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//      yaw = ypr[0] * 180 / M_PI;
//      roll = ypr[2] * 180 / M_PI;
//      /////////////print ypr data/////////////
//      /*Serial.print(ypr[0] * 180/M_PI);  //yaw
//        Serial.print("\t");
//        Serial.print(ypr[1] * 180/M_PI);  //pitch
//        Serial.print("\t");
//        Serial.println(ypr[2] * 180/M_PI);  //roll
//      *////////////////////////////////////////
//    }
//  }
//}
void lcd_print(int x, int y, double adad ) {

  int adad1 = adad * 10;
  lcd.setCursor(x, y);
  if (adad1 < 0) {
    adad1 = adad1 * -1;
    lcd.print("-");
  }
  else {
    lcd.print("+");
  }
  lcd.print(adad1 / 1000);
  lcd.print((adad1 / 100) % 10);
  lcd.print((adad1 / 10) % 10);
  lcd.print(".");
  lcd.print(adad1 % 10);
}
void  line  (int Speed)
{
  adadgiri_tcrt ();
  if (lineD[8] == 1)motor(Speed, Speed);
  //else if (lineD[7] == 1)motor( 100, -100);
  //else if (lineD[9] == 1)motor(-100, 100);
    else if (lineD[6] == 1)motor(130, -130);
    else if (lineD[10] == 1)motor(-130, 130);
  //  else if (lineD[5] == 1)motor(-180, 180);
  //  else if (lineD[11] == 1)motor(200, -200);
  //  else if (lineD[4] == 1)motor(-225, 225);
  //  else if (lineD[12] == 1)motor(225, -225);
  //  else if (lineD[3] == 1)motor(-255, 255);
  //  else if (lineD[13] == 1)motor(255, -255);
  // else if (lineD[1] == 1)motor(250, -250);
  //else if (lineD[15] == 1)motor(-250, 250);
  else
  {
    motor(70, 70);
    //digitalWrite(led1,1);
    //delay(5);
  }
  // digitalWrite(led1,0);
}
void max_tcrt()
{
  adadgiri_tcrt();
  for (int i = 1; i <= 16; i++)
  {
    if (linemax[i] < linee[i]) linemax[i] = linee[i];
  }
}
void pid_rescue(int p , float input_gyro) {
  output_pid = p * input_gyro;
}
void rescue() {

  //  gy521();
  if (dis < 20 && halat == 1)
  {
    motor(0, 0);
    delay(1000);
    degree += 90;
    halat = 0;
  }
  degree = degree % 360;
  if (dis > 20) halat = 1;
  int pid_input = yaw - degree;
  if (pid_input > 180) pid_input -= 360;
  if (pid_input < -180) pid_input += 360;
  pid_rescue(5 , pid_input);
  motor(100, 100);
  lcd_print(0, 0, yaw);
  lcd_print(0, 1, dis);
  lcd_print(7, 0, pid_input);
  srf();
}
void servo_setpwm(int angle, int servonum) {
  int Setup = 1;
  if (Setup == 1) {
    Setup = 0;
    delay(10);
  }
  //  pwm.setPWM(servonum, 0, (angle * 3.2) + 28);
}
void srf()
{
  dis = ultrasonic.read();
  Serial.println(dis);
  lcd_print(0, 1, dis);
}
void ramp()
{
  //  gy521();
  line(Speed);
  if (roll > 20)Speed = 250;
  else if (roll < -20) Speed = 100;
  else Speed = 175;
  lcd_print(70, 0, roll);
}
void chop(double adad, int dopom  ) //dopom=delay of Print on monitor
{
  if (millis() % dopom < 1)
  {
    Serial.print(adad);
    Serial.print("/t");
  }
}
void setup() {

  Wire.begin();
  //MULTI
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);

  // DRIVER 1
  pinMode(i1, OUTPUT);
  pinMode(i2, OUTPUT);
  pinMode(i3, OUTPUT);
  pinMode(i4, OUTPUT);

  //DRIVER 2
  pinMode(i21, OUTPUT);
  pinMode(i22, OUTPUT);
  pinMode(i23, OUTPUT);
  pinMode(i24, OUTPUT);
  //sw
  pinMode(sw, INPUT);
  //  SRF 05
  //  lcd16 * 2
  lcd.init();
  lcd.backlight();
  //  gy521
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 48; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(9600);
  //  mpu.initialize();
  //  Serial.println(mpu.testConnection() ? F("MPU6050 connection test successed ") : F("MPU6050 connection test failed"));
  //  deviceStatus = mpu.dmpInitialize();
  /////////////Offsets/////////////
  //  mpu.setXGyroOffset(20);
  //  mpu.setYGyroOffset(-41);
  //  mpu.setZGyroOffset(48);
  //  mpu.setXAccelOffset(-3729);
  //  mpu.setYAccelOffset(3067);
  //  mpu.setZAccelOffset(1704);
  /////////////////////////////////
  if (deviceStatus == 0)
  {
    Serial.println("DMP initialization success, now enable DMP for use");
    //    mpu.setDMPEnabled(true);

    //wait for first interrupt . currently just leave it false automatically
    dmpReady = true;
    Serial.println("DMP is ready to use.");
    //    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    //ERROR! , device status !=0 when initializing DMP
    Serial.print("DMP initialization failed when using MPU6050 library:");
    if (deviceStatus == 1)
      Serial.println(" intial memory load failed");
    else if (deviceStatus == 2)
      Serial.println(" failed to update DMP configuration");
    else
    {
      Serial.print(" unknow error with code: ");
      Serial.println(deviceStatus);
    }
  }
  //for led
  //  pinMode(led1, OUTPUT);
  //  pinMode(led2, OUTPUT);
  //for servo
  //  pwm.begin();
  //  pwm.setPWMFreq(50);
}
void loop()
{
  line(100);
  Serial.print("left ") , Serial.print(linee[7]) , Serial.print("middle ") , Serial.print(linee[8]), Serial.print("right ") , Serial.println(linee[9]);
}
