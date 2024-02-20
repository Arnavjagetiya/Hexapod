#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <PID_v1.h>

#define SERVOMIN  150
#define SERVOMAX 600

Servo Servo1;
Servo Servo2;

#define SERVO_FREQ 60

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  Servo1.attach(3);
  Servo1.attach(5);
}

void loop() {
  start();
  delay(2000);
  stand();
  fourlegs();
  walkforearm3l();
}
void wallkleg3l(int channel, int angle){
  walkforearm3l();
  walkshoulder3l(115);
}

void walkshoulder3l(int angle){
  for (int angle; angle <= 150; angle =+ 35){
    Servo1.write(angle);
    delay(1000);
  }
}

void fourlegs(){
  claw1r(0,125);
  forearm1r(1,200);
  shoulder1r(2,220);
  claw3r(6,20);
  forearm3r(7,200);
  shoulder3r(8,70);
  claw1l(9,120);
  forearm1l(10,50);
  shoulder1l(11,-50);
  claw3l(15,130);
  forearm3l(12,50);
  shoulder3l(14,180);
}

void start(){
  claw1r(0,120);
  claw2r(2,120);
  claw3r(4,10);
  claw1l(6,120);
  claw2l(8,120);
  claw3l(10,120);
  forearm1r(1,100);
  forearm2r(3,100);
  forearm3r(5,120);
  forearm1l(7,111);
  forearm2l(9,120);
  forearm3l(11,111);
}

void stand(){
  claw1r(0,125);
  claw2r(2,120);
  claw3r(4,20);
  claw1l(6,120);
  claw2l(8,130);
  claw3l(10,130);
  forearm1r(1,200);
  forearm2r(3,90);
  forearm3r(5,200);
  forearm1l(7,50);
  forearm2l(9,50);
  forearm3l(11,50);
}

void walkforearm3l(){ 
  forearm3l(11,80);
  delay(500);
  forearm3l(11,50);
}

void claw1r(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void forearm1r(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void claw2r(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void forearm2r(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void claw3r(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void forearm3r(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void claw1l(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void forearm1l(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void claw2l(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void forearm2l(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void claw3l(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void forearm3l(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void shoulder1r(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void shoulder3r(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  pwm.setPWM(channel, 0, Output);
}

void shoulder1l(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  Servo1.write(Output);
  pwm.setPWM(channel, 0, Output);
}

void shoulder3l(int channel, int angle) {
  double kp = 1.125;
  double ki = 0.0;
  double kd = 0.2;
  double Setpoint = 90;
  double Input;
  Input = 10;
  double Output = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.Compute();
  Servo2.write(Output);
  pwm.setPWM(channel, 0, Output);
}