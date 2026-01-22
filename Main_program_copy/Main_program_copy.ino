//include libraries
#include "HUSKYLENS.h"

//define macros
#define YELLOW_GOAL 1
#define BLUE_GOAL 2
#define BALL 3

//global constants
const byte changeDelay = 65;
const byte LeftMotorPWM = 5;
const byte LeftMotorDIR = 8;
const byte RightMotorPWM = 6;
const byte RightMotorDIR = 7;
const byte LineSensorPin = 14;
const byte BallSensorPin = 15;

//global state variables
byte fieldState = 0x00;
byte robotState = 0x00;
//global objects
HUSKYLENS huskylens;

//function declarations
void forward();
void reverse();
void turnRight();
void turnLeft();
void seeLine();
void seeBall();


void setup() {
  //set up GPIO pins
  pinMode(LeftMotorPWM, OUTPUT);
  pinMode(LeftMotorDIR, OUTPUT);
  pinMode(RightMotorPWM, OUTPUT);
  pinMode(RightMotorDIR, OUTPUT);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(LineSensorPin), seeLine, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BallSensorPin), seeBall, RISING);

  //setup LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);   //(row, col)
  lcd.print("LCD ready");

  Serial.begin(9600);    //initializes the USB peripheral to talk to the PC
  Serial1.begin(9600);    //initializes the Serial peripheral to talk to HuskyLens

  //setup Huksylens
  while(!huskylens.begin(Serial1)) {
    lcd.clear();
    lcd.Cursor(0,0);
    lcd.print("Huskylens offline");
    delay(100);
  }
  lcd.clear();

}

void loop() {
    if (!huskylens.requests) {
    lcd.Cursor(0,0);
    lcd.print("failed request");
  }

  if (huskylens.available) {
    result = huskylens.read();
    lcd.Cursor(0,0);
    lcd.print("ID: " + String(result.command));
    lcd.setCursor(1,0);
    lcd.print("X: " + String(result.xCenter) + "   ");
    lcd.setCursor(2,0);
    lcd.print("Y: " + String(result.yCenter) + "   ");
    delay(500);
    state |= 0x10;    //see block

  } else {
    lcd.setCursor(0,0);
    lcd.print("no block");

  if(fieldState & 0x80) {
    awayFromLine();
  } else if(fieldState & 0x10) {
    makeShot();
  } else if(fieldState & 0x02) {
    getBall();
  } else {
    findBall();
  }
}

void forward(byte speed) {
  if (state != 0x00) {
    analogWrite(LeftMotorPWM, 255);
    digitalWrite(LeftMotorDIR, LOW);
    analogWrite(RightMotorPWM, 255);
    digitalWrite(RightMotorDIR, LOW);
    robotState = 0x01;
  }
}

void reverse(byte speed) {
  analogWrite(LeftMotorPWM, 255);
  digitalWrite(LeftMotorDIR, HIGH);
  analogWrite(RightMotorPWM, 255);
  digitalWrite(RightMotorDIR, HIGH);
  robotState = 0x02;
}

void turnLeft(byte speed) {
  analogWrite(RightMotorPWM, 0);
  analogWrite(LeftMotorPWM, 255);
  digitalWrite(LeftMotorDIR, LOW);
  robotState = 0x03;
}

void turnRight(byte speed) {
  analogWrite(LeftMotorPWM, 0);
  analogWrite(RightMotorPWM, 255);
  digitalWrite(RightMotorDIR, LOW);
  robotState = 0x04;
}

void seeLine() {
  fieldState = 0x80;
}

void seeBall() {
  fieldState = 0x10;
}

void awayFromLine() {
  brake();

  reverse();
  delay(500);
}
