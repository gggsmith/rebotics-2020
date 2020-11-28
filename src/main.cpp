#include <Arduino.h>
#include <SparkFun_TB6612.h>

// Драйвер моторов
#define AIN1 9
#define BIN1 7
#define AIN2 10
#define BIN2 6
#define PWMA 11
#define PWMB 5
#define STBY 8
// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

Motor motorA = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorB = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup() {
  Serial.begin(9600);
}

void loop() {

  motorB.drive(255,1000);
  motorB.drive(-255,1000);
  motorB.brake();
  delay(10000);
  
}