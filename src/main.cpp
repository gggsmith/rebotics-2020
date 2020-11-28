#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <Encoder.h>
#include <Servo.h>

// Драйвер моторов
#define AIN1 9
#define BIN1 7
#define AIN2 10
#define BIN2 6
#define PWMA 11
#define PWMB 5
#define STBY 8
const int offsetA = 1;
const int offsetB = 1;

Motor motorB = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


// Энкодер мотора
// Для лучшей производительности должны быть подключены в interruptable pin (хотя бы a-фаза)
#define APHASE 2
#define BPHASE 3

Encoder encoderB = Encoder(APHASE, BPHASE);



// Серво-привод
#define SERVO 4
Servo servo;


void setup() {
  Serial.begin(9600);
  servo.attach(SERVO);
}

void loop() {

  servo.write(0);
  delay(1000);
  servo.write(180);
  delay(1000);


  motorB.drive(255,1000);
  Serial.println(encoderB.read());
  motorB.drive(-255,1000);
  Serial.println(encoderB.read());
  motorB.brake();
  delay(5000);
  
}