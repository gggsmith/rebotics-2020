#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <Encoder.h>
#include <Servo.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <stdio.h>
#include <Tuner.h>

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

Motor motor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Энкодер мотора
// Для лучшей производительности должны быть подключены в interruptable pin (хотя бы a-фаза)
#define APHASE 2
#define BPHASE 3

Encoder encoder = Encoder(APHASE, BPHASE);


// Серво-привод
#define SERVO 4
Servo servo;

template<typename... Args> void debugln(const char * fmt, Args... args) {
  char buff[100];
  sprintf(buff, fmt, args...);

  Serial.println(buff);
}

void debugFloat(float val) {
  char buff[100];
  sprintf(buff, "%d.%d", (int) val, (int) abs((val - (int) val) * 10000));

  Serial.print(buff);
}

void liftUp() {
  servo.write(-180);
  delay(1000);
}

void putDown() {
  servo.write(180);
  delay(1000);
}

void stopPulling() {
  motor.drive(0);
  encoder.write(0);
}

bool readyToThrow(int target) {
  motor.drive(255, 5000);
  return true;
}


const int encoderTarget = 10000;
const int maxSpeed = 100;


double Input, Output, Setpoint;

// double kp = 0.1379, ki = 0.113, kd = 0.4181;
// double kp = 0.914, ki = 0.48, kd = 0;
double kp = 0.916, ki = 0.54, kd = 0;

PID pid = PID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);


Tuner* tuner = new Tuner(& motor, & encoder, {kp, ki, kd}, maxSpeed, 5, 0);
bool tuning = true;


void setup() {
  Serial.begin(9600);
  servo.attach(SERVO);

  Input = 0;
  Setpoint = encoderTarget;

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-maxSpeed, maxSpeed); 
  pid.SetSampleTime(100);

  if(tuning)
  {
    tuner->setup(encoderTarget, maxSpeed);
  }
}



void loop() {
  if (tuning) {
    if (tuner->tune()) {
      motor.standby();
      Serial.println("Completed!");
      Serial.flush();
      exit(0);
    }

    return;
  } 

  Input = encoder.read();
  debugln("Encoder: %d", encoder.read());
  
  pid.Compute();

  debugFloat(Output);
  debugln("");

  motor.drive(Output);


  // debugln("loop %d", i++);
  // if (readyToThrow(encoderTarget)) {
  //   liftUp();

  //   stopPulling();

  //   putDown();
  // }
}
