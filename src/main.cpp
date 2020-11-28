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


const int encoderTarget = 10000;
const int maxSpeed = 230;


double Input, Output, Setpoint;

// double kp = 0.1379, ki = 0.113, kd = 0.4181;
double kp = 0.914, ki = 0.48, kd = 0;
// double kp = 0.916, ki = 0.54, kd = 0;

PID pid = PID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);


Tuner* tuner = new Tuner(& motor, & encoder, {kp, ki, kd}, maxSpeed, 5, 0, true);
bool tuning = false;

const int aims[6] = {1, 1, 1, 2, 2, 3};
const int targets[4] = {-1, 3000, 5000, 8000};


void setup() {
  Serial.begin(9600);
  servo.attach(SERVO);

  Input = 0;
  Setpoint = encoderTarget;

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-maxSpeed, maxSpeed); 
  pid.SetSampleTime(50);

  if(tuning) {
    tuner->setup(encoderTarget, maxSpeed);
  }
  
  motor.standby();
}

long iteration = 0;

int aim = -1;
int aimTarget = -1;
int ballId = 0;
long startedAt = 0;

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

  if (ballId >= 6) {
    return; // всех отстреляли
  }

  iteration++;

  // Событие смены шарика
  if (aim < 0 || aimTarget < 0) {
    aim = aims[ballId];
    aimTarget = targets[aim];

    encoder.write(0);

    Setpoint = aimTarget;
    debugln("Ball: %d; Aim: %d; Encoder target: %d", ballId, aim, aimTarget);
    
    delay(3000); // задержка между заводами на всякий случай
    startedAt = millis();
  }

  Input = encoder.read();
  pid.Compute();

  motor.drive(Output);

  bool ready = millis() - startedAt > 3000;
  if (iteration % 1000 == 0 || ready) {
    Serial.print("Iteration: ");
    Serial.print(iteration);
    Serial.print("; Encoder: ");
    Serial.print(Input);
    Serial.print("; PWM: ");
    debugFloat(Output);
    Serial.println();
  }

  if (ready) {
    Serial.print("Completed. Diff: ");
    debugFloat(abs(Input - aimTarget));
    Serial.println();

    Serial.flush();
    
    motor.standby();

    // servo.write(-180); // поднять серво
    delay(1000); 

    // servo.write(180); // вернуть серво на место
    delay(1000); 

    ballId++;
    aim = -1;
    iteration = 0;
  }


  // debugln("loop %d", i++);
  // if (readyToThrow(encoderTarget)) {
  //   liftUp();

  //   stopPulling();

  //   putDown();
  // }
}
