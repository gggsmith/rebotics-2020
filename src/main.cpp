#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <Encoder.h>
#include <Servo.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <stdio.h>

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


/**
 * Создает натяжение с заданными параметрами:
 * Натянет мотор до числа тиков encoder \c stopAt
 * Крутить будет с равномерно увеличивающейся скоростью, которая будет измениться раз в \c interval ms
 * на \c speedStep pwm, но не превышая скорость \c maxSpeed.
 */
void createTorsion(int stopAt, int maxSpeed, int speedStep, int interval) {
    motor.brake();  

    debugln("Start creating torsion, will stop at %d. max speed: %d; speed step: %d; interval: %d", stopAt, maxSpeed, speedStep, interval);

    int cur = 0;
    encoder.write(cur);

    int speed = 30;
    while (cur < stopAt)
    {
      speed = min(min(speed + speedStep, maxSpeed), 255);

      debugln("Next speed: %d", speed);

      // чтобы вовремя остановиться, если достигнем число тиков в текущем интервале
      int duration = 0;
      while (cur < stopAt && duration < interval)
      {
        motor.drive(speed, 5);
        duration += 5;
        cur = encoder.read();
      }
      
      debugln("Moved to: %d, %d encoder ticks left", cur, (stopAt - cur)); 
    }

    debugln("Torsion created. Going to standby mode");

    motor.brake();
    motor.standby();
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
const int maxSpeed = 150;


double Setpoint, Input, Output;
double kp = 0.916, ki = 0.54, kd = 0;

PID pid = PID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
PID_ATune aTune(&Input, &Output);

boolean tuning = false;
byte ATuneModeRemember = 2;
double aTuneStep = maxSpeed, aTuneNoise = 5, aTuneStartValue = 0;

unsigned int aTuneLookBack = 10;
long position = 0L;

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = pid.GetMode();
  else
    pid.SetMode(ATuneModeRemember);
}


void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    Output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}


void setup() {
  Serial.begin(9600);
  servo.attach(SERVO);

  Input = encoder.read();
  Setpoint = encoderTarget;

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-maxSpeed, maxSpeed); 
  pid.SetSampleTime(100);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }

  position = encoder.read();
}


int i = 0;
int debugFreq = 10000;

void loop() {
  i++;
  position += encoder.readAndReset();
  
  Input = position;
  if (i % debugFreq == 0) {
    Serial.print("Encoder: "); 
    Serial.print(position < 0 ? "-" : ""); 
    debugFloat(abs(position)); 
    Serial.println();
  }

  if(tuning)
    {
      byte val = (aTune.Runtime());
      if (val!=0)
      {
        tuning = false;
      }
      if(!tuning)
      { //we're done, set the tuning parameters
        kp = aTune.GetKp();
        ki = aTune.GetKi();
        kd = aTune.GetKd();
        debugFloat(kp);
        debugFloat(ki);
        debugFloat(kd);

        pid.SetTunings(kp, ki, kd);
        AutoTuneHelper(false);
      }
    }
    else pid.Compute();
  
  if (i % debugFreq == 0 && tuning) {
    debugFloat(Output);
    Serial.print("kp: ");
    debugFloat(kp);
    Serial.print("; ki: ");
    debugFloat(ki);
    Serial.print("; kd: ");
    debugFloat(kd);
  }

  motor.drive(Output);

  if (i % debugFreq == 0) {
    i = 0;
  }

  // Input = encoder.read();
  // debugln("Encoder: %d", encoder.read());
  
  // pid.Compute();

  // debugFloat(Output);
  // debugFloat(Outpui);
  // debugFloat(Outpud);
  // debugln("");

  // motor.drive(Output);

  // debugln("loop %d", i++);
  // if (readyToThrow(encoderTarget)) {
  //   liftUp();

  //   stopPulling();

  //   putDown();
  // }
}
