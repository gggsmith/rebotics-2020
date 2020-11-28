#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <Encoder.h>
#include <Servo.h>
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

template<typename... Args> void debug(const char * fmt, Args... args) {
  char buff[100];
  sprintf(buff, fmt, args...);

  Serial.println(buff);
}


/**
 * Создает натяжение с заданными параметрами:
 * Натянет мотор до числа тиков encoder \c stopAt
 * Крутить будет с равномерно увеличивающейся скоростью, которая будет измениться раз в \c interval ms
 * на \c speedStep pwm, но не превышая скорость \c maxSpeed.
 */
void createTorsion(int stopAt, int maxSpeed, int speedStep, int interval) {
    motor.brake();  

    debug("Start creating torsion, will stop at %d. max speed: %d; speed step: %d; interval: %d", stopAt, maxSpeed, speedStep, interval);

    int cur = 0;
    encoder.write(cur);

    int speed = 30;
    while (cur < stopAt)
    {
      speed = min(min(speed + speedStep, maxSpeed), 255);

      debug("Next speed: %d", speed);

      // чтобы вовремя остановиться, если достигнем число тиков в текущем интервале
      int duration = 0;
      while (cur < stopAt && duration < interval)
      {
        motor.drive(speed, 5);
        duration += 5;
        cur = encoder.read();
      }
      
      debug("Moved to: %d, %d encoder ticks left", cur, (stopAt - cur)); 
    }

    debug("Torsion created. Going to standby mode");

    motor.brake();
    motor.standby();
}


void setup() {
  Serial.begin(9600);
  // servo.attach(SERVO);
}


void loop() {
    createTorsion(10000, 100, 10, 1000);

    delay(60000);
}
