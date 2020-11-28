#include <Tuner.h>

Tuner::Tuner(Motor* motor, Encoder* encoder, Coefficients coeff, 
double step, double noise, double startValue, bool pi)
{
    this->motor = motor;
    this->encoder = encoder;
    this->iteration = 0;
    this->debugFreq = 5000;
    this->tuning = true;
    this->initialCoeff = coeff;
    this->step = step;
    this->noise = noise;
    this->startValue = startValue;
    this->pi = pi;
}


void Tuner::setup(int encoderTarget, int maxSpeed) 
{
    double kp = initialCoeff.kp, ki = initialCoeff.ki, kd = initialCoeff.kd;
    

    pid = new PID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
    autoTune = new PID_ATune(&Input, &Output);

    byte ATuneModeRemember = 2;
    double aTuneStep = maxSpeed, aTuneNoise = noise, aTuneStartValue = startValue;

    unsigned int aTuneLookBack = 10;

    Setpoint = encoderTarget;
    Input = this->encoder->read();
    Output = aTuneStartValue;

    pid->SetOutputLimits(-maxSpeed, maxSpeed); 
    pid->SetSampleTime(10);
    pid->SetMode(ATuneModeRemember);

    autoTune->SetNoiseBand(aTuneNoise);
    autoTune->SetOutputStep(aTuneStep);
    autoTune->SetLookbackSec((int)aTuneLookBack);
    autoTune->SetControlType(pi ? 0 : 1);

    position = Input;   
}

void Tuner::debugFloat(float f) 
{
    char buff[20];
    sprintf(buff, "%d.%d", (int) f, (int) abs((f - (int) f) * 10000));

    Serial.print(buff);
}

bool Tuner::tune() 
{
    iteration++;
    position += encoder->readAndReset();
    
    Input = position;

    if (tuning) {
        byte val = (autoTune->Runtime());
        if (val!=0) {
            tuning = false;
        }
    }

    if (iteration % debugFreq == 0 || !tuning) {
        Serial.print("Iteration: "); 
        Serial.print(iteration);
        Serial.print("; Encoder: "); 
        Serial.print(position < 0 ? "-" : ""); 
        debugFloat(abs(position)); 
        Serial.print("; PWM: ");
        debugFloat(Output);
        if (!tuning) {
            Coefficients cur = Tuner::getCoeff();

            Serial.print("; Kp: ");
            debugFloat(cur.kp);
            Serial.print("; Ki: ");
            debugFloat(cur.ki);
            Serial.print("; Kd: ");
            debugFloat(cur.kd);
        }
        Serial.print("; tuning: ");
        Serial.print(tuning ? "true" : "false");
        Serial.println();
    } 
    
    if (tuning) {
        motor->drive(Output);
    }

    return !tuning;
}

Coefficients Tuner::getCoeff()
{
    return Coefficients({autoTune->GetKp(), autoTune->GetKi(), autoTune->GetKd()});
}