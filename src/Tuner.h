#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <SparkFun_TB6612.h>
#include <Encoder.h>
#include <Coefficients.h>

class Tuner {
    
    public:
    
        Tuner(Motor* motor, Encoder* encoder, Coefficients coeff, 
double step, double noise, double startValue);

        void setup(int encoderTarget, int maxSpeed);

        bool tune();

        Coefficients getCoeff();

    private:

        Motor* motor;
        Encoder* encoder;
        PID* pid;
        PID_ATune* autoTune;
        
        double step;
        double noise;
        double startValue;
        Coefficients initialCoeff;
        
        double Setpoint, Input, Output;
        
        bool tuning;

        long position;
        long iteration;
        long debugFreq;

        void debugFloat(float f);
};