#ifndef PID_H // header guard
#define PID_H



namespace PID{ 
    struct PID
    {
        float Kp = 50;
        float Ki = 0.1;
        float Kd = 0.01;

        float pout;
        float iout;
        float dout;
        float out;

        float error;
        float lastError;
        float integral;

        float dt = 0.001f;
    };
    class PIDController{
        private:
            PID pidParam;
            float target;
            float margin = 2000;
            float limit = 16000;
            float reductionRatio = 3591/187;
            float integralLimit = 800;
        public:
        //setter functions
            void initializeParam(float p, float i, float d, float targetVal, float clampMargin, float lim,float integralLim){
                pidParam.Kp = p;
                pidParam.Ki = i;
                pidParam.Kd = d;
                target = targetVal;
                margin = clampMargin;
                limit = lim;
                integralLimit = integralLim;
            }
            void setP(float p){
                pidParam.Kp = p;
            }
            void setI(float i){
                pidParam.Ki = i;
            }
            void setD(float d){
                pidParam.Kd = d;
            }
            void setReductionRatio(float ratio){
                reductionRatio = ratio;
            }
            void setError(float error){
                pidParam.error = error;
            }
            void setTarget(float tg){
                target = tg;
            }
            void setMargin(float mg){
                margin = mg;
            }
            void setDt(float dt){
                pidParam.dt = dt;
            }
            void setLimit(float lim){
                limit = lim;
            }
            void calcNewError(float cValue){
                float currentValue = cValue/reductionRatio;
                pidParam.lastError = pidParam.error;
                pidParam.error = target - currentValue;

                pidParam.pout = pidParam.Kp * pidParam.error;

                pidParam.integral += pidParam.Ki * pidParam.error * pidParam.dt;
                pidParam.iout = pidParam.integral;
                
                pidParam.dout = pidParam.Kd * (pidParam.error - pidParam.lastError) / pidParam.dt;
                
                pidParam.out = pidParam.pout + pidParam.iout + pidParam.dout;


                // integral clamping
                if ((pidParam.integral > integralLimit) or (pidParam.integral < -integralLimit)){
                    // checking signs (error and output)
                    if (pidParam.integral < 0){
                        pidParam.integral = -integralLimit;
                    } else if (pidParam.integral > 0 ){
                        pidParam.integral = integralLimit;
                    }
                }
            }
        //getter functions
            float getOutput(){
                return pidParam.out;
            }
            float getClampedOutput(){
                if (pidParam.out > limit){
                    return limit;
                } else if (pidParam.out < -limit){
                    return -limit;
                } else{
                    return pidParam.out;
                }
            }
            PID getParams(){
                return pidParam;
            }
            float getP(){
                return pidParam.Kp;
            }
            float getI(){
                return pidParam.Ki;
            }
            float getD(){
                return pidParam.Kd;
            }
    };

}//namespace

#endif