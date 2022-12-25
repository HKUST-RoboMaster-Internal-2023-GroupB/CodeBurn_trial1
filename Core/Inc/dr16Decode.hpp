float motorDecode(float maxRPM, uint16_t channelData){
    float mSpeed;
    mSpeed = channelData - 1024;
    mSpeed = (mSpeed/660)*maxRPM;
    
    //clamping target
    if (mSpeed > maxRPM){
        mSpeed = maxRPM;
    } else if (mSpeed < -maxRPM){
        mSpeed = -maxRPM;
    }
    return mSpeed;
}