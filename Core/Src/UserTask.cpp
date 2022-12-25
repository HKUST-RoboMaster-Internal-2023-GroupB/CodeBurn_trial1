/**
 * @file DR16.cpp
 * @author JIANG Yicheng (EthenJ@outlook.sg)
 * @brief
 * @version 0.1
 * @date 2022-10-25
 *
 * @copyright This file is only for HKUST Enterprize RM2023 internal competition. All Rights Reserved.
 */

#include "DJIMotor.hpp"
#include "DR16.hpp"
#include "FreeRTOS.h"
#include "can.h"
#include "gpio.h"
#include "main.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include "pid.hpp"
#include "dr16Decode.hpp"


StackType_t uxBlinkTaskStack[128];
StaticTask_t xBlinkTaskTCB;

void blink(void *pvPara)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    uint32_t i = 400;
    while (true)
    {
        for (; i < 500; i++)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
            vTaskDelay(20);
        }
        for (; i > 400; i--)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
            vTaskDelay(20);
        }
    }
}


static volatile float targetPos = 0;

StackType_t uxPIDTaskStack[128];
StaticTask_t xPIDTaskTCB;
void PIDTask(void *pvPara)
{
    DJIMotor::DJIMotor &motor = DJIMotor::getMotor(0x205);
    motor.setCurrentLimit(30000);
    static volatile PID::PID pid;

    while (true)
    {
        pid.lastError = pid.error;
        pid.error = targetPos - motor.getPosition();

        pid.pout = pid.Kp * pid.error;

        pid.integral += pid.Ki * pid.error * pid.dt;
        pid.iout = pid.integral;

        pid.dout = pid.Kd * (pid.error - pid.lastError) / pid.dt;

        pid.out = pid.pout + pid.iout + pid.dout;

        motor.setOutputCurrent(pid.out);
        DJIMotor::sendMotorGroup(1);
        vTaskDelay((uint32_t)(pid.dt * 1000) == 0? 1: (uint32_t)(pid.dt * 1000));
    }
}
static volatile char uartRxData[1]={};
static volatile float step = 1;
StackType_t uxTargetUpdateTaskStack[128];
StaticTask_t xTargetUpdateTaskTCB;
void TargetUpdateTask(void *pvPara)
{
    while (true)
    {
        
        while (DR16::getRcData().rc.ch3 - 1024 <= -200 || DR16::getRcData().rc.ch3 - 1024 >= 200)
            vTaskDelay(1);
            
        while (DR16::getRcData().rc.ch3 - 1024 > -200 && DR16::getRcData().rc.ch3 - 1024 < 200)
            vTaskDelay(1);


        if (DR16::getRcData().rc.ch3 - 1024 > 0)
            targetPos += step;
        else
            targetPos -= step;
        vTaskDelay(1);
    }
}


// target variables
static volatile float target1;
static volatile float target2;
static volatile float target3;
static volatile float target4;


// Watch these two vars in ozone to see DR16 data
static volatile bool varConnect; 
static volatile DR16::RcData varData;
//Max rpm
static volatile float maxRpm = 300;
static volatile float rotationSpeed = 30;
static volatile float motorSpeedX;
static volatile float motorSpeedY;
static volatile float motorSpeedW;
StackType_t uxUpdateDR16TaskStack[128];
StaticTask_t xUpdateDR16TaskTCB;
void UpdateDR16Task(void *pvPara){
    while (true){
            varConnect = DR16::isConnected();
            varData.rc.ch0 = DR16::getRcData().rc.ch0;
            varData.rc.ch1 = DR16::getRcData().rc.ch1;
            varData.rc.ch2 = DR16::getRcData().rc.ch2;
            varData.rc.ch3 = DR16::getRcData().rc.ch3;
            varData.rc.sw1 = DR16::getRcData().rc.sw1;
            varData.rc.sw2 = DR16::getRcData().rc.sw2;
            varData.rc.ch4 = DR16::getRcData().rc.ch4;
            
            //Dr16 decoding to target velocity
            motorSpeedX = motorDecode(maxRpm,varData.rc.ch3);
            motorSpeedY = motorDecode(maxRpm,varData.rc.ch2);
            motorSpeedW = motorDecode(rotationSpeed,varData.rc.ch0);

            //Testing
            target2 = motorSpeedX;




            vTaskDelay(1);
    }
    
}

// can trancieving via motor driver (not tested)

// Checking motor connection 
static volatile bool connect1;
static volatile bool connect2;
static volatile bool connect3;
static volatile bool connect4;

//Test value vars
static volatile float rpm1;
static volatile PID::PID testVars;
static volatile float testP = 50;
static volatile float testI = 0.1;
static volatile float testD = 0.01;



StackType_t uxPIDMotorTaskStack[128];
StaticTask_t xPIDMotorTaskTCB;
void PIDMotorTask(void *pvPara){
    //Get motor reference
    DJIMotor::DJIMotor &motor1 = DJIMotor::getMotor(0x201);
    DJIMotor::DJIMotor &motor2 = DJIMotor::getMotor(0x202);
    DJIMotor::DJIMotor &motor3 = DJIMotor::getMotor(0x203);
    DJIMotor::DJIMotor &motor4 = DJIMotor::getMotor(0x204);

    
    //create motor controllers
    PID::PIDController contr1;
    PID::PIDController contr2;
    PID::PIDController contr3;
    PID::PIDController contr4;

    //set limit
    int limit = 16000;
    motor1.setCurrentLimit(limit);
    motor2.setCurrentLimit(limit);
    motor3.setCurrentLimit(limit);
    motor4.setCurrentLimit(limit);


    while (true){
        // set targets
        contr1.setTarget(target1);
        contr2.setTarget(target2);
        contr3.setTarget(target3);
        contr4.setTarget(target4);
        // update error
        contr1.calcNewError(motor1.getRPM());
        contr2.calcNewError(motor2.getRPM());
        contr3.calcNewError(motor3.getRPM());
        contr4.calcNewError(motor4.getRPM());

        if (varConnect){
            // get output and clamp
            motor1.setOutputCurrent(contr1.getClampedOutput());
            motor2.setOutputCurrent(contr2.getClampedOutput());
            motor3.setOutputCurrent(contr3.getClampedOutput());
            motor4.setOutputCurrent(contr4.getClampedOutput());
        } else{
            motor1.setOutputCurrent(0);
            motor2.setOutputCurrent(0);
            motor3.setOutputCurrent(0);
            motor4.setOutputCurrent(0);
        }

        #pragma region watchVariables
        // setting watch variables
        connect1 = motor1.isConnected();
        connect2 = motor2.isConnected();
        connect3 = motor3.isConnected();
        connect4 = motor4.isConnected();

        rpm1 = motor2.getRPM();

        
        testVars.pout = contr2.getParams().pout;
        testVars.iout = contr2.getParams().iout;
        testVars.dout = contr2.getParams().dout;
        testVars.error = contr2.getParams().error;
        testVars.integral = contr2.getParams().integral;
        testVars.out = contr2.getParams().out;
        testVars.lastError = contr2.getParams().lastError;
        testVars.Kp = contr2.getP();
        testVars.Ki = contr2.getI();
        testVars.Kd = contr2.getD();
        //testing pid vars
        contr2.setP(testP);
        contr2.setI(testI);
        contr2.setD(testD);
        #pragma endregion watchVariables
        


        // send data
        DJIMotor::sendMotorGroup(0);
        vTaskDelay(1);
    }
}





// callback function for uart rx
void uartRxCallback(UART_HandleTypeDef *huart, uint16_t size)
{HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)uartRxData, 1);
  // Some operation right after the reception
  // E.g. Check length, conversion
  // Start UART reception again
}
/**
 * @brief Create user tasks
 */
void startUserTasks()
{
    HAL_UART_RegisterRxEventCallback(&huart2, uartRxCallback);
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)uartRxData, 1);
    DR16::init();
    DJIMotor::init();
    xTaskCreateStatic(blink, "blink", 128, NULL, 0, uxBlinkTaskStack, &xBlinkTaskTCB);
    //xTaskCreateStatic(PIDTask, "PID", 128, NULL, 10, uxPIDTaskStack, &xPIDTaskTCB);
    //xTaskCreateStatic(TargetUpdateTask, "TargetUpdate", 128, NULL, 10, uxTargetUpdateTaskStack, &xTargetUpdateTaskTCB);
    xTaskCreateStatic(UpdateDR16Task, "DR16Update", 128, NULL, 10, uxUpdateDR16TaskStack, &xUpdateDR16TaskTCB);
    xTaskCreateStatic(PIDMotorTask, "PIDMotor", 128, NULL, 10, uxPIDMotorTaskStack, &xPIDMotorTaskTCB);
}

