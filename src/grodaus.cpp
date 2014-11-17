/* sample.cpp for TOPPERS/ATK(OSEK) */

// ECRobot++ API
#include "NxtColorSensor.h"
#include "Lcd.h"
#include "Motor.h"
#include "TouchSensor.h"
using namespace ecrobot;

extern "C" {
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* TOPPERS/ATK declarations */
DeclareCounter(SysTimerCnt);
DeclareAlarm(AlarmTaskMotorControl);
DeclareEvent(EventTaskMotorControl);

NxtColorSensor sensor(PORT_1, NxtColorSensor::_LIGHTSENSOR_WHITE);
Motor motorLeft(PORT_A);
Motor motorRight(PORT_C);

TouchSensor touchSensor(PORT_4);

S16 sensorOkLow = 150;
S16 sensorOkHigh = 200;
const S16 sensorDelta = 10;

//const S8 motorForward = Motor::PWM_MAX;
//const S8 motorBackward = Motor::PWM_MIN;
const S8 motorForward = 20;
const S8 motorBackward = -10;

// nxtOSEK hook to be invoked from an ISR in category 2
void user_1ms_isr_type2(void) {
    (void)SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
}

bool findLineLeft(S32 maxRotate);
bool findLineRight(S32 maxRotate);

TASK(TaskMotorControl) {
    Lcd lcd;
    
    S16 sensorValue;
    bool onLine;

    WaitEvent(EventTaskMotorControl);
    ClearEvent(EventTaskMotorControl);
    WaitEvent(EventTaskMotorControl);
    ClearEvent(EventTaskMotorControl);
    WaitEvent(EventTaskMotorControl);
    ClearEvent(EventTaskMotorControl);
    WaitEvent(EventTaskMotorControl);
    ClearEvent(EventTaskMotorControl);
    WaitEvent(EventTaskMotorControl);
    ClearEvent(EventTaskMotorControl);
    WaitEvent(EventTaskMotorControl);
    ClearEvent(EventTaskMotorControl);
    WaitEvent(EventTaskMotorControl);
    ClearEvent(EventTaskMotorControl);
    WaitEvent(EventTaskMotorControl);
    ClearEvent(EventTaskMotorControl);
    WaitEvent(EventTaskMotorControl);
    ClearEvent(EventTaskMotorControl);
    WaitEvent(EventTaskMotorControl);
    ClearEvent(EventTaskMotorControl);

    S8 blackValue,whiteValue;
    lcd.cursor(0,1);
    lcd.putf("s","Calibrate black");
    lcd.disp();

    // initialize
    bool pressed = false;
   /* while(!pressed)
    {
        WaitEvent(EventTaskMotorControl);
        ClearEvent(EventTaskMotorControl);

    	sensorOkLow = sensor.get() - sensorDelta;
    	sensorOkHigh = sensor.get() + sensorDelta;
    	pressed = touchSensor.isPressed();
    }*/
    while(!pressed)
    {
    	WaitEvent(EventTaskMotorControl);
	    ClearEvent(EventTaskMotorControl);

	    blackValue = sensor.get();
	    pressed = touchSensor.isPressed();
    }
    pressed = false;
    lcd.cursor(0,1);
    lcd.putf("s","Calibrate white");
    lcd.disp();
    while(!pressed)
    {
    	WaitEvent(EventTaskMotorControl);
	    ClearEvent(EventTaskMotorControl);

	    whiteValue = sensor.get();
	    pressed = touchSensor.isPressed();
    }
    S8 middle = (whiteValue + blackValue) / 2;
    sensorOkLow = middle - sensorDelta;
    sensorOkHigh = middle + sensorDelta;

    lcd.cursor(0, 1);
	lcd.putf("sd", "Calibrate: ", sensorOkHigh - sensorDelta);
	lcd.disp();

    while(1) {
        WaitEvent(EventTaskMotorControl);
        ClearEvent(EventTaskMotorControl);
        
        sensorValue = sensor.get();
        
        onLine = true;
        if (sensorValue < sensorOkLow) {
            // presumably too far right
            onLine = findLineLeft(100);
            lcd.cursor(2, 6);
            lcd.putf("s", "Too far right");
            lcd.disp();
        } else if (sensorValue > sensorOkHigh) {
            // presumably too far left
            onLine = findLineRight(100);
            lcd.cursor(2, 6);
            lcd.putf("s", "Too far left");
            lcd.disp();
        }
        
        if (onLine) {
//            lcd.cursor(2, 6);
//            lcd.putf("s", "   On Line   ");
//            lcd.disp();
            motorLeft.setPWM(motorForward + 5);
            motorRight.setPWM(motorForward);
        } else {
            // We lost the line
            motorLeft.setPWM(0);
            motorRight.setPWM(0);
            motorLeft.setBrake(true);
            motorRight.setBrake(true);
            lcd.cursor(0, 4);
            lcd.putf("s", "HELP!!! I'm lost!");
            lcd.disp();
            CancelAlarm(AlarmTaskMotorControl);
            TerminateTask();
        }
    }
}

TASK(TaskReadSensor)
{
    Lcd lcd;
    // Trigger TaskMotorControl every 100ms
    SetRelAlarm(AlarmTaskMotorControl, 1, 100);
    while(1) {
        // read the sensor data if nothing else is running
        sensor.processBackground();
        lcd.cursor(0, 0);
        lcd.putf("sd", "Sensor: ", sensor.get(), 5);
        lcd.disp();
    }
}



bool findLine(Motor &motor1, Motor &motor2, S32 maxRotate) {
	S16 sensorValue;
	S32 threshold = motor1.getCount() + maxRotate;
	motor2.reset();
	motor1.setPWM(motorBackward);
	motor2.setPWM(motorForward);
	while (motor2.getCount() < maxRotate) {
		WaitEvent(EventTaskMotorControl);
		ClearEvent(EventTaskMotorControl);
		sensorValue = sensor.get();
		if (sensorValue >= sensorOkLow &&
			sensorValue <= sensorOkHigh) {
			// found the line again
			return true;
		}
	}
	if (threshold < 10000) {
		// look in the other direction
		return findLine(motor2, motor1, 3*threshold);
	}
	return false;
}

bool findLineLeft(S32 maxRotate) {
	return findLine(motorLeft, motorRight, maxRotate);
}

bool findLineRight(S32 maxRotate) {
	return findLine(motorRight, motorLeft, maxRotate);
}

}
