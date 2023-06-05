
////////////////////////////////////////////////////////////////
// ELEX 4699 Electrical System Design
// QR Shooting Robot
// CCar.cpp
// Created June 04, 2023 by Yui Nguyen
// This CCar class contains functions that directly control the car using pigpio.h
////////////////////////////////////////////////////////////////
#include "CCar.h"

#define ENL 12
#define ENR 16
#define in1L 18
#define in2L 23
#define in1R 1
#define in2R 24
#define STB 25
#define pulse 2
#define fire_servo 2
#define track_servo 15
#define TARGET1 21
#define TARGET2 22
#define TARGET3 27
#define TARGET4 23
#define ACC_DELAY 10
#define FIRE_INIT_POS 2000
#define FIRE_INCREMENT 20
#define FORLOOPEXIT 70
#define DELAY_SHOOT 2000
#define DELAY_RETREAT 1000

#define BIAS_SPEED 20

#define SHOOT_DIST 11000

int PWMPERIOD = 90;
#define PWMPERIODTURN 65

//#define servo 3

using namespace std;
//CGuidance guide;

CCar::CCar()
{
    gpioInitialise();//ctor

    gpioSetMode(ENL, PI_OUTPUT);
    gpioSetMode(ENR, PI_OUTPUT);
    gpioSetMode(in1L, PI_OUTPUT);
    gpioSetMode(in2L, PI_OUTPUT);
    gpioSetMode(in1R, PI_OUTPUT);
    gpioSetMode(in2R, PI_OUTPUT);
    gpioSetMode(STB, PI_OUTPUT);
    gpioSetMode(fire_servo, PI_OUTPUT);
    gpioSetMode(track_servo, PI_OUTPUT);
    target_cnt = 0;
    //initialize state machine
    //MAIN_STATE main_state = 4_TARGETS;
}

CCar::~CCar()
{
    gpioTerminate();//dtor
}

void CCar::forward()
{
	cout << "FORWARD\n";
	//implementing acceleration
	gpioPWM(ENR,PWMPERIOD/4);
	delay(ACC_DELAY);
	gpioPWM(ENR,PWMPERIOD/3);
	delay(ACC_DELAY);
	gpioPWM(ENR,PWMPERIOD/2);
	delay(ACC_DELAY);
	gpioPWM(ENR,PWMPERIOD);
	gpioWrite(in2R, LOW);
	gpioWrite(in1R, HIGH);

    gpioPWM(ENL,PWMPERIOD/4);
    delay(ACC_DELAY);
	gpioPWM(ENL,PWMPERIOD/3);
	delay(ACC_DELAY);
	gpioPWM(ENL,PWMPERIOD/2);
	delay(ACC_DELAY);
	gpioPWM(ENL,PWMPERIOD);
	gpioWrite(in2L, LOW);
	gpioWrite(in1L, HIGH);


}
void CCar::forward_auto(int LPWMPERIOD, int RPWMPERIOD)
{
	cout << "FORWARD\n";
	//implementing acceleration
	gpioPWM(ENR,RPWMPERIOD/4);
	delay(ACC_DELAY);
	gpioPWM(ENR,RPWMPERIOD/3);
	delay(ACC_DELAY);
	gpioPWM(ENR,RPWMPERIOD/2);
	delay(ACC_DELAY);
	gpioPWM(ENR,RPWMPERIOD);
	gpioWrite(in2R, LOW);
	gpioWrite(in1R, HIGH);

    gpioPWM(ENL,LPWMPERIOD/4);
    delay(ACC_DELAY);
    gpioPWM(ENL,LPWMPERIOD/3);
    delay(ACC_DELAY);
    gpioPWM(ENL,LPWMPERIOD/2);
    delay(ACC_DELAY);
	gpioPWM(ENL,LPWMPERIOD);
	gpioWrite(in2L, LOW);
	gpioWrite(in1L, HIGH);
}

void CCar::backward()
{
	cout << "BACKWARD\n";

	gpioPWM(ENR,PWMPERIOD - BIAS_SPEED);
	gpioWrite(in1R, LOW);
	gpioWrite(in2R, HIGH);

	gpioPWM(ENL,PWMPERIOD - BIAS_SPEED);
	gpioWrite(in1L, LOW);
	gpioWrite(in2L, HIGH);
}

void CCar::backward_auto(int left, int right)
{
	cout << "BACKWARD\n";

	gpioPWM(ENR,right);
	gpioWrite(in1R, LOW);
	gpioWrite(in2R, HIGH);

	gpioPWM(ENL,left);
	gpioWrite(in1L, LOW);
	gpioWrite(in2L, HIGH);
}

void CCar::left()
{
	cout << "LEFT\n";

	gpioPWM(ENR,PWMPERIODTURN);
    gpioWrite(in2R, HIGH);
	gpioWrite(in1R, LOW);

	gpioPWM(ENL,PWMPERIODTURN);
	gpioWrite(in2L, LOW);
	gpioWrite(in1L, HIGH);
}

void CCar::right()
{
	cout << "RIGHT\n";

	gpioPWM(ENR,PWMPERIODTURN);
	gpioWrite(in2R, LOW);
	gpioWrite(in1R, HIGH);

	gpioPWM(ENL,PWMPERIODTURN);
	gpioWrite(in2L, HIGH);
	gpioWrite(in1L, LOW);
}


void CCar::stopcar()
{
    cout << "STOP\n";
	gpioWrite(in1R, LOW);
	gpioWrite(in2R, LOW);
	//deccelerate
	gpioPWM(ENR,50);
	gpioPWM(ENR,0);

	gpioWrite(in1L, LOW);
	gpioWrite(in2L, LOW);
	gpioPWM(ENR,50);
	gpioPWM(ENL,0);
}

void CCar::fire()
{
	cout << "FIRING\n\n";
	int fire = FIRE_INIT_POS;
    for (int j = 0; j <= FORLOOPEXIT; j++)
    {

	gpioServo(fire_servo, fire);
	fire -= FIRE_INCREMENT;
	delayMicroseconds(DELAY_SHOOT);
	}

	for (int i = 0; i <= FORLOOPEXIT; i++)
	{
	gpioServo(fire_servo, fire);
	fire += FIRE_INCREMENT;
	delayMicroseconds(DELAY_RETREAT);
	}
}
 void CCar::trackServ(int servPos)
 {
 cout << "TURN SERVO\n";
 gpioServo(track_servo, servPos);
 }

bool CCar::all_targets()
{
}

bool CCar::is_end_target_seen()
{
}
