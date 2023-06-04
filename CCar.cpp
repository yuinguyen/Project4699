




//#include <pigpio.h>
//#include "car.h"
#include "CCar.h"

#define ENL 12// 21
#define ENR 16
#define in1L 18//26
#define in2L 23//19
#define in1R 1//6
#define in2R 24//13
#define STB 25
#define pulse 2
#define fire_servo 2//3
#define track_servo 15//4
#define TARGET1 21
#define TARGET2 22
#define TARGET3 27
#define TARGET4 23


#define SHOOT_DIST 11000

int PWMPERIOD = 90;
#define PWMPERIODTURN 65

//#define servo 3

using namespace std;
//CGuidance guide;

CCar::CCar()
{
    gpioInitialise();//ctor
    //wiringPiSetup();

    gpioSetMode(ENL, PI_OUTPUT);
    gpioSetMode(ENR, PI_OUTPUT);
    gpioSetMode(in1L, PI_OUTPUT);
    gpioSetMode(in2L, PI_OUTPUT);
    gpioSetMode(in1R, PI_OUTPUT);
    gpioSetMode(in2R, PI_OUTPUT);
    gpioSetMode(STB, PI_OUTPUT);
    gpioSetMode(fire_servo, PI_OUTPUT);
    gpioSetMode(track_servo, PI_OUTPUT);
    //gpioWrite(STB, HIGH);

    //gpioServo(track_servo, 1000);
    target_cnt = 0;

    //initialize state machine
    //MAIN_STATE main_state = 4_TARGETS;


}

CCar::~CCar()
{
//gpioWrite(STB, LOW);
    gpioTerminate();//dtor
}

void CCar::forward()
{
	cout << "FORWARD\n";
	//adding acceleration
	gpioPWM(ENR,PWMPERIOD/4);
	delay(10);
	gpioPWM(ENR,PWMPERIOD/3);
	delay(10);
	gpioPWM(ENR,PWMPERIOD/2);
	delay(10);
	gpioPWM(ENR,PWMPERIOD);
	//gpioWrite(ENR, HIGH);
	gpioWrite(in2R, LOW);
	gpioWrite(in1R, HIGH);

    gpioPWM(ENL,PWMPERIOD/4);
    delay(10);
	gpioPWM(ENL,PWMPERIOD/3);
	delay(10);
	gpioPWM(ENL,PWMPERIOD/2);
	delay(10);
	gpioPWM(ENL,PWMPERIOD);
	//gpioWrite(ENL, HIGH);
	gpioWrite(in2L, LOW);
	gpioWrite(in1L, HIGH);


}
void CCar::forward_auto(int LPWMPERIOD, int RPWMPERIOD)
{
	cout << "FORWARD\n";
	gpioPWM(ENR,RPWMPERIOD/4);
	delay(10);
	gpioPWM(ENR,RPWMPERIOD/3);
	delay(10);
	gpioPWM(ENR,RPWMPERIOD/2);
	delay(10);
	gpioPWM(ENR,RPWMPERIOD);
	gpioWrite(in2R, LOW);
	gpioWrite(in1R, HIGH);

    gpioPWM(ENL,LPWMPERIOD/4);
    delay(10);
    gpioPWM(ENL,LPWMPERIOD/3);
    delay(10);
    gpioPWM(ENL,LPWMPERIOD/2);
    delay(10);
	gpioPWM(ENL,LPWMPERIOD);
	gpioWrite(in2L, LOW);
	gpioWrite(in1L, HIGH);


}

void CCar::backward()
{
	cout << "BACKWARD\n";
//    gpioPWM(ENR,PWMPERIOD);
//	gpioWrite(in1R, LOW);
//	gpioWrite(in2R, HIGH);
//
//	gpioPWM(ENL,PWMPERIOD);
//	gpioWrite(in1L, LOW);
//	gpioWrite(in2L, HIGH);
//
//	delay(500); //go fast for half a sec
	gpioPWM(ENR,PWMPERIOD - 20);
	gpioWrite(in1R, LOW);
	gpioWrite(in2R, HIGH);

	gpioPWM(ENL,PWMPERIOD - 20);
	gpioWrite(in1L, LOW);
	gpioWrite(in2L, HIGH);
}

void CCar::backward_auto(int left, int right)
{
	cout << "BACKWARD\n";
//    gpioPWM(ENR,PWMPERIOD);
//	gpioWrite(in1R, LOW);
//	gpioWrite(in2R, HIGH);
//
//	gpioPWM(ENL,PWMPERIOD);
//	gpioWrite(in1L, LOW);
//	gpioWrite(in2L, HIGH);
//
//	delay(500); //go fast for half a sec
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
	int fire = 2000;
    for (int j = 0; j <= 70; j++)
    {

    //gpioSetMode(fire_servo, PI_OUTPUT);
	gpioServo(fire_servo, fire);
	fire -= 20;
	delayMicroseconds(2000);
	}

	for (int i = 0; i<=70; i++)
	{
	    //gpioSetMode(fire_servo, PI_OUTPUT);
    //gpioSetMode(fire_servo, PI_OUTPUT);
	gpioServo(fire_servo, fire);
	fire += 20;
	delayMicroseconds(1000);
	}
	//delay(500);
	//gpioPWM(fire_servo, 125);
	//delay(1000);
	//gpioWrite(pulse,HIGH);
	//delay(18);
	//gpioWrite(pulse,LOW);
	//gpioPWM(fire_servo, 50);

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
