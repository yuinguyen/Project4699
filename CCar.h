#ifndef CCAR_H
#define CCAR_H
//#include "CGuidance.h"
#include "pigpio.h"
#include <wiringPi.h>
#include <iostream>


class CCar
{
    private:
        //Server _server;
        //CMotor _motors;
        //CGuidance _guidance;
        static void serverthrd(CCar *ptr);
        static void imagethrd(CCar *ptr);
        int target_cnt;
    public:
        CCar();
        ~CCar();
        void drive();
        void forward();
        void forward_auto(int LPWMPERIOD, int RPWMPERIOD);
        void backward();
        void backward_auto(int left, int right);

        void left();
        void right();
        void stopcar();
        void fire();
        void trackServ(int servPos);
        //typedef enum MAIN_STATE {4_TARGETS, FINISH};
        //typedef enum SUB_4_TARGETS {FIND_TARGET, SHOOT};
        //typedef enum SUB_FINISH {MOVE_BACK, FIND_END, MOVE_END};
        void state_machine();

        bool find_target();
        bool is_shot();
        bool all_targets();
        bool is_end_target_seen(); //move backward to check for end target
        void move_end();

        //bool drive();

};


#endif // CCAR_H
