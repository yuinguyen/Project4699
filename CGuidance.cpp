////////////////////////////////////////////////////////////////
// ELEX 4699 Electrical System Design
// QR Shooting Robot
// CGuidance.cpp
// Created June 04, 2023 by Yui Nguyen
// This CGuidance class takes care of image processing and state functions for the autonomous mode
////////////////////////////////////////////////////////////////
#include "CGuidance.h"
#include "CCar.h"
#define RESWIDTH 480
#define RESHEIGHT 469
#define SERVO_MAX 2500
#define SERVO_MIN 500
#define TARGET1 21
#define TARGET2 22
#define TARGET3 27
#define TARGET4 23
#define QREND 29
#define SHOOT_DIST 2000

#define SHOOTTWIST 80
#define TURNDELAY 90
#define STOPDELAY 100

#define SERVOTURNDELAY 60

#define MAXBOUND 290
#define MINBOUND 210

#define STEPMAXBOUND 180
#define STEPMIDBOUND 130
#define STEPMINBOUND 80

#define BIGSTEP 15
#define MIDSTEP 10
#define SMOLSTEP 5

#define PATHA_SERVO_INCREMENT 10

#define ADDSPEED 23
#define REDUCESPEED 45
#define BASESPEED 100
#define QRA 30
#define QRB 34

#define AREA_QRA_THRES 380
#define AREA_QRA_THRES_AFTER 1000
#define AREA_TARG2_THRES 8300
#define AREA_TARG3_THRES 6000
#define AREA_TARG4_THRES 5500
#define AREA_QREND_THRES 11000

#define REVERSE_LEFT_SPEED 40
#define REVERSE_RIGHT_SPEED 90

#define PATHA_FWDSPEED 90
#define BIAS_SPEED 5
#define FINISH_BIAS_SPEED 21

#define INITSERVO 600


using namespace cv;
using namespace std;

CCar car;

CGuidance::CGuidance()
{
    //initialize
    cnt = 0;
    shootcnt = 0;
    serverthreadexit = false;
    foundtarg1 = false;
    foundtarg2 = false;
    foundtarg3 = false;
    foundtarg4 = false;

    pathAdone = false;
    pathBdone = false;
    pathCdone = false;
    pathDdone = false;

    targ1shot = false;
    targ2shot = false;
    targ3shot = false;
    targ4shot = false;

    look_targ1 = false;
    look_targ2 = false;
    look_targ3 = false;
    look_targ4 = false;

    start_look_targ2 = false;
    start_look_targ3 = false;
    start_look_targ4 = false;
}
CGuidance::~CGuidance()
{
}

void CGuidance::update(cv::Mat imfromMain)
{
}

int CGuidance::get_area(cv::Point2f xy1, cv::Point2f xy2, cv::Point2f xy3, cv::Point2f xy4)
{
    area = abs((xy1.x * xy2.y - xy1.y * xy2.x) + (xy2.x * xy3.y - xy2.y * xy3.x) + (xy3.x * xy4.y - xy3.y * xy4.x) + (xy4.x * xy1.y - xy4.y * xy1.x)) / 2;
    return area;
}

cv::Mat CGuidance::detectMarker (cv::Mat inputImage)
{
    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);
    cv::Mat outputImage = inputImage.clone();
    if (markerIds.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    }
    std::vector<cv::Point2f> ptsOut(4);
    int cnt = 0;
    for (int i = 0; i<(int)markerIds.size(); i++)
    {
        //get marker's center coordinates
        int x = (markerCorners[i][0].x + markerCorners[i][2].x)/2;
        int y = (markerCorners[i][0].y + markerCorners[i][2].y)/2;
        ptsOut[0] = (cv::Point2f(x,y));
        int dist = get_area(markerCorners[i][0], markerCorners[i][1], markerCorners[i][2], markerCorners[i][3]);
        //displaying relative distance
        cv::putText (outputImage, "dist: "+std::to_string(dist), ptsOut[0], 1, 0.5, cv::Scalar(255,255,255), 1, LINE_8, false);
    }
    return outputImage;
}

bool CGuidance::find_target()
{
    if (markerIds.size() > 0)
    {
        for (int i = 0; i<(int)markerIds.size(); i++)
        {
            if (markerIds[i] == TARGET1 ||  markerIds[i] == TARGET2 || markerIds[i] == TARGET3 || markerIds[i] == TARGET4)
            {
                //setting flags of finding targets
                if (markerIds[i] == TARGET1 && foundtarg1 == false)
                {
                    foundtarg1 = true;
                    std::cout << "TARGET 1 FOUND: id " + std::to_string(TARGET1)  << std::endl;
                }
                else if (markerIds[i] == TARGET2 && foundtarg2 == false)
                {
                    foundtarg2 = true;
                    std::cout << "TARGET 2 FOUND: id " + std::to_string(TARGET2) << std::endl;
                }
                else if (markerIds[i] == TARGET3 && foundtarg3 == false)
                {
                    foundtarg3 = true;
                    std::cout << "TARGET 3 FOUND: id " + std::to_string(TARGET3) << std::endl;
                }
                else if (markerIds[i] == TARGET4 && foundtarg4 == false)
                {
                    foundtarg4 = true;
                    std::cout << "TARGET 4 FOUND: id " + std::to_string(TARGET4) << std::endl;
                }
            }
            else return false;
        }
    }
}

bool CGuidance::find_target_auto(std::string wheel_or_serv, int baseleftspeed, int baserightspeed)
{
    if (markerIds.size() > 0)
    {
        for (int i = 0; i<(int)markerIds.size(); i++)
        {
            if (markerIds[i] == TARGET1 ||  markerIds[i] == TARGET2 || markerIds[i] == TARGET3 || markerIds[i] == TARGET4 || markerIds[i] == QREND)
            {
                if (markerIds[i] == TARGET1 && foundtarg1 == false)
                {
                    foundtarg1 = true;
                    std::cout << "TARGET 1 FOUND: id " + std::to_string(TARGET1)  << std::endl;
                }
                else if (markerIds[i] == TARGET2 && foundtarg2 == false)
                {
                    foundtarg2 = true;
                    std::cout << "TARGET 2 FOUND: id " + std::to_string(TARGET2) << std::endl;
                }
                else if (markerIds[i] == TARGET3 && foundtarg3 == false)
                {
                    foundtarg3 = true;
                    std::cout << "TARGET 3 FOUND: id " + std::to_string(TARGET3) << std::endl;
                }
                else if (markerIds[i] == TARGET4 && foundtarg4 == false)
                {
                    foundtarg4 = true;
                    std::cout << "TARGET 4 FOUND: id " + std::to_string(TARGET4) << std::endl;
                }
                //choose to adjust servo or wheel to track target
                int x = (markerCorners[i][0].x + markerCorners[i][2].x)/2;
                if (wheel_or_serv == "SERVO")
                {
                    adjustServo(x);
                }
                else if (wheel_or_serv == "WHEEL")
                {
                    adjustWheels(x, baseleftspeed, baserightspeed);
                }
            }
            else
            {
                car.stopcar();
                return false;
            } 
        }


    }
}
//code influenced by Myles Parfeniuk
void CGuidance::adjustServo(int centerPointX)
{
    bool addPwm= false;
    bool noChange= false;
    int stepCheck= 0;
    int step=0;
    if(centerPointX < MINBOUND)
    {
        addPwm= true;
        stepCheck= MINBOUND - centerPointX;
    }
    else if(centerPointX > MAXBOUND)
    {
        addPwm=false;
        stepCheck= centerPointX -MAXBOUND;
    }
    else if((centerPointX < MAXBOUND) && (centerPointX > MINBOUND))
    {
        noChange= true;
        shoottarget1 = true;
        car.fire();
        car.trackServ(INITSERVO);
        targ1shot = true;
        std::cout <<"target 1 shot!" << std::endl;
        cnt = 0;

    }

    if(stepCheck > STEPMAXBOUND)
    {
        step= BIGSTEP;
    }
    else if(stepCheck > STEPMIDBOUND && stepCheck <=STEPMAXBOUND )
    {
        step= MIDSTEP;
    }
    else  if (stepCheck >STEPMINBOUND && stepCheck <= STEPMIDBOUND)
    {
        step= SMOLSTEP;
    }
    else
    {
        step = MIDSTEP;
    }

    //step = stepCheck*4;
    if(addPwm == true)
    {
        servPos+=step;
        delay(SERVOTURNDELAY);
    }
    else
    {
        servPos-=step;
        delay(SERVOTURNDELAY);
    }

    if(servPos > SERVO_MAX)
    {
        servPos= SERVO_MAX;
    }
    else if(servPos < SERVO_MIN)
    {
        servPos= SERVO_MIN;
    }
    if (noChange == false)
    {
        car.trackServ(servPos);

    }

}
//code influenced by Myles Parfeniuk, but applied to wheels
void CGuidance::adjustWheels(int centerPointX, int baseleftspeed, int baserightspeed)
{

    bool addPwm= false;
    bool noChange= false;
    int stepCheck= 0;
    int step=0;
    if(centerPointX < MINBOUND)
    {
        addPwm= true;
        car.forward_auto(baseleftspeed + ADDSPEED, baserightspeed - REDUCESPEED);
    }
    else if(centerPointX > MAXBOUND)
    {
        addPwm=false;
        car.forward_auto(baseleftspeed - REDUCESPEED, baserightspeed + ADDSPEED);
    }
    else if((centerPointX < MAXBOUND) && (centerPointX > MINBOUND))
    {
        noChange= true;
        car.forward_auto(baseleftspeed - ADDSPEED, baserightspeed - ADDSPEED);
    }

}

int CGuidance::get_pos(int targID)
{
    for (int i = 0; i<(int)markerIds.size(); i++)
    {
        if (markerIds[i] == targID)
        {
            int x = (markerCorners[i][0].x + markerCorners[i][2].x)/2;
            return x;
        }

        else return 0;
    }
}

bool CGuidance::at_center(int centerPointX)
{
    bool addPwm= false;
    bool noChange= false;
    int stepCheck= 0;
    int step=0;
    if((centerPointX < MAXBOUND) && (centerPointX > MINBOUND))
    {
        noChange= true;
        return true;
    }
    else return false;
}


void CGuidance::pathA(int init_pos)
{
    if (targ1shot == false)
    {
        for (int i = 0; i<(int)markerIds.size(); i++)
        {
            if (markerIds[i] == QRA && area < AREA_QRA_THRES) //look for the first qr code
            {
                car.forward_auto(PATHA_FWDSPEED + BIAS_SPEED, PATHA_FWDSPEED);
            }
            else if ((markerIds[i] == QRA && area >= AREA_QRA_THRES))
            {
                car.stopcar();
                look_targ1 = true;
            }
        }

        if (look_targ1 == true)
        {
            car.trackServ(init_pos + cnt);
            cnt += PATHA_SERVO_INCREMENT;
            if (cnt >= SERVO_MAX - init_pos)
            {
                cnt = SERVO_MAX - init_pos;
            }

            delay(20);
            for (int i = 0; i<(int)markerIds.size(); i++)
            {
                if (markerIds[i] == TARGET1 && targ1shot == false)
                {
                    find_target_auto("SERVO", BASESPEED, BASESPEED);
                }
            }


        }
    }



    else if (targ1shot == true && pathAdone == false)//if targ1 shot
    {
        if (start_look_targ2 == false)
        {
            for (int i = 0; i<(int)markerIds.size(); i++)
            {
                //calc area again bc thread confuses them all
                std::cout <<"preparing for detecting targ 2" << std::endl;
                std::vector<cv::Point2f> ptsOut(4);
                int x = (markerCorners[i][0].x + markerCorners[i][2].x)/2;
                int y = (markerCorners[i][0].y + markerCorners[i][2].y)/2;
                ptsOut[0] = (cv::Point2f(x,y));
                int dist = get_area(markerCorners[i][0], markerCorners[i][1], markerCorners[i][2], markerCorners[i][3]);
                if (markerIds[i] == QRA && dist < AREA_QRA_THRES_AFTER)
                {
                    car.forward_auto(PATHA_FWDSPEED + BIAS_SPEED*2, PATHA_FWDSPEED + BIAS_SPEED); //just a little faster
                    std::cout <<"move car forward to qra" << std::endl;
                }
                else if (markerIds[i] == QRA && dist >= AREA_QRA_THRES_AFTER)
                {
                    car.stopcar();
                    start_look_targ2 = true;
                }
            }
        }
        if (start_look_targ2 == true)
        {
            if ((int)markerIds.size() == 0 && pathAdone == false)
            {
                car.right();
                delay(TURNDELAY+3);
                car.stopcar();
                delay(STOPDELAY+20);
            }
            for (int i = 0; i<(int)markerIds.size(); i++)
            {
                if (markerIds[i] != TARGET2  && foundtarg2 == false)
                {
                    car.right();
                    delay(TURNDELAY);
                    car.stopcar();
                    delay(STOPDELAY);
                }
                else if (markerIds[i] == TARGET2 )
                {
                    foundtarg2 = true;
                    car.stopcar();
                    pathAdone = true;
                }
            }
        }
    }
}

void CGuidance::pathB(int init_pos)
{
    for (int i = 0; i<(int)markerIds.size(); i++)
    {
        //find area again
        std::vector<cv::Point2f> ptsOut(4);
        int x = (markerCorners[i][0].x + markerCorners[i][2].x)/2;
        int y = (markerCorners[i][0].y + markerCorners[i][2].y)/2;
        ptsOut[0] = (cv::Point2f(x,y));
        int dist = get_area(markerCorners[i][0], markerCorners[i][1], markerCorners[i][2], markerCorners[i][3]);
        if (markerIds[i] == TARGET2  && dist <= AREA_TARG2_THRES)
        {
            find_target_auto("WHEEL",BASESPEED, BASESPEED);
        }
        else if (markerIds[i] == TARGET2 && dist > AREA_TARG2_THRES)
        {
            car.stopcar();
            car.fire();
            shootcnt = 0;
            car.trackServ(init_pos);
            targ2shot = true;
            start_look_targ3 = true;
        }
    }
    if (start_look_targ3 == true)
    {
        if ((int)markerIds.size() == 0 && pathBdone == false)
        {
            //start turning to look for the next target
            car.right();
            delay(TURNDELAY);
            car.stopcar();
            delay(STOPDELAY);
        }
        for (int i = 0; i<(int)markerIds.size(); i++)
        {
            if (markerIds[i] != TARGET3  && foundtarg3 == false)
            {
                car.right();
                delay(TURNDELAY);
                car.stopcar();
                delay(STOPDELAY);
            }
            else if (markerIds[i] == TARGET3 )
            {
                foundtarg3 = true;
                car.stopcar();
                pathBdone = true;
            }
        }
    }
}

void CGuidance::pathC(int init_pos)
{
    for (int i = 0; i<(int)markerIds.size(); i++)
    {
        //find area again
        std::vector<cv::Point2f> ptsOut(4);
        int x = (markerCorners[i][0].x + markerCorners[i][2].x)/2;
        int y = (markerCorners[i][0].y + markerCorners[i][2].y)/2;
        ptsOut[0] = (cv::Point2f(x,y));
        int dist = get_area(markerCorners[i][0], markerCorners[i][1], markerCorners[i][2], markerCorners[i][3]);
        if (markerIds[i] == TARGET3  && dist <= AREA_TARG3_THRES)
        {
            find_target_auto("WHEEL",BASESPEED, BASESPEED);
        }
        else if (markerIds[i] == TARGET3 && dist > AREA_TARG3_THRES)
        {
            //SHOOT TARGET!!
            car.stopcar();
            car.fire();
            car.trackServ(init_pos);
            shootcnt = 0;
            targ3shot = true;
            start_look_targ4 = true;

        }
    }

    if (start_look_targ4 == true)
    {
        if ((int)markerIds.size() == 0 && pathCdone == false)
        {
            car.right();
            delay(TURNDELAY);
            car.stopcar();
            delay(STOPDELAY);
        }
        for (int i = 0; i<(int)markerIds.size(); i++)
        {
            if (markerIds[i] != TARGET4  && foundtarg4 == false)
            {
                car.right();
                delay(TURNDELAY);
                car.stopcar();
                delay(STOPDELAY);
            }
            else if (markerIds[i] == TARGET4 ) //&& at_center(get_pos(TARGET2)) == true)
            {
                foundtarg4 = true;
                car.stopcar();
                pathCdone = true;
            }
        }
    }
}

void CGuidance::pathD(int init_pos)
{
    for (int i = 0; i<(int)markerIds.size(); i++)
    {
        //find area again
        std::vector<cv::Point2f> ptsOut(4);
        int x = (markerCorners[i][0].x + markerCorners[i][2].x)/2;
        int y = (markerCorners[i][0].y + markerCorners[i][2].y)/2;
        ptsOut[0] = (cv::Point2f(x,y));
        int dist = get_area(markerCorners[i][0], markerCorners[i][1], markerCorners[i][2], markerCorners[i][3]);
        if (markerIds[i] == TARGET4  && dist <= AREA_TARG4_THRES)
        {
            find_target_auto("WHEEL",BASESPEED, BASESPEED);
        }
        else if (markerIds[i] == TARGET4 && dist > AREA_TARG4_THRES)
        {
            car.stopcar();
            car.fire();
            shootcnt = 0;
            car.trackServ(init_pos);
            targ4shot = true;
            turn_pathD = true;
        }
    }

    if (turn_pathD == true)
    {
            car.right();
            delay (250);
            car.stopcar();
            pathDdone = true;
    }


}


void CGuidance::pathE(int init_pos)
{
if (markerIds.size() == 0 && foundqrend == false)
        {
            car.backward_auto(REVERSE_LEFT_SPEED, REVERSE_RIGHT_SPEED);
        }
    for (int i = 0; i<(int)markerIds.size(); i++)
    {
        std::vector<cv::Point2f> ptsOut(4);
        int x = (markerCorners[i][0].x + markerCorners[i][2].x)/2;
        int y = (markerCorners[i][0].y + markerCorners[i][2].y)/2;
        ptsOut[0] = (cv::Point2f(x,y));
        int dist = get_area(markerCorners[i][0], markerCorners[i][1], markerCorners[i][2], markerCorners[i][3]);
        if (markerIds[i] != QREND && foundqrend == false)
        {

            car.backward_auto(REVERSE_LEFT_SPEED, REVERSE_RIGHT_SPEED);
        }
        else if (markerIds[i] == QREND && dist <= AREA_QREND_THRES)
        {
            foundqrend = true;
            std::cout<<"going towards end" << std::endl;
            find_target_auto("WHEEL",BASESPEED + FINISH_BIAS_SPEED, BASESPEED);
        }
        else if (markerIds[i] == QREND && dist > AREA_QREND_THRES)
        {
            std::cout << "close enough!"<< std::endl;
            car.stopcar();
            pathEdone = true;
        }
    }
}