////////////////////////////////////////////////////////////////
// ELEX 4699 Electrical System Design
// QR Shooting Robot
// Project4699.cpp
// Created June 04, 2023 by Yui Nguyen
// This contains the main function that pratically controls the car, including the manual mode based on commands.
// Due to some issues, I was unable to open the Pi Cam and run the server for manual mode in their own classes. Hence, 
// I moved those functions into main to solve the problem.
////////////////////////////////////////////////////////////////
#pragma once
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <iostream>
#include <sstream>
#include <string>
#include "Server.h"

#include "CCar.h"
#include "CGuidance.h"

#define RESWIDTH 480
#define RESHEIGHT 469
#define FPS 60

#define IMPORT 4618
#define CMDPORT 4619

#define SERV_PATHA 600

#define SERVO_MAX 2500
#define SERVO_MIN 500

#define SERV_INCREMENT 5
#define SERV_DELAY 3000
#define SERV_LOOP_THRES 50
Server cmdserv;
Server imserv;
using namespace cv;

CGuidance _send_im;
cv::VideoCapture _cap;
cv::Mat _display_im;
bool camthreadexit = false;

//Code from https://cboard.cprogramming.com/c-programming/63166-kbhit-linux.html
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}
//threads for server images
void imthread()
{
	imserv.start(4618);
}

void imserver()
{
	std::thread t3(&imthread);
	t3.detach();
}

//threads for server commands
void cmdthread()
{
	cmdserv.start(4619);
}

void server()
{
	std::thread t2(&cmdthread);
	t2.detach();
}



void intro()
{
	std::cout << "\n\nOptions:";
	std::cout << "\n(w) forward";
	std::cout << "\n(s) backward";
	std::cout << "\n(a) left";
	std::cout << "\n(d) right";
	std::cout << "\n(x) stop";
	std::cout << "\n(c) close";
	std::cout << "\n(o) open";
	std::cout << "\nCMD> ";
}


void cam_setup()
{
	_cap.open(0, cv::CAP_V4L);
	_cap.set(CAP_PROP_FRAME_WIDTH, (unsigned int)RESWIDTH);
	_cap.set(CAP_PROP_FRAME_HEIGHT, (unsigned int)RESHEIGHT);
	_cap.set(CAP_PROP_FPS, FPS);
}


cv::Mat Camera()
{
	do
	{
		if (_cap.isOpened() == true)
		{
			_cap >> _display_im;
			if (_display_im.empty() == false)
			{
				_send_im.detectMarker(_display_im);
				//turned off imshow for faster processing
				//imshow("detectMarker", _send_im.detectMarker(_display_im));
				cv::waitKey(50);
				imserv.set_txim(Camera());
				return _display_im;
			}
		}
		else _cap.release();
		if (camthreadexit == true)
		{
			break;
		}
	} while (camthreadexit == false);
}

void camthread()
{
	std::thread t3(&Camera);
	t3.detach();
}


int main(int argc, char* argv[])
{

	CCar car;
	server();
	imserver();
	cam_setup();
	camthread();

	char cmd = -1;
	int count_manual_track = 800;
	std::vector<std::string> cmds;

	std::cout << "\n Default is Autonomous. Switch to Manual? y/n" << std::endl;
	cmd = getchar();
	if (cmd == 'y') //if yes then go to manual mode
	{
		intro();
		do
		{
			cmdserv.get_cmd(cmds); //get command
			_send_im.find_target();

			if (cmds.size() > 0)
			{
				std::cout << "\nCMD rec";
				for (int i = 0; i < cmds.size(); i++)
				{
					if (cmds.at(i) == "s")
					{
						std::cout << "\nReceived 's' command";
						cmd = 's';
						std::string reply = "Server ack: BACK";
						cmdserv.send_string(reply);
						car.backward();
					}
					else if (cmds.at(i) == "w")
					{
						std::cout << "\nReceived 'w' command";
						cmd = 'w';
						std::string reply = "Server ack: FWD";
						cmdserv.send_string(reply);
						car.forward();
					}
					else if (cmds.at(i) == "a")
					{
						std::cout << "\nReceived 'a' command";
						cmd = 'a';
						std::string reply = "Server ack: LEFT";
						cmdserv.send_string(reply);
						car.left();
					}
					else if (cmds.at(i) == "d")
					{
						std::cout << "\nReceived 'd' command";
						cmd = 'd';
						std::string reply = "Server ack: RIGHT";
						cmdserv.send_string(reply);
						car.right();
					}
					else if (cmds.at(i) == "x")
					{
						std::cout << "\nReceived 'x' command";
						cmd = 'x';
						std::string reply = "Server ack: STOP";
						cmdserv.send_string(reply);
						car.stopcar();
					}
					else if (cmds.at(i) == "f")
					{
						std::cout << "\nReceived 'f command";
						cmd = 'f';
						std::string reply = "Server ack: FIRE";
						cmdserv.send_string(reply);
						car.fire();
					}
					else if (cmds.at(i) == "j")
					{
						std::cout << "\nReceived 'j' command";
						cmd = 'j';
						std::string reply = "Server ack: servo left";
						cmdserv.send_string(reply);
						for (int j = 0; j <= SERV_LOOP_THRES; j++)
						{

							if (count_manual_track > SERVO_MAX) {
								count_manual_track = SERVO_MAX;
							}
							else if (count_manual_track < SERVO_MIN) {
								count_manual_track = SERVO_MIN;
							}
							car.trackServ(count_manual_track);
							count_manual_track += SERV_INCREMENT;
							delayMicroseconds(SERV_DELAY);
						}
					}
					else if (cmds.at(i) == "l")
					{
						std::cout << "\nReceived 'l' command";
						cmd = 'j';
						std::string reply = "Server ack: servo right";
						cmdserv.send_string(reply);
						for (int j = 0; j <= SERV_LOOP_THRES; j++)
						{

							if (count_manual_track > SERVO_MAX) {
								count_manual_track = SERVO_MAX;
							}
							else if (count_manual_track < SERVO_MIN) {
								count_manual_track = SERVO_MIN;
							}

							car.trackServ(count_manual_track);
							count_manual_track -= SERV_INCREMENT;
							delayMicroseconds(SERV_DELAY);
						}
					}
				}
			}

			if (kbhit())
			{
				cmd = getchar();
				switch (cmd)
				{
				case 's': car.backward(); break;
				case 'w': car.forward(); break;
				case 'a': car.left(); break;
				case 'd': car.right(); break;
				case 'x': car.stopcar(); break;
				case 'f': car.fire(); break;
					//turning servo manually
				case 'j':
				{
					for (int j = 0; j <= SERV_LOOP_THRES; j++)
					{

						if (count_manual_track > SERVO_MAX) 
						{
							count_manual_track = SERVO_MAX;
						}
						else if (count_manual_track < SERVO_MIN) 
						{
							count_manual_track = SERVO_MIN;
						}
						car.trackServ(count_manual_track);
						count_manual_track += SERV_INCREMENT;
						delayMicroseconds(SERV_DELAY);
					}
					break; }
				case 'l':
				{
					for (int j = 0; j <= SERV_LOOP_THRES; j++)
					{

						if (count_manual_track > SERVO_MAX) 
						{
							count_manual_track = SERVO_MAX;
						}
						else if (count_manual_track < SERVO_MIN) 
						{
							count_manual_track = SERVO_MIN;
						}

						car.trackServ(count_manual_track);
						count_manual_track -= SERV_INCREMENT;
						delayMicroseconds(SERV_DELAY);
					}
					break; }
				}
			}
		} while (cmd != '0');
	}
	//SWITCH TO AUTONOMOUS
	//STATE MACHINES
	else if (cmd == 'n')
	{
		car.trackServ(SERV_PATHA); //init servo
		do
		{

			if (kbhit())
			{
				cmd = getchar();
				if (cmd == '0') break;
				else if (cmd == 'r') _send_im.cnt = 0;
			}
			//check for whether target is shot
/*
			cmdserv.get_cmd(cmds);
			 if (cmds.size() > 0)
				{
				  std::cout << "\nCMD rec";
				  for (int i = 0; i < cmds.size(); i++)
				  {
					if (cmds.at(0) == "1")
					{
					_send_im.targ1shot = true;
					std::cout << "T1 shot";
					}
					else
					{
					_send_im.targ1shot = false;
					}

					if (cmds.at(1) == "1")
					{
					_send_im.targ2shot = true;
					}
					else
					{
					_send_im.targ2shot = false;
					}

					if (cmds.at(2) == "1")
					{
					_send_im.targ3shot = true;
					}
					else
					{
					_send_im.targ3shot = false;
					}

					if (cmds.at(3) == "1")
					{
					_send_im.targ4shot = true;
					}
					else
					{
					_send_im.targ4shot = false;
					}
				}
			}
*/

			_send_im.pathA(SERV_PATHA);


			if (_send_im.pathAdone == true)
			{
				std::cout << "path A is done!" << std::endl;
				break;
			}

		} while (_send_im.pathAdone == false);
		car.trackServ(SERV_PATHA);
		int cnt = 0;
		//switch to path B
		do
		{

			if (cnt <= 1)
			{
				std::cout << "onto part B!" << std::endl;
				cnt++;
			}
			if (kbhit())
			{
				cmd = getchar();
				if (cmd == '0') break;
				else if (cmd == 'r') _send_im.cnt = 0;
			}

			_send_im.pathB(SERV_PATHA);
			//_send_im.find_target_auto();
			//_send_im.is_shot();

			if (_send_im.pathBdone == true)
			{
				std::cout << "path B is done!" << std::endl;
			}

		} while (_send_im.pathBdone == false && _send_im.pathAdone == true);
		cnt = 0;
		//switch to path C
		do
		{

			if (cnt <= 1)
			{
				std::cout << "onto part C!" << std::endl;
				cnt++;
			}
			if (kbhit())
			{
				cmd = getchar();
				if (cmd == '0') break;
				else if (cmd == 'r') _send_im.cnt = 0;
			}

			_send_im.pathC(SERV_PATHA);
			//_send_im.find_target_auto();
			//_send_im.is_shot();

			if (_send_im.pathCdone == true)
			{
				std::cout << "path C is done!" << std::endl;
			}

		} while (_send_im.pathCdone == false && _send_im.pathBdone == true);

		cnt = 0;
		//switch to path C
		do
		{

			if (cnt <= 1)
			{
				std::cout << "onto part D!" << std::endl;
				cnt++;
			}
			if (kbhit())
			{
				cmd = getchar();
				if (cmd == '0') break;
				else if (cmd == 'r') _send_im.cnt = 0;
			}

			_send_im.pathD(SERV_PATHA);
			//_send_im.find_target_auto();
			//_send_im.is_shot();

			if (_send_im.pathDdone == true)
			{
				std::cout << "path D is done!" << std::endl;
			}

		} while (_send_im.pathDdone == false && _send_im.pathCdone == true);
		cnt = 0;
		//switch to path C
		do
		{

			if (cnt <= 1)
			{
				std::cout << "onto part E!" << std::endl;
				cnt++;
			}
			if (kbhit())
			{
				cmd = getchar();
				if (cmd == '0') break;
				else if (cmd == 'r') _send_im.cnt = 0; //RESET
			}
			_send_im.pathE(SERV_PATHA);
			//check if state is done
			if (_send_im.pathEdone == true)
			{
				std::cout << "path E is done!" << std::endl;
			}

		} while (_send_im.pathEdone == false && _send_im.pathDdone == true);
	}

	camthreadexit = true;
	car.stopcar();
	cmdserv.stop();
	imserv.stop();
	cv::destroyAllWindows();
	std::cout << "Program closed successfully" << std::endl;
	return 0;
}
