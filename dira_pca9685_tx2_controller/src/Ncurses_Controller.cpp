#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <ncurses.h>

#define startVel_ 10.0

int main(int argc, char** argv)
{
	ros::init(argc,argv,"Ncurses_Publisher");
	ros::NodeHandle nh;
	ros::Publisher speedPub = nh.advertise<std_msgs::Float32>("/set_speed_car_api", 1);
	ros::Publisher steerPub = nh.advertise<std_msgs::Float32>("/set_steer_car_api", 1);
	ros::Rate loop_rate(10);
	int oldC = 0;
	float speedMsg_ = startVel_, steerMsg_ = 60, oldMsg_;
	
	int c;
	initscr();
	cbreak();
	noecho();
	keypad(stdscr, TRUE);
	nodelay(stdscr, TRUE);
	
	
	while (ros::ok())
	{
		std_msgs::Float32 msg;
		//no key pressed
		if ((c = getch()) == ERR)
		{
			
		}
		//keypressed
		else
		{
			if (oldC != c) 
			{
				ROS_INFO("oldC: %d c: %d",oldC, c);
				speedMsg_ = startVel_;
				switch (c) 
				{
					case(119):	//case for "w" key - forward
						msg.data = speedMsg_;
						ROS_INFO("message %lf was sent\n", msg.data);
						speedPub.publish(msg);
						break;
					case(115):	//case for "s" key - backward
						msg.data = -speedMsg_;
						ROS_INFO("message %lf was sent\n", msg.data);
						speedPub.publish(msg);
						break;
					case(97):	//case for "a" key - left turn
						msg.data = steerMsg_;
						ROS_INFO("message %lf was sent\n", msg.data);
						steerPub.publish(msg);
						break;
					case(100): //case for "d" key - right turn
						msg.data = -steerMsg_;
						ROS_INFO("message %lf was sent\n", msg.data);
						steerPub.publish(msg);
						break;
					case(113): //case for "q" key - stop
						msg.data = 0;
						ROS_INFO("message %lf was sent\n", msg.data);
						speedPub.publish(msg);
						steerPub.publish(msg);
						break;
					default:
						break;
				}
			}
			else if (speedMsg_ < 100)
			{
				speedMsg_++;
				if (c == 119) msg.data = speedMsg_;
				else if (c == 115) msg.data = -speedMsg_;
				if (oldMsg_ != msg.data) speedPub.publish(msg);
				oldMsg_ = msg.data;
			}
			else 
			{
				speedMsg_--;
			}
		}		
		oldC = c;
		ros::spinOnce();
		loop_rate.sleep();
	}
	endwin();
	
	return 0;
}
