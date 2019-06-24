#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/Float32.h"
#include <ncurses.h>

#define KEYCODE_R 0x43 	//right arrow
#define KEYCODE_L 0x44 	//left arrow
#define KEYCODE_U 0x41	//up arrow
#define KEYCODE_D 0x42	//down arrow
#define KEYCODE_Q 0x51	//key "q"

#define startVel_ 15
#define startSte_ 0

class TeleopTurtle
{
public:
	TeleopTurtle();
	void keyLoop();

private:
	ros::NodeHandle nh_;
	ros::Publisher speedPub_, steerPub_;
};

TeleopTurtle::TeleopTurtle()
{
	speedPub_ = nh_.advertise<std_msgs::Float32>("/set_speed_car_api", 1);
	steerPub_ = nh_.advertise<std_msgs::Float32>("/set_steer_car_api", 1);
}

int kfd = 0;
struct termios cooked, tRaw;

//call on receive interupt (Ctrl+C)
void quit(int sig)
{
	(void)sig;
	//change the console to cooked mode
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Teleop_Publisher");
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
	char c;
	float 	speedMsg_ = startVel_,
			steerMsg_ = startSte_;
	bool dirty=false;
	std_msgs::Float32 msg;

	// get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&tRaw, &cooked, sizeof(struct termios));
	tRaw.c_lflag &=~ (ICANON);
	// Setting a new line, then end of file                         
	tRaw.c_cc[VEOL] = 1;
	tRaw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &tRaw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the turtle.");


	for(;;)
	{
		// get the next event from the keyboard  
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		ROS_DEBUG("value: 0x%02X\n", c);
  
		switch(c)
		{
			case KEYCODE_L:
				ROS_DEBUG("LEFT");
				msg.data = 60;
				steerPub_.publish(msg);
				break;
			case KEYCODE_R:
				ROS_DEBUG("RIGHT");
				msg.data = -60;
				steerPub_.publish(msg);
				break;
			case KEYCODE_U:
				ROS_DEBUG("UP");
				msg.data = 20;
				speedPub_.publish(msg);
				break;
			case KEYCODE_D:
				ROS_DEBUG("DOWN");
				msg.data = -20;
				speedPub_.publish(msg);
				break;
			default:
				ROS_DEBUG("STOP");
				msg.data = 0;
				speedPub_.publish(msg);
				steerPub_.publish(msg);
				break;
		}
	}
	return;
}
