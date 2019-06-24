#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <termios.h>

#define startVel_ 10.0

int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);          // disable buffering      
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
	
	int c = getchar();  // read character (non-blocking)
		
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return c;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"Dira_Traxxas_Publisher");
	ros::NodeHandle nh;
	ros::Publisher speedPub = nh.advertise<std_msgs::Float32>("/set_speed_car_api", 1);
	ros::Publisher steerPub = nh.advertise<std_msgs::Float32>("/set_steer_car_api", 1);
	ros::Rate loop_rate(10);
	int oldC = 0;
	float speedMsg = startVel_, steerMsg = 60, oldMsg;
	int c;
	bool dirty = false;
	
	while (ros::ok())
	{
		std_msgs::Float32 msg;
		if (!dirty) c = getch();
		ROS_INFO("key %c was pressed\n", c);
		if (oldC != c) 
		{
			ROS_INFO("oldC: %d c: %d",oldC, c);
			speedMsg = startVel_;
			switch (c) 
			{
				case(119):	//case for "w" key - forward
					msg.data = speedMsg;
					ROS_INFO("message %lf was sent\n", msg.data);
					speedPub.publish(msg);
					break;
				case(115):	//case for "s" key - backward
					msg.data = -speedMsg;
					ROS_INFO("message %lf was sent\n", msg.data);
					speedPub.publish(msg);
					break;
				case(97):	//case for "a" key - left turn
					msg.data = steerMsg;
					ROS_INFO("message %lf was sent\n", msg.data);
					steerPub.publish(msg);
					break;
				case(100): //case for "d" key - right turn
					msg.data = -steerMsg;
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
		else if (speedMsg < 100)
		{
			speedMsg++;
			if (c == 119) msg.data = speedMsg;
			else if (c == 115) msg.data = -speedMsg;
			if (oldMsg != msg.data) speedPub.publish(msg);
			oldMsg = msg.data;
		}
		else 
		{
			speedMsg--;
			
		}
		oldC = c;
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
