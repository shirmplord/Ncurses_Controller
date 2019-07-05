#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <ncurses.h>
#include <cmath>

constexpr auto startVel_ = 0.0;
constexpr auto speedLimit_ = 20.0;
constexpr auto startSte_ = 0.0;
constexpr auto steerLimit_ = 60.0;
constexpr auto delay_ = 2;


class Callback
{
        public:
                Callback();
                ~Callback();
                void SensorCallback(const std_msgs::Bool::ConstPtr& msg);
                bool GetStatus();
        private
                bool safe;
};

int main(int argc, char** argv)
{
        ros::init(argc,argv,"Ncurses_Publisher");
    ros::NodeHandle nh;
    ros::Publisher speedPub_ = nh.advertise<std_msgs::Float32>("/set_speed_car_api", 10);
    ros::Publisher steerPub_ = nh.advertise<std_msgs::Float32>("/set_steer_car_api", 10);
    ros::NodeHandle nh_sub;
    Callback callback;
    ros::Subscriber sensorSub = nh_sub.subscribe<std_msgs::Bool>
            ("ss_status", 10, &Callback::SensorCallback, &callback);
    ros::Rate loop_rate(50);
    float speedMsg_ = startVel_, steerMsg_ = startSte_;
    bool turning = false, sentVel= false, sentSte = false, doNotPub;
    bool flag = false;              //flag for reset
    int count = 0, moving = 0;
    float forwardLimit_;

    int c;
        //Initialize the Ncurse block
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);


    while (ros::ok())
    {
        std_msgs::Float32 msgVel_;
        std_msgs::Float32 msgSte_;
        //no object read on the sensor
        if (callback.GetStatus())
        {
                //no key pressed
                if ((c = getch()) == ERR)
                {
                //delay for turning
                        if (!moving)
                        {
                                //delay for decelarate process
                                if (count >= delay_)
                                {
                                        //slowing down towards zero
                                        if (speedMsg_ > startVel_) speedMsg_--;
                                        else if (speedMsg_ < -startVel_) speedMsg_++;
                                        msgVel_.data = speedMsg_;
                                        //stop sending after reaching 0
                                        if (speedMsg_ != startVel_ && speedMsg_ != -startVel_)
                                        {
                                        speedPub_.publish(msgVel_);
                                }
                                count = 0;
                        }
                        else count++;
                }
                else if (moving > 0) moving --;
                //adjust steer towards zero
                if (steerMsg_ > 0) steerMsg_-= 10;
                else if (steerMsg_ < 0) steerMsg_ += 10;
                msgSte_.data = steerMsg_;
                //stop sending after reaching zero
                if (steerMsg_ != 0)
                {
                    steerPub_.publish(msgSte_);
                }
            }
            //keypressed
            else
            {
                 forwardLimit_ = speedLimit_;
                 bool doNotPub = false;
                 //publish part
                 if (c != KEY_LEFT && c != KEY_RIGHT) turning = false;
                 if (c != KEY_UP && c != KEY_DOWN) moving--;
                 switch (c)
                 {
                        case KEY_UP:
                        moving = 100;
                        if (steerMsg_ > 0) steerMsg_ -= 10;
                        else if (steerMsg_ < 0) steerMsg_ += 10;

                        if (speedMsg_ < forwardLimit_) speedMsg_++;

                        msgSte_.data = steerMsg_;
                        msgVel_.data = speedMsg_;
                        break;
                        case KEY_DOWN:
                        moving = 100;
                        if (steerMsg_ > 0) steerMsg_ -= 10;
                        else if (steerMsg_ < 0) steerMsg_ += 10;

                        if (speedMsg_ > -speedLimit_) speedMsg_--;

                        msgSte_.data = steerMsg_;
                        msgVel_.data = speedMsg_;
                        break;
                        case KEY_LEFT:
                        if (steerMsg_ < steerLimit_) steerMsg_ += 10;
                        msgSte_.data = steerMsg_;
                        msgVel_.data = speedMsg_;
                        turning = true;
                        break;
                        case KEY_RIGHT:
                        if (steerMsg_ > -steerLimit_) steerMsg_ -= 10;
                        msgSte_.data = steerMsg_;
                        msgVel_.data = speedMsg_;
                        turning = true;
                        break;
                                        default:
                        doNotPub = true;
                        break;
                 }
                 if (!doNotPub)
                 {
                 	speedPub_.publish(msgVel_);
                        steerPub_.publish(msgSte_);
                 }
            }
        }
        //------object read on the sensor
        else if ((speedMsg_ != 0 || steerMsg_ != 0))
        {
        	speedMsg_ = 0;
                steerMsg_ = 0;
                msgVel_.data = 0;
                msgSte_.data = 0;
               	if (forwardLimit_ > 0)
		{
                	speedPub_.publish(msgVel_);
                        steerPub_.publish(msgSte_);
                }
                forwardLimit_ = 0;
        }
                ros::spinOnce();
                loop_rate.sleep();
        }
        endwin();

        return 0;
}

Callback::Callback(){}
Callback::~Callback(){}

void Callback::SensorCallback(const std_msgs::Bool::ConstPtr& msg)
{
        safe = msg->data;
        return;
}

bool Callback::GetStatus()
{
        return safe;
}
