#include "iostream"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "DiRa_Traxxas_Controller.h"

PCA9685 *pca9685 = new PCA9685();

void set_speed_car(const std_msgs::Float32::ConstPtr& msg)
{
    double speed = msg->data;
    if (speed != 0)
    {
        api_set_FORWARD_control(pca9685, speed);
    }
    else
    {
        api_set_BRAKE_control(pca9685, speed);
    }
	ROS_INFO("new throttle: [%f]",speed);
}

void set_steer_car(const std_msgs::Float32::ConstPtr& msg)
{
    double steer_angle=msg->data;
    api_set_STEERING_control(pca9685, steer_angle);
    ROS_INFO("new steer: [%f]",steer_angle);
}

void set_pos_camera(const std_msgs::Float32::ConstPtr& msg)
{
    double camera_angle = msg->data;
    api_set_CAMERA_control(pca9685, camera_angle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DiRa_PCA9685_Controller");
    api_pwm_pca9685_init( pca9685 );
    double spd = 0;
    api_set_FORWARD_control(pca9685, spd);
    ros::NodeHandle n;
    ros::NodeHandle m;
    ros::NodeHandle l;
    ros::Subscriber sub1 = n.subscribe<std_msgs::Float32>("/set_speed_car_api", 10, set_speed_car);
    ros::Subscriber sub2 = m.subscribe<std_msgs::Float32>("/set_steer_car_api", 10, set_steer_car);
    ros::Subscriber sub3 = l.subscribe<std_msgs::Float32>("/set_pos_camera_api", 10, set_pos_camera);			
    ros::spin();
    return 0;
}

