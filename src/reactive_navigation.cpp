#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na



class ReactiveController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;

    double front_obstacle_distance;
    double right_obstacle_distance;
    double left_obstacle_distance;
    bool robot_stopped;

    int min1 = 15;
    int max1 = 50;
    int random_dir;
    int min2 = 90;
    int max2 = 180;
    int random_stuck;


    geometry_msgs::Twist calculateCommand()
    {
    auto msg = geometry_msgs::Twist();


    random_dir = min1 + (rand() % static_cast<int>(max1 - min1 + 1)); //random number for direction angle
    random_stuck = min2 + (rand() % static_cast<int>(max2 - min2 + 1)); //random number for angle when stuck

    if(front_obstacle_distance > 0.5){
      if (left_obstacle_distance < 0.3 && right_obstacle_distance >= 0.4){
        msg.angular.z = ((5*M_PI)/36); //25 degree
        msg.linear.x = 0.2;
      }

      else if (right_obstacle_distance < 0.3 && left_obstacle_distance >= 0.4){
        msg.angular.z = -(5*M_PI)/36;
        msg.linear.x = 0.2;
      }

      else {
        msg.linear.x = 0.1;
      }

    }


    else{
        if (left_obstacle_distance > right_obstacle_distance){
          msg.linear.x = -0.1;
          msg.angular.z = -M_PI/4;
          std::cout << "NO left\n";
        }

        else if (left_obstacle_distance < right_obstacle_distance){
          msg.linear.x = -0.1;
          msg.angular.z = M_PI/4;
          std::cout << "NO right\n";
        }

        else if (fabs(left_obstacle_distance - right_obstacle_distance) < 0.4 || fabs(right_obstacle_distance - left_obstacle_distance) < 0.4 ){
          msg.angular.z = M_PI;
        }

        else //if (front_obstacle_distance < 0.2){
          {
          msg.linear.x = -0.3;
        }

    }

    return msg;
}


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        left_obstacle_distance = *std::min_element(&msg->ranges [0],&msg->ranges [80]);
        front_obstacle_distance = *std::min_element(&msg->ranges [81], &msg->ranges [160]);
	right_obstacle_distance = *std::min_element(&msg->ranges [161],&msg->ranges [239]);
        //ROS_INFO("Min distance to obstacle: %f", obstacle_distance);
    }


public:
    ReactiveController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Create a subscriber for laser scans
        this->laser_sub = n.subscribe("base_scan", 10, &ReactiveController::laserCallback, this);

    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);

            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "reactive_controller");


    // Create our controller object and run it
    auto controller = ReactiveController();
    controller.run();
}
