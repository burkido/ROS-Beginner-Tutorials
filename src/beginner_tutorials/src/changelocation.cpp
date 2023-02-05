#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_srvs/Empty.h>
#include "gazebo_msgs/SetModelState.h"

using namespace std;
ros::Publisher velocity_pub;
geometry_msgs::Twist msg;

void move_distance(ros::NodeHandle nh, double distance) {

    ros::Rate loop_rate(10);
    double t0 = ros::Time::now().toSec();
    double current_distance = 0;

    while(current_distance < distance) {
        msg.linear.x = 0.5;
        velocity_pub.publish(msg);
        double t1 = ros::Time::now().toSec();
        current_distance = 0.5 * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    msg.linear.x = 0;
    msg.angular.z = 0;
    velocity_pub.publish(msg);
}




int main (int argc, char **argv) {
   
   ros::init(argc, argv, "gazebocontrol");
   ros::NodeHandle nh;
   //velocity_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    ros::ServiceClient resetsim_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");

    std_srvs::Empty srv;
    resetsim_client.call(srv);

    move_distance(nh, 1.0);

    ros::ServiceClient resetworld_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    resetworld_client.call(srv);

    return 0;
}