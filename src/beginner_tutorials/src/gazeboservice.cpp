#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "gazebo_msgs/SetModelState.h"

using namespace std;

ros::Publisher velocity_pub;
geometry_msgs::Twist msg;

void move_distance(ros::NodeHandle nh, double distance, double speed) {
    
    ros::Rate loop_rate(10);
    double t0 = ros::Time::now().toSec();
    double current_distance = 0;

    while(current_distance < distance) {
        msg.linear.x = speed;
        velocity_pub.publish(msg);
        double t1 = ros::Time::now().toSec();
        current_distance = msg.linear.x * (t1-t0);
        cout << "Moving forward " << t1 << " " << t0 << endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    msg.linear.x = 0;
    msg.angular.z = 0;
    velocity_pub.publish(msg);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "gazeboservice");
    ros::NodeHandle nh;
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    move_distance(nh, 1, 0.1);

    ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    std_srvs::Empty srv;
    clear_client.call(srv);

    return 0;
}