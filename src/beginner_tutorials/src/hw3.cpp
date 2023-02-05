#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

ros::Publisher pub;
geometry_msgs::Twist msg;
geometry_msgs::Pose2D current_pose;

//For stage -> use robot namespace robot_0 or robot_1 before cmd_vel
//For gazebo -> use cmd_vel if 1 robot
//For gazebo -> use tb3_0 tb3_1 if more than 1 robots

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    //current_pose.theta = tf::getYaw(msg->pose.pose.orientation);

    //ROS_INFO("%s", msg->header.frame_id.to_str());
    //ROS_INFO("%s", msg->twist.twist.linear.x);
    //ROS_INFO("%s", msg->pose.pose.position.x);

    tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pose.theta = yaw;

    //ROS_INFO("Current Pose: x: %f, y: %f, theta: %f", current_pose.x, current_pose.y, current_pose.theta);
}

void move(ros::NodeHandle nh) {

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok() && current_pose.x < 2.0) {
        msg.linear.x = 0.2;
        //msg.angular.z = 0.0;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    //stop the robot
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    pub.publish(msg);
}


void rotate_2(ros::NodeHandle n, double degree) {
    
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(10);
    
    msg.linear.x = 0.0;
    msg.angular.z = 0.2;
    
    double target_rad = degree * M_PI / 180.0;
    double c = 0.5; //constant from angılatrıcal velocity function
    double inittheta = current_pose.theta;

    while (ros::ok() /*&& current_pose.theta < inittheta + target_rad*/) {
        std::cout << "targetrad: " << msg.angular.z << std::endl;
        msg.linear.x = 0.0;
        msg.angular.z = c * sqrt(target_rad - current_pose.theta);
        if((target_rad - inittheta) > 0 && msg.angular.z < 0.008) break;
        if((target_rad - inittheta) < 0 && msg.angular.z > -0.008) break;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebocontrol");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/odom", 1000, odomCallback);

    //move(nh);
    rotate_2(nh, 90);

    ros::spin();

    return 0;
}