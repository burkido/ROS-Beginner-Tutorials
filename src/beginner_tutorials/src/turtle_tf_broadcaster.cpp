#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg) {

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

void move(ros::NodeHandle n) {
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>(turtle_name+"/cmd_vel", 1000);
    geometry_msgs::Twist msg;
    ros::Rate loop_rate(10);
    msg.linear.x = 0.5;
    msg.angular.z = 0.2;
    while (ros::ok()) {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_tf_broadcaster");
    if (argc != 2){
        ROS_ERROR("need turtle name as argument");
        return -1;
    }

    turtle_name = argv[1];
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
    if (turtle_name == "/turtle1")
        move(node);

    ros::spin();  

    return 0;
}