#include <ros/ros.h>
//#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "setcolornode_cpp");
    ros::NodeHandle nh;

    int color_r, color_g, color_b;

    nh.getParam("r", color_r);
    nh.getParam("g", color_g);
    nh.getParam("b", color_b);
    ros::service::waitForService("clear");

    ros::param::set("background_r", color_r);
    ros::param::set("background_g", color_g);
    ros::param::set("background_b", color_b);

    // change color section
    /*ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>("clear");

    std_srvs::Empty srv;

    clear_client.call(srv);*/

    // spawn section
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response res;
    req.x = 2;
    req.y = 3;
    req.theta = 1.5;
    req.name = "turtle2";
    bool success = spawn_client.call(req, res);

    return 0;
}