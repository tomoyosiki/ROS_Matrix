#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv){
    ros::init(argc, argv, "timer");
    ros::NodeHandle n;

    //ros::ServiceClient task_cli = n.serviceClient<ros_matrix::send_mat>("start");
    return 0;
}