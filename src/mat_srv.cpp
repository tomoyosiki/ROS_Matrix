#include "ros/ros.h"
#include <ros_matrix/Mat_int.h>
#include <ros_matrix/Matrix.h>

void testCallback(const ros_matrix::Matrix::ConstPtr& msg){
    int nRow = msg->nrow;
    int nCol = msg->ncol;
    for(int i = 0; i < nRow * nCol; i++){
        ROS_INFO("data %d is %d", i, msg->data[i]);
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("test", 1000, testCallback);

    ros::spin();

    return 0;
}
