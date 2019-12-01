#include "ros/ros.h"
#include <ros_matrix/Mat_int.h>
#include <ros_matrix/Matrix.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<ros_matrix::Matrix>("test", 1000);

    ros::Rate loop_rate(1);
    int count = 0;
    ros_matrix::Matrix mat;
    // hand shaker
    pub.publish(mat);
    loop_rate.sleep();
    while(ros::ok() && count < 10){
        ros_matrix::Matrix mat;
        ros_matrix::Mat_int elem;
        int nRow = 2;
        int nCol = 2;
        mat.nrow = nRow;
        mat.ncol = nCol;
        for(int i = 0; i < nRow * nCol; i++){
            elem.elem = i;
            mat.data.push_back(elem);
        }
        ROS_INFO("mat read");
        pub.publish(mat);
        ROS_INFO("mat send");
        loop_rate.sleep();
        count++;
    }
    return 0;
}
