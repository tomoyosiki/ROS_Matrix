#include "ros/ros.h"
#include <ros_matrix/Mat_int.h>
#include <ros_matrix/Matrix.h>
#include <ros_matrix/Matrix_mul.h>
#include <ros_matrix/return_mat.h>
#include <iostream>
#include <chrono>
#include <sched.h>
#include <sstream>
#include <string>
#include <ros/callback_queue.h>

ros_matrix::Matrix Omat;
bool getResult = false;
void testCallback(const ros_matrix::Matrix::ConstPtr& msg){
    ROS_INFO_STREAM("Start Callback");
    int nRow = msg->nrow;
    int nCol = msg->ncol;
    for(int i = 0; i < nRow * nCol; i++){
        int a = (int)Omat.data[i].elem;
        int b = (int)msg->data[i].elem;
        if(a != b){
            ROS_INFO_STREAM("Calculation False");
        }
    }
}

bool returnCallback(ros_matrix::return_mat::Request& req, ros_matrix::return_mat::Response&){
  ROS_INFO_STREAM("Return Callback");
    int nRow = req.Omat.nrow;
    int nCol = req.Omat.ncol;
    for(int i = 0; i < nRow * nCol; i++){
        int a = (int)Omat.data[i].elem;
        int b = (int)req.Omat.data[i].elem;
        if(a != b){
            ROS_INFO_STREAM("Calculation False");
        }
    }
    ROS_INFO_STREAM("Calculation Success");
    getResult = true;
    return true;
}

int main(int argc, char **argv){
    // System Setting, keep process running on appointed core
    
    /*
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = 50;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0) {
        perror("Problem setting scheduling policy to SCHED_FIFO (probably need rtprio rule in /etc/security/limits.conf)");
        exit(1);
    }

    cpu_set_t my_set;
    CPU_ZERO(&my_set);
    CPU_SET(7, &my_set);
    sched_setaffinity(0, sizeof(cpu_set_t), &my_set);
    */

    ros::WallTime start_, end_;
    
    std::string nodeName = argv[1];

    ROS_INFO_STREAM(nodeName);
    start_ = ros::WallTime::now();
    ros::init(argc, argv, nodeName);


    ROS_INFO_STREAM("Start");
    ros::NodeHandle n;
    std::string arg = argv[2];
    std::size_t pos;
    int nRow = std::stoi(arg, &pos);
    int nCol = nRow;
    
    ros::Rate loopRate(10);
    ros::Publisher pub = n.advertise<ros_matrix::Matrix_mul>("mat_cal", 1000);
    ROS_INFO_STREAM("get subscribers");
    while(ros::ok() && pub.getNumSubscribers() <= 0){
        loopRate.sleep();
    }

    ros::ServiceServer return_srv_ = n.advertiseService("return", returnCallback);

    ros_matrix::Matrix_mul mats;
    ros_matrix::Mat_int elem;
    
    Omat.nrow = nRow;
    Omat.ncol = nRow;
    mats.Lmat.nrow = nRow;
    mats.Lmat.ncol = nCol;
    mats.Rmat.nrow = nRow;
    mats.Rmat.ncol = nCol;
    nodeName.append("_topic");
    mats.Rtopic = nodeName;
    for(int i = 0; i < nRow * nCol; i++){
        if(i % 2 == 0){
            elem.elem = 1;
            mats.Lmat.data.push_back(elem);
            elem.elem = 2;
            mats.Rmat.data.push_back(elem);
        }else{
            elem.elem = 2;
            mats.Lmat.data.push_back(elem);
            elem.elem = 1;
            mats.Rmat.data.push_back(elem);
        }
        elem.elem = 0;
        Omat.data.push_back(elem);
    }
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Data Preparation (ms): " << execution_time);

    start_ = ros::WallTime::now();
    long sum = 0;
    for(int i = 0; i < nRow; i++){
        for(int j = 0; j < nCol; j++){
            for(int k = 0; k < nCol; k++){
                int a = (int)mats.Lmat.data[i * nRow + k].elem;
                int b = (int)mats.Rmat.data[k * nRow + j].elem;
                Omat.data[i * nRow + j].elem += a * b;
            }
        }
    }
    end_ = ros::WallTime::now();
    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time on local (ms): " << execution_time);


    ROS_INFO("start to send data");
    start_ = ros::WallTime::now();
    
    ros::Subscriber sub = n.subscribe(nodeName, 0, testCallback);
    // shake hand
    int count = 0;
    while(ros::ok() && count < 1){
        pub.publish(mats);
        //auto time_span = static_cast<std::chrono::duration<double>>(end - start);
        //std::cout<< " sending data took: "<<time_span.count()<<" seconds !!!" << std::endl;
        ROS_INFO("end send data, go rest ....");
        //ros::spinOnce();

        //loop_rate.sleep();
        count++;
    }
    end_ = ros::WallTime::now();
    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time1 on Remote (ms): " << execution_time);

    start_ = ros::WallTime::now();
    while(ros::ok()){
        ros::spinOnce();;
        if(getResult){
            break;
        }
    }
    sub.shutdown(); 
    end_ = ros::WallTime::now();
    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time2 on Remote (ms): " << execution_time);
    //ros::spin();

    return 0;
}
