#include "ros/ros.h"
#include <ros_matrix/Mat_int.h>
#include <ros_matrix/Matrix_mul.h>
#include <iostream>
#include <chrono>
#include <sched.h>
#include <sstream>

int main(int argc, char **argv){
    // System Setting, keep process running on appointed core
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


    ros::WallTime start_, end_;
    
    start_ = ros::WallTime::now();
    ros::init(argc, argv, "talker_loop");

    ros::NodeHandle n;
    std::string arg = argv[1];
    std::size_t pos;
    int nRow = std::stoi(arg, &pos);
    int nCol = nRow;

    ros::Publisher pub = n.advertise<ros_matrix::Matrix_mul>("test", nRow * nCol * 2);

    ros::Rate loop_rate(1);
    ros_matrix::Matrix_mul mats;
    ros_matrix::Mat_int elem;
    
    mats.Lmat.nrow = nRow;
    mats.Lmat.ncol = nCol;
    mats.Rmat.nrow = nRow;
    mats.Rmat.ncol = nCol;
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
    }
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time (ms): " << execution_time);


    ROS_INFO("start to send data");
    // shake hand
    int count = 0;
    auto sum = 0;
    while(ros::ok()){
        pub.publish(mats);
        //auto time_span = static_cast<std::chrono::duration<double>>(end - start);
        //std::cout<< " sending data took: "<<time_span.count()<<" seconds !!!" << std::endl;
        ROS_INFO("end send data, go rest ....");
        //ros::spinOnce();

        loop_rate.sleep();
        count++;
    }
    /*
    auto start = sc.now();
    long sum = 0;
    for(int i = 0; i < nRow; i++){
        for(int j = 0; j < nCol; j++){
            for(int k = 0; k < nCol; k++){
                int a = (int)mats.Lmat.data[i * nRow + k].elem;
                int b = (int)mats.Rmat.data[k * nRow + j].elem;
                sum += a * b;
            }
        }
    }
    auto end = sc.now();
    auto time_span = static_cast<std::chrono::duration<double>>(end - start); 
    std::cout<< sum << " Operation took: "<<time_span.count()<<" seconds !!!" << std::endl;
    */
    

    return 0;
}
