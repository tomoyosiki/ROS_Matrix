#include "ros/ros.h"
#include <ros_matrix/Mat_int.h>
#include <ros_matrix/Matrix_mul.h>
#include <iostream>
#include <chrono>
#include <sched.h>

int main(int argc, char **argv){
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


    std::chrono::steady_clock sc;

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<ros_matrix::Matrix_mul>("test", 1000);

    ros::Rate loop_rate(1);
    int count = 0;
    ros_matrix::Matrix_mul mats;
    pub.publish(mats);
    ros_matrix::Mat_int elem;
    int nRow = 2000;
    int nCol = 2000;
    mats.Lmat.nrow = nRow;
    mats.Lmat.ncol = nCol;
    mats.Rmat.nrow = nRow;
    mats.Rmat.ncol = nCol;
    for(int i = 0; i < nRow * nCol; i++){
        elem.elem = 1;
        mats.Lmat.data.push_back(elem);
        elem.elem = 2;
        mats.Rmat.data.push_back(elem);
    }
    int test;
    std::cin >> test;
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

    pub.publish(mats);

    return 0;
}
