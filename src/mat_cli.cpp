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
#include <ros_matrix/send_mat.h>

ros_matrix::Matrix Omat;
bool getResult = false;
int releasedJob = 0;
int executedJob = 0;

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
            return false;
        }
    }
    ROS_INFO_STREAM("Calculation Success");
    getResult = true;
    return true;
}

void timerCallback(const ros::TimerEvent&){
    ROS_INFO_STREAM("release " << releasedJob << " executedJob " << executedJob << " waiting new job");
    releasedJob += 1;
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
    
    ros::init(argc, argv, nodeName);


    ROS_INFO_STREAM("Start");
    ros::NodeHandle n;
    std::string row_arg = argv[2];
    std::size_t pos;
    int nRow = std::stoi(row_arg, &pos);
    int nCol = nRow;
    std::string period_arg = argv[3];
    int period = std::stoi(period_arg);

    ros::ServiceServer return_srv_;
    ros::ServiceClient task_cli = n.serviceClient<ros_matrix::send_mat>("mat_cal_srv");

    ros::Timer timer;
    bool ifPeriodic = false;
    if(period > 0){
        ifPeriodic= true;
        timer = n.createTimer(ros::Duration(period / 1000.0), timerCallback);
    }else{
        releasedJob = 1;
    }
    
    
    
    while(true){
        ros::spinOnce();
        if(releasedJob == executedJob){
            
            //ROS_INFO_STREAM("release " << releasedJob << " executedJob " << executedJob << " waiting new job");
            continue;
        }

        std::string returnSrvName = "return_";
        returnSrvName = returnSrvName + nodeName + std::to_string(executedJob);
        return_srv_ = n.advertiseService(returnSrvName, returnCallback);
        
        start_ = ros::WallTime::now();
        ros_matrix::send_mat mats;
        ros_matrix::Mat_int elem;
        Omat.data.clear();
        Omat.nrow = nRow;
        Omat.ncol = nRow;
        mats.request.Lmat.nrow = nRow;
        mats.request.Lmat.ncol = nCol;
        mats.request.Rmat.nrow = nRow;
        mats.request.Rmat.ncol = nCol;
        mats.request.Service = returnSrvName;
        mats.request.Period = period;
        for(int i = 0; i < nRow * nCol; i++){
            if(i % 2 == 0){
                elem.elem = 1;
                mats.request.Lmat.data.push_back(elem);
                elem.elem = 2;
                mats.request.Rmat.data.push_back(elem);
            }else{
                elem.elem = 2;
                mats.request.Lmat.data.push_back(elem);
                elem.elem = 1;
                mats.request.Rmat.data.push_back(elem);
            }
            elem.elem = 0;
            Omat.data.push_back(elem);
        }
        end_ = ros::WallTime::now();
        double execution_time = (end_ - start_).toNSec() * 1e-6;
        ROS_INFO_STREAM("Data Preparation (ms): " << execution_time);

        /*
        start_ = ros::WallTime::now();
        long sum = 0;
        for(int i = 0; i < nRow; i++){
            for(int j = 0; j < nCol; j++){
                for(int k = 0; k < nCol; k++){
                    int a = (int)mats.request.Lmat.data[i * nRow + k].elem;
                    int b = (int)mats.request.Rmat.data[k * nRow + j].elem;
                    Omat.data[i * nRow + j].elem += a * b;
                }
            }
        }
        end_ = ros::WallTime::now();
        execution_time = (end_ - start_).toNSec() * 1e-6;
        ROS_INFO_STREAM("Exectution time on local (ms): " << execution_time);
        */

        start_ = ros::WallTime::now();
        int i, j, k;
        i = j = k= 0;
        while(1){
            int a = (int)mats.request.Lmat.data[i * nRow + k].elem;
            int b = (int)mats.request.Rmat.data[k * nRow + j].elem;
            Omat.data[i * nRow + j].elem += a * b;
                if(k < nRow - 1){
                    k++;
                }else{
                    k = 0;
                    if(j < nRow - 1){
                        j++;
                    }else{
                        j = 0;
                        if(i < nRow - 1){
                            i++;
                        }else{
                            i = 0;
                            break;
                        }
                    }
                }
        }
        end_ = ros::WallTime::now();
        execution_time = (end_ - start_).toNSec() * 1e-6;
        ROS_INFO_STREAM("Exectution time on local (ms): " << execution_time);


        ROS_INFO("start to send data");
        start_ = ros::WallTime::now();
        
        while(!task_cli.call(mats)){
            ROS_INFO_STREAM("fail");
        }
        ROS_INFO_STREAM("Suc");
        end_ = ros::WallTime::now();
        execution_time = (end_ - start_).toNSec() * 1e-6;
        ROS_INFO_STREAM("Exectution time1 on Remote (ms): " << execution_time);

        start_ = ros::WallTime::now();
        while(ros::ok()){
            //ROS_INFO_STREAM("wait");
            ros::spinOnce();
            if(getResult){
                break;
            }
        }
        getResult = false;
        executedJob += 1;
        return_srv_.shutdown(); 
        end_ = ros::WallTime::now();
        execution_time = (end_ - start_).toNSec() * 1e-6;
        ROS_INFO_STREAM("Exectution time2 on Remote (ms): " << execution_time);
        if(!ifPeriodic){
            break;
        }
    }
    
    return 0;
}
