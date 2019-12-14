#include "ros/ros.h"
#include <ros_matrix/Mat_int.h>
#include <ros_matrix/Matrix.h>
#include <ros_matrix/Matrix_mul.h>
#include <ros_matrix/Mat_int.h>
#include "std_msgs/String.h"
#include <iostream>
#include <chrono>
#include <sched.h>
#include <vector>
#include <ros/transport_hints.h>
#include <ros_matrix/return_mat.h>
#include <ros_matrix/send_mat.h>

struct _Context{
    ros_matrix::Matrix Lmat;
    ros_matrix::Matrix Rmat;
    ros_matrix::Matrix Omat;
    int i;
    int j;
    int k;
    int period;
    std::string Service;
};
typedef struct _Context context;
std::vector<context> Contexts;
context curCtx;
int curContextId = -1;
int i;
int j;
int k;
ros_matrix::Matrix *Lmat;
ros_matrix::Matrix *Rmat;
ros_matrix::Matrix *Omat;
int Size;
std::string Service;
bool start_sig = false;

int maxPeriod = 0;


bool taskCallback(ros_matrix::send_mat::Request& req, ros_matrix::send_mat::Response&){
    std::cout << "start call back" << std::endl;
    ROS_INFO("start call back");
    std::chrono::steady_clock sc;
    auto start = sc.now();
    //const ros_matrix::Matrix lmat = msg->Lmat;
    //const ros_matrix::Matrix Rmat = msg->Rmat;
    context newCtx;
    newCtx.i = 0;
    newCtx.j = 0;
    newCtx.k = 0;
    newCtx.Service = req.Service;
    newCtx.period = req.Period;;
    
    int nRow = req.Lmat.nrow;
    int nCol = req.Lmat.ncol;
    
    ros_matrix::Mat_int elem;
    for(int i = 0; i < nRow * nCol; i++){
        newCtx.Lmat.data.push_back(req.Lmat.data[i]);
        newCtx.Rmat.data.push_back(req.Rmat.data[i]);

        elem.elem = 0;
        newCtx.Omat.data.push_back(elem);
    }
    newCtx.Rmat.nrow = nRow;
    newCtx.Rmat.ncol = nCol;
    newCtx.Lmat.nrow = nRow;
    newCtx.Lmat.ncol = nCol;
    newCtx.Omat.nrow = nRow;
    newCtx.Omat.ncol = nCol;

    if(Contexts.size() == 0 || maxPeriod <= newCtx.period){
        Contexts.push_back(newCtx);
        maxPeriod = newCtx.period;
    }else{
        for(int i = 0; i < Contexts.size(); i++){
            if(Contexts[curContextId].period > newCtx.period){
                Contexts.insert(Contexts.begin() + i, newCtx);
                break;
            }
        }
    }

    for(int i = 0; i < Contexts.size(); i++){
        ROS_INFO_STREAM("Context " << i << " is " << Contexts[i].Service);
    }
    
    
    auto end = sc.now();

    auto time_span = static_cast<std::chrono::duration<double>>(end - start); 
    
    std::cout<< "callback end " << Contexts.size() << std::endl;
    for(int i = 0; i < Contexts.size(); i++){
        std::cout<<" Size: "<< Contexts[i].Lmat.data.size()<< std::endl;
    }
    
    Rmat = &Contexts[curContextId].Rmat;  
    Lmat = &Contexts[curContextId].Lmat;
    Omat = &Contexts[curContextId].Omat;
    std::cout<<" Operation took: "<<time_span.count()<<" seconds !!!" << std::endl;
    return true;
}

void timerCallback(const ros::TimerEvent&){
    //std::cout<< "Scheduling " << Contexts.size() << std::endl;
    //std::chrono::steady_clock sc;
    //auto start = sc.now();
    if(curContextId != -1){
        Contexts[curContextId].i = i;
        Contexts[curContextId].j = j;
        Contexts[curContextId].k = k;
        Contexts[curContextId].Service = Service;
        curContextId = -1;
    }

    if(Contexts.size() > 0){
        curContextId = Contexts.size() - 1;
        i = Contexts[curContextId].i;
        j = Contexts[curContextId].j;
        k = Contexts[curContextId].k;
        Service = Contexts[curContextId].Service;
        Rmat = &Contexts[curContextId].Rmat;
        
        Lmat = &Contexts[curContextId].Lmat;
        //std::cout<<" Lmat Size: "<< Lmat->data.size()<< std::endl;
        Omat = &Contexts[curContextId].Omat;
        //std::cout<<" Omat Size: "<< Omat->data.size()<< " Omat max Size " << Omat->data.max_size() << std::endl;
        
        Size = Contexts[curContextId].Rmat.nrow;
        std::cout << "Task " << Service << " curContextId " << curContextId << " i :" << i << " j: " << j << " k:" << k << std::endl;
        //Contexts.clear();
    }
    //auto end = sc.now();
    //auto time_span = static_cast<std::chrono::duration<double>>(end - start); 
    //std::cout<<" Scheduling took: "<<time_span.count()<<" seconds !!!" << std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "mat_srv");

    ros::NodeHandle n;
    //ros::Subscriber sub = n.subscribe("timer", 1000, timerCallback);
    ros::ServiceServer task_srv_ = n.advertiseService("mat_cal_srv", taskCallback);

    ros::WallTime start_, end_;
    double execution_time = 0.0;

    std::string arg = argv[1];
    std::size_t pos;
    int fq = std::stoi(arg, &pos);

    ros::Timer timer = n.createTimer(ros::Duration(1.0/fq), timerCallback);

    /*
    while(ros::ok()){
        //ROS_INFO_STREAM("wait");
        ros::spinOnce();;
        if(start_sig){
            break;
        }
    }
    */

    ROS_INFO("start listen");
    while(ros::ok()){
        if(curContextId >= 0){
            //start_ = ros::WallTime::now();
            int a = (int)Lmat->data[i * Size + k].elem;
            int b = (int)Rmat->data[k * Size + j].elem;
            Omat->data[i * Size + j].elem += a * b;
            
            if(k < Size - 1){
                k++;
                //end_ = ros::WallTime::now();
                //execution_time += (end_ - start_).toNSec() * 1e-6;
            }else{
                k = 0;
                if(j < Size - 1){
                    j++;
                    //end_ = ros::WallTime::now();
                    //execution_time += (end_ - start_).toNSec() * 1e-6;
                }else{
                    j = 0;
                    if(i < Size - 1){
                        i++;
                        //end_ = ros::WallTime::now();
                        //execution_time += (end_ - start_).toNSec() * 1e-6;
                    }else{
                        start_ = ros::WallTime::now();
                        i = 0;
                        int count = 0;
                        ROS_INFO_STREAM("Service name " << Service);
                        ros::ServiceClient return_cli = n.serviceClient<ros_matrix::return_mat>(Service, true);
                        return_cli.waitForExistence();
                        
                        ros_matrix::return_mat _mat;
                        ros_matrix::Mat_int elem;
                        _mat.request.Omat.nrow = Omat->nrow;
                        _mat.request.Omat.ncol = Omat->ncol;
                        for(int k = 0; k < Omat->nrow * Omat->ncol; k++){
                            elem.elem = Omat->data[k].elem;
                            _mat.request.Omat.data.push_back(elem);
                        }
                        return_cli.call(_mat);
                        ROS_INFO_STREAM("Suc");
                        Contexts.pop_back();
                        curContextId = -1;
                        /*
                        ros::Publisher pub = n.advertise<ros_matrix::Matrix>(Contexts[curContextId].topic, 1000);
                        ROS_INFO_STREAM("Topic " << Contexts[curContextId].topic << " is checking");
                        while(pub.getNumSubscribers() <= 0){
                            loop_rate.sleep();
                        }
                        ROS_INFO_STREAM("Topic " << Contexts[curContextId].topic << " is publishing");
                        while(pub.getNumSubscribers() != 0){
                            pub.publish(Contexts[curContextId].Omat);
                        }
                        std::cout << "Topic is " << Contexts[curContextId].topic << " Current Context Id : " << curContextId << " finish" << std::endl;
                        Contexts.pop_back();
                        curContextId = -1;
                        pub.shutdown();
                        */
                        return_cli.shutdown();
                        end_ = ros::WallTime::now();
                        execution_time += (end_ - start_).toNSec() * 1e-6;
                        ROS_INFO_STREAM("Exectution time on local (ms): " << execution_time);
                        //execution_time = 0.0;
                        
                    }
                }
            }
            //pub.publish(Contexts[curContextId].Omat);
        }

        ros::spinOnce();
        /*
        while(!ros::getGlobalCallbackQueue()->isEmpty()){
            ros::getGlobalCallbackQueue()->callOne();
        }
        */
    }
    
    
    return 0;
}
