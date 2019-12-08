#include "ros/ros.h"
#include <ros_matrix/Mat_int.h>
#include <ros_matrix/Matrix.h>
#include <ros_matrix/Matrix_mul.h>
#include "std_msgs/String.h"
#include <iostream>
#include <chrono>
#include <sched.h>
#include <vector>
#include <ros/transport_hints.h>

struct _Context{
    ros_matrix::Matrix Lmat;
    ros_matrix::Matrix Rmat;
    ros_matrix::Matrix Omat;
    int i;
    int j;
    int k;
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

void testCallback(const ros_matrix::Matrix_mul::ConstPtr& msg){
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
    
    int nRow = msg->Lmat.nrow;
    int nCol = msg->Lmat.ncol;
    
    ros_matrix::Mat_int elem;
    for(int i = 0; i < nRow * nCol; i++){
        newCtx.Lmat.data.push_back(msg->Lmat.data[i]);
        newCtx.Rmat.data.push_back(msg->Rmat.data[i]);

        elem.elem = 0;
        newCtx.Omat.data.push_back(elem);
    }
    newCtx.Rmat.nrow = nRow;
    newCtx.Rmat.ncol = nCol;
    newCtx.Lmat.nrow = nRow;
    newCtx.Lmat.ncol = nCol;
    newCtx.Omat.nrow = nRow;
    newCtx.Omat.ncol = nCol;

    Contexts.push_back(newCtx);
    auto end = sc.now();

    auto time_span = static_cast<std::chrono::duration<double>>(end - start); 
    std::cout<< "callback end " << Contexts.size() << std::endl;
    for(int i = 0; i < Contexts.size(); i++){
        std::cout<<" Lmat Size: "<< Contexts[i].Lmat.data.size()<< std::endl;
        std::cout<<" Rmat Size: "<< Contexts[i].Rmat.data.size()<< std::endl;
        std::cout<<" Omat Size: "<< Contexts[i].Omat.data.size()<< std::endl;
    }
    Rmat = &Contexts[curContextId].Rmat;  
    Lmat = &Contexts[curContextId].Lmat;
    Omat = &Contexts[curContextId].Omat;
    std::cout<<" Operation took: "<<time_span.count()<<" seconds !!!" << std::endl;
}

void timerCallback(const std_msgs::String::ConstPtr& msg){
    std::cout<< "Scheduling " << Contexts.size() << std::endl;
    std::chrono::steady_clock sc;
    auto start = sc.now();
    if(curContextId != -1){
        Contexts[curContextId].i = i;
        Contexts[curContextId].j = j;
        Contexts[curContextId].k = k;
        curContextId = -1;
    }

    if(Contexts.size() > 0){
        curContextId = Contexts.size() - 1;
        i = Contexts[curContextId].i;
        j = Contexts[curContextId].j;
        k = Contexts[curContextId].k;
        Rmat = &Contexts[curContextId].Rmat;
        
        Lmat = &Contexts[curContextId].Lmat;
        //std::cout<<" Lmat Size: "<< Lmat->data.size()<< std::endl;
        Omat = &Contexts[curContextId].Omat;
        //std::cout<<" Omat Size: "<< Omat->data.size()<< " Omat max Size " << Omat->data.max_size() << std::endl;
        
        Size = Contexts[curContextId].Rmat.nrow;
        std::cout << "Job " << curContextId << " i :" << i << " j: " << j << " k:" << k << std::endl;
        //Contexts.clear();
    }
    auto end = sc.now();
    auto time_span = static_cast<std::chrono::duration<double>>(end - start); 
    std::cout<<" Scheduling took: "<<time_span.count()<<" seconds !!!" << std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("test", 1000, testCallback);
    ros::Subscriber sub2 = n.subscribe("timer", 1000, timerCallback);

    ROS_INFO("start listen");
    while(ros::ok()){
        if(curContextId >= 0){
            //std::cout << "Current Context Id :" << curContextId << std::endl;
            //std::cout << "i :" << i << " j: " << j << " k:" << k << std::endl;
            //std::cout << "Size is " << Size << std::endl;
            int a = (int)Lmat->data[i * Size + k].elem;
            int b = (int)Rmat->data[k * Size + j].elem;
            //std::cout<<" a: "<< a << " b: " << b <<  std::endl;
            /*
            if(k = 0){
                ros_matrix::Mat_int elem;
                elem.elem = 0;
                std::cout<<" Omat Size: "<< Omat->data.size()<< std::endl;
                Omat->data.push_back(elem);
                std::cout<<" Omat Size: "<< Omat->data.size()<< std::endl;
            }
            */

            Omat->data[i * Size + j].elem += a * b;
            if(k < Size - 1){
                k++;
            }else{
                k = 0;
                if(j < Size - 1){
                    j++;
                }else{
                    j = 0;
                    if(i < Size - 1){
                        i++;
                    }else{
                        i = 0;
                        std::cout << "Current Context Id : " << curContextId << " finish" << std::endl;
                        Contexts.pop_back();
                        curContextId = -1;
                    }
                }
            }
        }


        ros::spinOnce();
    }
    
    
    return 0;
}
