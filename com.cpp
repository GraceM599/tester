//
// Created by mihir on 10/3/2025.
//
#include <iostream>
#include <vector>
#include <unistd.h>
#include "SCServo_Linux/SCServo.h"
#include <fcntl.h>
#include <cstdint>
#include <cstring>


SMS_STS sm_st;

int main(int argc, char **argv)
{
    if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
    }
    std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sm_st.begin(115200, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }
    int ID = sm_st.Ping(1);
    if(ID!=-1){
        std::cout<<"ID:"<<ID<<std::endl;
    }else{
        std::cout<<"Ping servo ID error!"<<std::endl;
    }
    sm_st.end();
    return 1;
}