#ifndef XSP_DEBUG_OBJECT_H
#define XSP_DEBUG_OBJECT_H

#include <string>
#include <iostream>

class DebugObject
{
private:
    std::string str;
public:
    DebugObject(std::string str_)
    : str(str_){
        std::cout<<"-->"<<str<<std::endl;
    }

    ~DebugObject() {
        std::cout<<"<--"<<str<<std::endl;
    }
};

#endif
