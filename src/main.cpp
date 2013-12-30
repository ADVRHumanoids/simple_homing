#include <yarp/os/all.h>
#include "simple_homing.h"

#define dT 0.01 //[s]

class simple_homing_module: public yarp::os::RFModule
{
protected:
    simple_homing* thr;
public:
    bool my_configure(int argc, char* argv[])
    {
        thr = new simple_homing(dT, argc, argv);
        if(!thr->start())
        {
            delete thr;
            return false;
        }
        std::cout<<"Starting Module"<<std::endl;
        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;
        return true;
    }

    virtual double getPeriod(){return 1.0;}
    virtual bool updateModule(){return true;}
};

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cout<<"yarpserver not running, pls run yarpserver"<<std::endl;
        return 0;}

    simple_homing_module mod;
    mod.my_configure(argc, argv);

    return mod.runModule();
}
