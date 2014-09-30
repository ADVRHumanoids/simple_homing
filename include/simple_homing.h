#ifndef _SIMPLE_HOMING_H_
#define _SIMPLE_HOMING_H_

#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>
#include "yarp_interface.h"
#include "file_parser.h"

#include <drc_shared/yarp_single_chain_interface.h>
#include<drc_shared/generic_thread.hpp>

//param helper
#define PARAM_ID_TORSO 0
#define PARAM_SIZE_TORSO 3

using namespace yarp::sig;

class simple_homing: public generic_thread
{
private:
    
    file_parser parser;
    
    walkman::drc::yarp_single_chain_interface torso_chain_interface;
    walkman::drc::yarp_single_chain_interface left_arm_chain_interface;
    walkman::drc::yarp_single_chain_interface right_arm_chain_interface;
    walkman::drc::yarp_single_chain_interface left_leg_chain_interface;
    walkman::drc::yarp_single_chain_interface right_leg_chain_interface;

    yarp::sig::Vector torso_homing;
    yarp::sig::Vector left_arm_homing;
    yarp::sig::Vector right_arm_homing;
    yarp::sig::Vector left_leg_homing;
    yarp::sig::Vector right_leg_homing;

    double max_vel;
    bool set_init_config;

    void controlAndMove( walkman::drc::yarp_single_chain_interface& chain_interface, yarp::sig::Vector homing );
    std::string computeStatus();
    
public:
    
    simple_homing( std::string module_prefix, 
                   yarp::os::ResourceFinder rf, 
                   std::shared_ptr<paramHelp::ParamHelperServer> ph );
    virtual bool custom_init();
    virtual void run();
    
};

#endif
