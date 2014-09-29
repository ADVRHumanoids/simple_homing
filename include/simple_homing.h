#ifndef _SIMPLE_HOMING_H_
#define _SIMPLE_HOMING_H_

#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>
#include "yarp_interface.h"
#include "file_parser.h"

#include<drc_shared/generic_thread.hpp>

using namespace yarp::sig;

class simple_homing: public generic_thread
{
public:
    simple_homing( std::string module_prefix, 
                   yarp::os::ResourceFinder rf, 
                   std::shared_ptr<paramHelp::ParamHelperServer> ph );

    virtual bool custom_init();

    virtual void run();

private:
    yarp_interface iYarp;
    file_parser parser;

    Vector q_torso;
    Vector q_left_arm;
    Vector q_right_arm;
    Vector q_left_leg;
    Vector q_right_leg;

    Vector torso_homing;
    Vector left_arm_homing;
    Vector right_arm_homing;
    Vector left_leg_homing;
    Vector right_leg_homing;

    double max_vel;
    double max_q_increment;
    double _period;
    bool set_init_config;
    unsigned int _t_counter_sending;
    unsigned int _t_counter_reading;
    double _t_period; //[sec]


    bool checkGoal(const Vector& q, const Vector& q_goal);

    void controlLaw(const Vector& homing_vector, const double max_q_increment, Vector& q);

    void controlAndMove(walkman::drc::yarp_single_chain_interface& chain, Vector& q_homing, double max_q_increment, Vector& q);
    
    std::string computeStatus();
    
};

#endif
