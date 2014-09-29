#include <yarp/os/all.h>
#include <stdlib.h>
#include "simple_homing.h"

#define DEFAULT_MAX_VEL 50.0 //[deg/sec]
#define PRECISION 0.1 //[deg]
#define SENDING_TIMING 0.2 //[sec]
#define READING_TIMING 0.2 //[sec]

simple_homing::simple_homing(std::string module_prefix, 
                             yarp::os::ResourceFinder rf, 
                             std::shared_ptr< paramHelp::ParamHelperServer > ph) :  iYarp(),
                                                                                    parser( rf ),
                                                                                    generic_thread(module_prefix, rf, ph)
{
    int torso_dofs;
    int left_arm_dofs, right_arm_dofs;
    int left_leg_dofs, right_leg_dofs;

    q_torso.resize(iYarp.torso.getNumberOfJoints(),0.0);
    q_left_arm.resize(iYarp.left_arm.getNumberOfJoints(),0.0);
    q_right_arm.resize(iYarp.right_arm.getNumberOfJoints(),0.0);
    q_left_leg.resize(iYarp.left_leg.getNumberOfJoints(),0.0);
    q_right_leg.resize(iYarp.right_leg.getNumberOfJoints(),0.0);

    torso_homing.resize(q_torso.size(),0.0);
    left_arm_homing.resize(q_left_arm.size(),0.0);
    right_arm_homing.resize(q_right_arm.size(),0.0);
    left_leg_homing.resize(q_left_leg.size(),0.0);
    right_leg_homing.resize(q_right_leg.size(),0.0);

    max_vel = DEFAULT_MAX_VEL; //[deg/sec]

    _period = get_thread_period();
    std::cout<<"Control Rate is: "<< _period <<" [ms]"<<std::endl;
    max_q_increment = max_vel*( static_cast<double>( _period )/1000 ); //[deg]

    set_init_config = false;
    _t_counter_sending = 0;
    _t_counter_reading = 0;
    _t_period = get_thread_period();
}



bool simple_homing::custom_init()
{
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);

    if(!parser.getConfiguration("torso", torso_homing))
        std::cout<<"Error loading torso homing. All zeros will be used."<<std::endl;
    if(!parser.getConfiguration("left_arm", left_arm_homing))
        std::cout<<"Error loading left_arm homing. All zeros will be used."<<std::endl;
    if(!parser.getConfiguration("right_arm", right_arm_homing))
        std::cout<<"Error loading right_arm homing. All zeros will be used."<<std::endl;
    if(!parser.getConfiguration("left_leg", left_leg_homing))
        std::cout<<"Error loading left_leg homing. All zeros will be used."<<std::endl;
    if(!parser.getConfiguration("right_leg", right_leg_homing))
        std::cout<<"Error loading right_leg homing. All zeros will be used."<<std::endl;
    if(!parser.getMaxVelocity(max_vel))
        std::cout<<"Error loading max vel. "<< max_vel<<" [deg/sec] will be used."<<std::endl;
    else
        max_q_increment = max_vel*( static_cast<double>( _period )/1000 ); //[deg]

    return true;
}

void simple_homing::controlAndMove(walkman::drc::yarp_single_chain_interface& chain,yarp::sig::Vector& q_homing,double max_q_increment,yarp::sig::Vector& q)
{
    if(chain.isAvailable){
        controlLaw(q_homing, max_q_increment, q);
        iYarp.fillBottleAndSend(q, chain.getChainName());        
        chain.move(q);
    }
}

void simple_homing::run()
{   
    if((double)_t_counter_reading*_t_period >= READING_TIMING)
    {
        iYarp.checkInput();
        _t_counter_reading = 0;
    }

    if(iYarp.sendTrj())
    {
        if(!set_init_config){
            iYarp.torso.encodersMotor->getEncoders(q_torso.data());
            iYarp.left_arm.encodersMotor->getEncoders(q_left_arm.data());
            iYarp.right_arm.encodersMotor->getEncoders(q_right_arm.data());
            iYarp.left_leg.encodersMotor->getEncoders(q_left_leg.data());
            iYarp.right_leg.encodersMotor->getEncoders(q_right_leg.data());
            set_init_config = true;
        }

        if(checkGoal(q_left_arm, left_arm_homing) &&
           checkGoal(q_right_arm, right_arm_homing) &&
           checkGoal(q_left_leg, left_leg_homing) &&
           checkGoal(q_right_leg, right_leg_homing) &&
           checkGoal(q_torso, torso_homing))
        {
            iYarp.stop();
            set_init_config = false;
            std::cout<<"Reached desired homing position"<<std::endl;
        }
        else
        {
             controlAndMove(iYarp.torso,torso_homing,max_q_increment,q_torso);
             controlAndMove(iYarp.right_arm,right_arm_homing,max_q_increment,q_right_arm);
             controlAndMove(iYarp.right_leg,right_leg_homing,max_q_increment,q_right_leg);
             controlAndMove(iYarp.left_arm,left_arm_homing,max_q_increment,q_left_arm);
             controlAndMove(iYarp.left_leg,left_leg_homing,max_q_increment,q_left_leg);
        }
    }

//     if((double)_t_counter_sending*_t_period >= SENDING_TIMING)
//     {
        iYarp.status.setStatus(computeStatus());
//         _t_counter_sending = 0;
//     }
// 
//     _t_counter_sending++;
    _t_counter_reading++;
}

bool simple_homing::checkGoal(const Vector& q, const Vector& q_goal)
{
    for(unsigned int i = 0; i < q.size(); ++i){
        if(!(fabs(q[i]-q_goal[i]) <= PRECISION))
            return false;
    }
    return true;
}

void simple_homing::controlLaw(const Vector& homing_vector, const double max_q_increment, Vector& q)
{
    unsigned int number_of_dofs = homing_vector.size();
    Vector delta_q(number_of_dofs);
    for(unsigned int i = 0; i < number_of_dofs; ++i)
    {
        delta_q[i] = homing_vector[i] - q[i];
        if(fabs(delta_q[i]) > max_q_increment)
            delta_q[i] = (delta_q[i]/fabs(delta_q[i])) * max_q_increment;
        q[i] += delta_q[i];
    }
}

std::string simple_homing::computeStatus()
{
    if(iYarp.sendTrj())
        return "moving";
    else
    {
        Vector q_larm(q_left_arm.size());
        Vector q_rarm(q_right_arm.size());
        Vector q_lleg(q_left_leg.size());
        Vector q_rleg(q_right_leg.size());
        Vector q_t(q_torso.size());
        iYarp.torso.sense(q_t);
        iYarp.left_arm.sense(q_larm);
        iYarp.right_arm.sense(q_rarm);
        iYarp.left_leg.sense(q_lleg);
        iYarp.right_leg.sense(q_rleg);
        if(checkGoal(q_larm, left_arm_homing) &&
            checkGoal(q_rarm, right_arm_homing) &&
            checkGoal(q_lleg, left_leg_homing) &&
            checkGoal(q_rleg, right_leg_homing) &&
            checkGoal(q_t, torso_homing))
            return "home";
        return "ready";
    }
}


