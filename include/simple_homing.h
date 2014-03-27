#ifndef _SIMPLE_HOMING_H_
#define _SIMPLE_HOMING_H_

#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>
#include "yarp_interface.h"
#include "file_parser.h"

#define DEFAULT_MAX_VEL 50.0 //[deg/sec]
#define PRECISION 0.1 //[deg]

using namespace yarp::sig;

class simple_homing: public yarp::os::RateThread
{
public:
    simple_homing(const double period, int argc, char* argv[]):
        iYarp(),
        RateThread(int(period*1000.0)),
        parser(argc, argv)
    {
        int torso_dofs;
//        iYarp.encodersMotor_torso->getAxes(&torso_dofs);
        int left_arm_dofs, right_arm_dofs;
//        iYarp.encodersMotor_left_arm->getAxes(&left_arm_dofs);
 //       iYarp.encodersMotor_right_arm->getAxes(&right_arm_dofs);
        int left_leg_dofs, right_leg_dofs;
//        iYarp.encodersMotor_left_leg->getAxes(&left_leg_dofs);
 //       iYarp.encodersMotor_right_leg->getAxes(&right_leg_dofs);

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

        _period = period;
        std::cout<<"Control Rate is: "<<_period*1000.0<<" [ms]"<<std::endl;
        max_q_increment = max_vel*_period; //[deg]

        set_init_config = false;
        _t_counter_sending = 0;
        _t_counter_reading = 0;
        _t_period = period;
    }

    virtual bool threadInit();

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


    bool checkGoal(const Vector& q, const Vector& q_goal)
    {
        for(unsigned int i = 0; i < q.size(); ++i){
            if(!(fabs(q[i]-q_goal[i]) <= PRECISION))
                return false;
        }
        return true;
    }

    void controlLaw(const Vector& homing_vector, const double max_q_increment, Vector& q)
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
    void controlAndMove(walkman::drc::yarp_single_chain_interface& chain, Vector& q_homing, double max_q_increment, Vector& q);
    
    std::string computeStatus()
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
};

#endif
