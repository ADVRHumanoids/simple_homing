#include <yarp/os/all.h>
#include <stdlib.h>
#include <file_parser.h>
#include <yarp/sig/Vector.h>
#include "yarp_interface.h"

using namespace yarp::os;
using namespace yarp::sig;

#define DEFAULT_MAX_VEL 5.0 //[deg/sec]
#define PRECISION 1.0 //[deg]
#define dT 0.01 //[s]

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

int main(int argc, char* argv[])
{ 
    Network yarp;
    if(!yarp.checkNetwork()){
        std::cout<<"yarpserver not running, pls run yarpserver"<<std::endl;
        return 0;}

    yarp_interface iYarp;
    int torso_dofs;
    iYarp.positionControl_torso->getAxes(&torso_dofs);
    int left_arm_dofs, right_arm_dofs;
    iYarp.positionControl_left_arm->getAxes(&left_arm_dofs);
    iYarp.positionControl_right_arm->getAxes(&right_arm_dofs);
    int left_leg_dofs, right_leg_dofs;
    iYarp.positionControl_left_leg->getAxes(&left_leg_dofs);
    iYarp.positionControl_right_leg->getAxes(&right_leg_dofs);

    Vector q_torso(torso_dofs);
    Vector q_left_arm(left_arm_dofs);
    Vector q_right_arm(right_arm_dofs);
    Vector q_left_leg(left_leg_dofs);
    Vector q_right_leg(right_leg_dofs);

    Vector torso_homing(q_torso.size());
    torso_homing.zero();
    Vector left_arm_homing(q_left_arm.size());
    left_arm_homing.zero();
    Vector right_arm_homing(q_right_arm.size());
    right_arm_homing.zero();
    Vector left_leg_homing(q_left_leg.size());
    left_leg_homing.zero();
    Vector right_leg_homing(q_right_leg.size());
    right_leg_homing.zero();
    double max_vel = DEFAULT_MAX_VEL; //[deg/sec]

    file_parser parser(argc, argv);

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

    double max_q_increment = max_vel*dT; //[deg]

    yarp::os::Time delay;
    while(true)
    {
        iYarp.checkInput();
        if(iYarp.sendTrj())
        {
            //CL
            iYarp.encodersMotor_torso->getEncoders(q_torso.data());
            iYarp.encodersMotor_left_arm->getEncoders(q_left_arm.data());
            iYarp.encodersMotor_right_arm->getEncoders(q_right_arm.data());
            iYarp.encodersMotor_left_leg->getEncoders(q_left_leg.data());
            iYarp.encodersMotor_right_leg->getEncoders(q_right_leg.data());

            if(checkGoal(q_left_arm, left_arm_homing) &&
               checkGoal(q_right_arm, right_arm_homing) &&
               checkGoal(q_left_leg, left_leg_homing) &&
               checkGoal(q_right_leg, right_leg_homing) &&
               checkGoal(q_torso, torso_homing))
            {
                iYarp.stop();
                std::cout<<"Reached desired homing position"<<std::endl;
            }
            else
            {

                controlLaw(left_arm_homing, max_q_increment, q_left_arm);
                controlLaw(right_arm_homing, max_q_increment, q_right_arm);
                controlLaw(left_leg_homing, max_q_increment, q_left_leg);
                controlLaw(right_leg_homing, max_q_increment, q_right_leg);
                controlLaw(torso_homing, max_q_increment, q_torso);

                iYarp.fillBottleAndSend(q_left_arm, "left_arm");
                iYarp.fillBottleAndSend(q_right_arm, "right_arm");
                iYarp.fillBottleAndSend(q_left_leg, "left_leg");
                iYarp.fillBottleAndSend(q_right_leg, "right_leg");
                iYarp.fillBottleAndSend(q_torso, "torso");

                iYarp.moveKinematicChain(q_left_arm, "left_arm", max_vel);
                iYarp.moveKinematicChain(q_right_arm, "right_arm", max_vel);
                iYarp.moveKinematicChain(q_left_leg, "left_leg", max_vel);
                iYarp.moveKinematicChain(q_right_leg, "right_leg", max_vel);
                iYarp.moveKinematicChain(q_torso, "torso", max_vel);

//                std::cout<<"Sending: "<<q_left_arm.toString()<<std::endl;
//                std::cout<<"Sending: "<<q_right_arm.toString()<<std::endl;
//                std::cout<<"Sending: "<<q_left_leg.toString()<<std::endl;
//                std::cout<<"Sending: "<<q_right_leg.toString()<<std::endl;
//                std::cout<<"Sending: "<<q_torso.toString()<<std::endl;
            }
        }

        delay.delay(dT);
    }


	return 0;
}
