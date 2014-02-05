#include <yarp/os/all.h>
#include <stdlib.h>
#include "simple_homing.h"

using namespace yarp::os;
using namespace yarp::sig;

bool simple_homing::threadInit()
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
        max_q_increment = max_vel*_period; //[deg]

    return true;
}

void simple_homing::run()
{
    iYarp.checkInput();
    if(iYarp.sendTrj())
    {
        if(!set_init_config){
            iYarp.encodersMotor_torso->getEncoders(q_torso.data());
            iYarp.encodersMotor_left_arm->getEncoders(q_left_arm.data());
            iYarp.encodersMotor_right_arm->getEncoders(q_right_arm.data());
            iYarp.encodersMotor_left_leg->getEncoders(q_left_leg.data());
            iYarp.encodersMotor_right_leg->getEncoders(q_right_leg.data());
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
            if(iYarp.isTorsoAvailable){
                controlLaw(torso_homing, max_q_increment, q_torso);
                iYarp.fillBottleAndSend(q_torso, "torso");
                iYarp.moveKinematicChain(q_torso, "torso");
            }
            if(iYarp.isLeftArmAvailable){
                controlLaw(left_arm_homing, max_q_increment, q_left_arm);
                iYarp.fillBottleAndSend(q_left_arm, "left_arm");
                iYarp.moveKinematicChain(q_left_arm, "left_arm");
            }
            if(iYarp.isRightArmAvailable){
                controlLaw(right_arm_homing, max_q_increment, q_right_arm);
                iYarp.fillBottleAndSend(q_right_arm, "right_arm");
                iYarp.moveKinematicChain(q_right_arm, "right_arm");
            }
            if(iYarp.isLeftLegAvailable){
                controlLaw(left_leg_homing, max_q_increment, q_left_leg);
                iYarp.fillBottleAndSend(q_left_leg, "left_leg");
                iYarp.moveKinematicChain(q_left_leg, "left_leg");

            }
            if(iYarp.isRightLegAvailable){
                controlLaw(right_leg_homing, max_q_increment, q_right_leg);
                iYarp.fillBottleAndSend(q_right_leg, "right_leg");
                iYarp.moveKinematicChain(q_right_leg, "right_leg");
            }

        }
    }

    iYarp.fillStatusBottleAndSend(computeStatus());

}
