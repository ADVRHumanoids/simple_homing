#include "yarp_interface.h"
#include <iterator>
#include <algorithm>
#include <vector>
#include <iostream>

//#define USE_POSITION_CONTROL_LEFT_ARM true
//#define USE_POSITION_CONTROL_RIGHT_ARM true

yarp_interface::yarp_interface()
{
    if(createPolyDriver("torso", polyDriver_torso))
    {
        polyDriver_torso.view(encodersMotor_torso);
        polyDriver_torso.view(positionControl_torso);
        polyDriver_torso.view(controlMode_torso);
    }
    if(createPolyDriver("left_arm", polyDriver_left_arm))
    {
        polyDriver_left_arm.view(encodersMotor_left_arm);
        polyDriver_left_arm.view(positionControl_left_arm);
        polyDriver_left_arm.view(controlMode_left_arm);
    }
    if(createPolyDriver("right_arm", polyDriver_right_arm))
    {
        polyDriver_right_arm.view(encodersMotor_right_arm);
        polyDriver_right_arm.view(positionControl_right_arm);
        polyDriver_right_arm.view(controlMode_right_arm);
    }
    if(createPolyDriver("left_leg", polyDriver_left_leg))
    {
        polyDriver_left_leg.view(encodersMotor_left_leg);
        polyDriver_left_leg.view(positionControl_left_leg);
        polyDriver_left_leg.view(controlMode_left_leg);
    }
    if(createPolyDriver("right_leg", polyDriver_right_leg))
    {
        polyDriver_right_leg.view(encodersMotor_right_leg);
        polyDriver_right_leg.view(positionControl_right_leg);
        polyDriver_right_leg.view(controlMode_right_leg);
    }

    send_trj = false;

    port_send_trj.open("/homing/send_trj:i");
    torso_configuration_ref_port.open("/homing/torso/reference:o");
    left_arm_configuration_ref_port.open("homing/left_arm/reference:o");
    right_arm_configuration_ref_port.open("/homing/right_arm/reference:o");
    left_leg_configuration_ref_port.open("/homing/left_leg/reference:o");
    left_leg_configuration_ref_port.open("/homing/left_leg/reference:o");
}

void yarp_interface::checkInput()
{
    yarp::os::Bottle* bot_send_trj = port_send_trj.read(false);
    if(bot_send_trj != NULL)
        send_trj = bot_send_trj->get(0).asBool();
}

yarp_interface::~yarp_interface()
{

}

bool yarp_interface::createPolyDriver(const std::string& kinematic_chain, yarp::dev::PolyDriver& polyDriver)
{
    yarp::os::Property options;
    options.put("robot", "coman");
    options.put("device", "remote_controlboard");

    yarp::os::ConstString s;
    s = "/homing/coman/" + kinematic_chain;
    options.put("local", s.c_str());

    yarp::os::ConstString ss;
    ss = "/coman/" + kinematic_chain;
    options.put("remote", ss.c_str());

    polyDriver.open(options);
    if (!polyDriver.isValid()){
        std::cout<<"Device "<<kinematic_chain<<" not available."<<std::endl;
        return false;
    }
    else{
        std::cout<<"Device "<<kinematic_chain<<" available."<<std::endl;
        return true;
    }
}

void yarp_interface::fillBottleAndSend(const yarp::sig::Vector &q_d, const std::string &kinematic_chain)
{
    yarp::os::Bottle bot;
    for(unsigned int i = 0; i < q_d.size(); ++i)
        bot.addDouble(q_d[i]);

    if(kinematic_chain.compare("torso") == 0)
        torso_configuration_ref_port.write(bot);
    else if(kinematic_chain.compare("left_arm") == 0)
        left_arm_configuration_ref_port.write(bot);
    else if(kinematic_chain.compare("right_arm") == 0)
        right_arm_configuration_ref_port.write(bot);
    else if(kinematic_chain.compare("left_leg") == 0)
        left_leg_configuration_ref_port.write(bot);
    else if(kinematic_chain.compare("right_leg") == 0)
        right_leg_configuration_ref_port.write(bot);
}

void yarp_interface::moveKinematicChain(const yarp::sig::Vector &q_d, const std::string &kinematic_chain,
                                        const double max_speed)
{
    for(unsigned int i = 0; i < q_d.size(); ++i)
    {
        if(kinematic_chain.compare("torso") == 0)
        {
            controlMode_torso->setPositionMode(i);
            positionControl_torso->setRefSpeed(i, max_speed);
            positionControl_torso->positionMove(i, q_d[i]);
        }
#if USE_POSITION_CONTROL_LEFT_ARM
        else if(kinematic_chain.compare("left_arm") == 0)
        {
            controlMode_left_arm->setPositionMode(i);
            positionControl_left_arm->setRefSpeed(i, max_speed);
            positionControl_left_arm->positionMove(i, q_d[i]);
        }
#endif
#if USE_POSITION_CONTROL_RIGHT_ARM
        else if(kinematic_chain.compare("right_arm") == 0)
        {
            controlMode_right_arm->setPositionMode(i);
            positionControl_right_arm->setRefSpeed(i, max_speed);
            positionControl_right_arm->positionMove(i, q_d[i]);
        }
#endif
        else if(kinematic_chain.compare("left_leg") == 0)
        {
            controlMode_left_leg->setPositionMode(i);
            positionControl_left_leg->setRefSpeed(i, max_speed);
            positionControl_left_leg->positionMove(i, q_d[i]);
        }
        else if(kinematic_chain.compare("right_leg") == 0)
        {
            controlMode_right_leg->setPositionMode(i);
            positionControl_right_leg->setRefSpeed(i, max_speed);
            positionControl_right_leg->positionMove(i, q_d[i]);
        }
    }
}
