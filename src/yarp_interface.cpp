#include "yarp_interface.h"
#include <iterator>
#include <algorithm>
#include <vector>
#include <iostream>

//#define USE_POSITION_CONTROL_LEFT_ARM true
//#define USE_POSITION_CONTROL_RIGHT_ARM true

yarp_interface::yarp_interface()
{
    isTorsoAvailable = false;
    isLeftArmAvailable = false;
    isRightArmAvailable = false;
    isLeftLegAvailable = false;
    isRightLegAvailable = false;

    if(createPolyDriver("torso", polyDriver_torso))
    {
        polyDriver_torso.view(encodersMotor_torso);
        polyDriver_torso.view(positionControl_torso);
        polyDriver_torso.view(controlMode_torso);
        torso_configuration_ref_port.open("/simple_homing/torso/reference:o");
        isTorsoAvailable = true;
    }
    if(createPolyDriver("left_arm", polyDriver_left_arm))
    {
        polyDriver_left_arm.view(encodersMotor_left_arm);
        polyDriver_left_arm.view(positionControl_left_arm);
        polyDriver_left_arm.view(controlMode_left_arm);
        left_arm_configuration_ref_port.open("/simple_homing/left_arm/reference:o");
        isLeftArmAvailable = true;
    }
    if(createPolyDriver("right_arm", polyDriver_right_arm))
    {
        polyDriver_right_arm.view(encodersMotor_right_arm);
        polyDriver_right_arm.view(positionControl_right_arm);
        polyDriver_right_arm.view(controlMode_right_arm);
        right_arm_configuration_ref_port.open("/simple_homing/right_arm/reference:o");
        isRightArmAvailable = true;
    }
    if(createPolyDriver("left_leg", polyDriver_left_leg))
    {
        polyDriver_left_leg.view(encodersMotor_left_leg);
        polyDriver_left_leg.view(positionControl_left_leg);
        polyDriver_left_leg.view(controlMode_left_leg);
        left_leg_configuration_ref_port.open("/simple_homing/left_leg/reference:o");
        isLeftLegAvailable = true;
    }
    if(createPolyDriver("right_leg", polyDriver_right_leg))
    {
        polyDriver_right_leg.view(encodersMotor_right_leg);
        polyDriver_right_leg.view(positionControl_right_leg);
        polyDriver_right_leg.view(controlMode_right_leg);
        right_leg_configuration_ref_port.open("/simple_homing/right_leg/reference:o");
        isRightLegAvailable = true;
    }

    send_trj = false;

    port_send_trj.open("/simple_homing/do_homing:i");
    status_port.open("/simple_homing/status:o");
}

void yarp_interface::checkInput()
{
    yarp::os::Bottle* bot_send_trj = port_send_trj.read(false);
    if(bot_send_trj != NULL)
        send_trj = (bool)bot_send_trj->get(0).asInt();

    if(send_trj)
    {
        setPositionControlModeKinematicChain("torso");
        setPositionControlModeKinematicChain("left_arm");
        setPositionControlModeKinematicChain("right_arm");
        setPositionControlModeKinematicChain("left_leg");
        setPositionControlModeKinematicChain("right_leg");
    }
}

yarp_interface::~yarp_interface()
{
    port_send_trj.close();
    right_arm_configuration_ref_port.close();
    left_arm_configuration_ref_port.close();
    torso_configuration_ref_port.close();
    right_leg_configuration_ref_port.close();
    left_leg_configuration_ref_port.close();
}

bool yarp_interface::createPolyDriver(const std::string& kinematic_chain, yarp::dev::PolyDriver& polyDriver)
{
    yarp::os::Property options;
    options.put("robot", "coman");
    options.put("device", "remote_controlboard");

    yarp::os::ConstString s;
    s = "/simple_homing/" + kinematic_chain;
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

void yarp_interface::fillStatusBottleAndSend(const std::string &status)
{
    yarp::os::Bottle bot;
    bot.addString(status.c_str());
    status_port.write(bot);
}

void yarp_interface::moveKinematicChain(const yarp::sig::Vector &q_d, const std::string &kinematic_chain)
{
    if(kinematic_chain.compare("torso") == 0)
        positionControl_torso->setPositions(q_d.data());
#if USE_POSITION_CONTROL_LEFT_ARM
    else if(kinematic_chain.compare("left_arm") == 0)
        positionControl_left_arm->setPositions(q_d.data());
#endif
#if USE_POSITION_CONTROL_RIGHT_ARM
    else if(kinematic_chain.compare("right_arm") == 0)
        positionControl_right_arm->setPositions(q_d.data());
#endif
    else if(kinematic_chain.compare("left_leg") == 0)
        positionControl_left_leg->setPositions(q_d.data());
    else if(kinematic_chain.compare("right_leg") == 0)
        positionControl_right_leg->setPositions(q_d.data());
}

void yarp_interface::setPositionControlModeKinematicChain(const std::string &kinematic_chain)
{
    int number_of_joints = 0;
    if(kinematic_chain.compare("torso") == 0)
    {
        encodersMotor_torso->getAxes(&number_of_joints);
        for(unsigned int i = 0; i < number_of_joints; ++i)
            controlMode_torso->setPositionMode(i);
    }
#if USE_POSITION_CONTROL_LEFT_ARM
    else if(kinematic_chain.compare("left_arm") == 0)
    {
        encodersMotor_left_arm->getAxes(&number_of_joints);
        for(unsigned int i = 0; i < number_of_joints; ++i)
            controlMode_left_arm->setPositionMode(i);
    }
#endif
#if USE_POSITION_CONTROL_RIGHT_ARM
    else if(kinematic_chain.compare("right_arm") == 0)
    {
        encodersMotor_right_arm->getAxes(&number_of_joints);
        for(unsigned int i = 0; i < number_of_joints; ++i)
            controlMode_right_arm->setPositionMode(i);
    }
#endif
    else if(kinematic_chain.compare("left_leg") == 0)
    {
        encodersMotor_left_leg->getAxes(&number_of_joints);
        for(unsigned int i = 0; i < number_of_joints; ++i)
            controlMode_left_leg->setPositionMode(i);
    }
    else if(kinematic_chain.compare("right_leg") == 0)
    {
        encodersMotor_right_leg->getAxes(&number_of_joints);
        for(unsigned int i = 0; i < number_of_joints; ++i)
            controlMode_right_leg->setPositionMode(i);
    }
}
