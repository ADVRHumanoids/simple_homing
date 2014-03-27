#include "yarp_interface.h"
#include <drc_shared/yarp_status_interface.h>
#include <iterator>
#include <algorithm>
#include <vector>
#include <iostream>

//#define USE_POSITION_CONTROL_LEFT_ARM true
//#define USE_POSITION_CONTROL_RIGHT_ARM true

using namespace walkman::drc;

yarp_interface::yarp_interface():left_leg("left_leg","simple_homing"),left_arm("left_arm","simple_homing"),
right_leg("right_leg","simple_homing"),right_arm("right_arm","simple_homing"),torso("torso","simple_homing"),
status("simple_homing"),commands("simple_homing")
{
    torso_configuration_ref_port.open("/simple_homing/torso/reference:o");
    left_arm_configuration_ref_port.open("/simple_homing/left_arm/reference:o");
    left_leg_configuration_ref_port.open("/simple_homing/left_leg/reference:o");
    right_arm_configuration_ref_port.open("/simple_homing/right_arm/reference:o");
    right_leg_configuration_ref_port.open("/simple_homing/right_leg/reference:o");
    
    send_trj = false;
    set_position_mode = false;

    //port_send_trj.open("/simple_homing/do_homing:i");
    status.start();
//     status_port.open("/simple_homing/status:o");
}

void yarp_interface::checkInput()
{
   int command;
   if (commands.getCommand(command))
   {
       send_trj=(command>0);
       set_position_mode=false;
   }
//     yarp::os::Bottle* bot_send_trj = port_send_trj.read(false);
//     if(bot_send_trj != NULL){
//         send_trj = (bool)bot_send_trj->get(0).asInt();
//         set_position_mode = false;}
    
    if(send_trj && !set_position_mode)
    {
        setPositionControlModeKinematicChain(torso);
        setPositionControlModeKinematicChain(left_arm);
        setPositionControlModeKinematicChain(right_arm);
        setPositionControlModeKinematicChain(left_leg);
        setPositionControlModeKinematicChain(right_leg);
        set_position_mode = true;
    }
}

yarp_interface::~yarp_interface()
{
    //port_send_trj.close();
    right_arm_configuration_ref_port.close();
    left_arm_configuration_ref_port.close();
    torso_configuration_ref_port.close();
    right_leg_configuration_ref_port.close();
    left_leg_configuration_ref_port.close();
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


void yarp_interface::setPositionControlModeKinematicChain(yarp_single_chain_interface& chain)
{
     for(int i = 0; i < chain.getNumberOfJoints(); i++)
         chain.controlMode->setPositionMode(i);
}
