#ifndef _YARP_INTERFACE_H_
#define _YARP_INTERFACE_H_

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <vector>

#include <drc_shared/yarp_single_chain_interface.h>
#include <drc_shared/yarp_status_interface.h>
#include <drc_shared/yarp_command_interface.hpp>

class yarp_interface
{
public:
    yarp_interface();
    ~yarp_interface();
    walkman::drc::yarp_single_chain_interface left_leg,left_arm,right_leg,right_arm,torso;
    walkman::drc::yarp_status_interface status;
    walkman::drc::yarp_command_interface commands;
    bool sendTrj() {return send_trj;}
    void checkInput();
    void stop() {send_trj = false;}
    void fillBottleAndSend(const yarp::sig::Vector& q_d, const std::string& kinematic_chain);
    void setPositionControlModeKinematicChain(walkman::drc::yarp_single_chain_interface& chain);
    void setImpedanceControlModeKinematicChain(walkman::drc::yarp_single_chain_interface& chain);

private:
    bool send_trj;
    bool set_position_mode;
//     yarp::os::BufferedPort<yarp::os::Bottle> port_send_trj;
    yarp::os::Port right_arm_configuration_ref_port;
    yarp::os::Port left_arm_configuration_ref_port;
    yarp::os::Port torso_configuration_ref_port;
    yarp::os::Port right_leg_configuration_ref_port;
    yarp::os::Port left_leg_configuration_ref_port;

};

#endif
