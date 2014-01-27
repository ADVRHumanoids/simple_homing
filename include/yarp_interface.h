#ifndef _YARP_INTERFACE_H_
#define _YARP_INTERFACE_H_

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <vector>


class yarp_interface
{
public:
    yarp_interface();
    ~yarp_interface();

    bool sendTrj() {return send_trj;}

    static const char * kinematic_chains;
    yarp::dev::IEncodersTimed *encodersMotor_torso;
    yarp::dev::IEncodersTimed *encodersMotor_left_arm;
    yarp::dev::IEncodersTimed *encodersMotor_right_arm;
    yarp::dev::IEncodersTimed *encodersMotor_left_leg;
    yarp::dev::IEncodersTimed *encodersMotor_right_leg;
    yarp::dev::IPositionControl2 *positionControl_torso;
    yarp::dev::IPositionControl2 *positionControl_left_arm;
    yarp::dev::IPositionControl2 *positionControl_right_arm;
    yarp::dev::IPositionControl2 *positionControl_left_leg;
    yarp::dev::IPositionControl2 *positionControl_right_leg;
    yarp::dev::IControlMode *controlMode_torso;
    yarp::dev::IControlMode *controlMode_left_arm;
    yarp::dev::IControlMode *controlMode_right_arm;
    yarp::dev::IControlMode *controlMode_left_leg;
    yarp::dev::IControlMode *controlMode_right_leg;
    bool isTorsoAvailable;
    bool isLeftArmAvailable;
    bool isRightArmAvailable;
    bool isLeftLegAvailable;
    bool isRightLegAvailable;

    void checkInput();
    void stop() {send_trj = false;}
    void fillBottleAndSend(const yarp::sig::Vector& q_d, const std::string& kinematic_chain);
    void fillStatusBottleAndSend(const std::string& status);
    void setPositionControlModeKinematicChain(const std::string& kinematic_chain, const double max_speed);
    void moveKinematicChain(const yarp::sig::Vector& q_d, const std::string& kinematic_chain);
    void setMaxVel(const double max_vel){ _max_vel = max_vel;}

private:
    yarp::dev::PolyDriver polyDriver_torso;
    yarp::dev::PolyDriver polyDriver_left_arm;
    yarp::dev::PolyDriver polyDriver_right_arm;
    yarp::dev::PolyDriver polyDriver_left_leg;
    yarp::dev::PolyDriver polyDriver_right_leg;
    bool send_trj;
    yarp::os::BufferedPort<yarp::os::Bottle> port_send_trj;
    yarp::os::Port right_arm_configuration_ref_port;
    yarp::os::Port left_arm_configuration_ref_port;
    yarp::os::Port torso_configuration_ref_port;
    yarp::os::Port right_leg_configuration_ref_port;
    yarp::os::Port left_leg_configuration_ref_port;
    yarp::os::Port status_port;
    double _max_vel;

    bool createPolyDriver(const std::string &kinematic_chain, yarp::dev::PolyDriver &polyDriver);
};

#endif
