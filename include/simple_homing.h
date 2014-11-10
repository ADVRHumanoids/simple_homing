#ifndef SIMPLE_HOMING_H_
#define SIMPLE_HOMING_H_

#include <yarp/sig/Vector.h>

#include <idynutils/yarp_single_chain_interface.h>
#include <GYM/yarp_command_interface.hpp>
#include <GYM/yarp_status_interface.h>
#include <GYM/generic_thread.hpp>
#include <idynutils/comanutils.h>

/**
 * @brief simple homing control thread: move all the joints of the robot to the desired homing position
 * 
 **/
class simple_homing: public generic_thread
{
private:   
    /**
     * @brief whole-body control facilities
     * 
     */
    ComanUtils coman;
    
    /**
     * @brief torso homing vector
     * 
     */
    yarp::sig::Vector torso_homing;
    
    /**
     * @brief left arm homing vector
     * 
     */
    yarp::sig::Vector left_arm_homing;
    
    /**
     * @brief right arm homing vector
     * 
     */
    yarp::sig::Vector right_arm_homing;
    
    /**
     * @brief left leg homing vector
     * 
     */
    yarp::sig::Vector left_leg_homing;
    
    /**
     * @brief right leg homing vector
     * 
     */
    yarp::sig::Vector right_leg_homing;
    
    /**
     * @brief whole-body reference position
     * 
     */
    yarp::sig::Vector q_homing;
    
    /**
     * @brief whole-body position
     * 
     */
    yarp::sig::Vector q;
    
    /**
     * @brief max speed ref in [radians/second]
     * 
     */
    double max_vel;
    
    /**
     * @brief maximum q increment in [radians]
     * 
     */
    double max_q_increment;
    
    /**
     * @brief simple_homing command interface
     * 
     */
    walkman::drc::yarp_command_interface command_interface;
    
    /**
     * @brief simple_homing status interface
     * 
     */
    walkman::drc::yarp_status_interface status_interface;

    void control_and_move();
    
    bool checkGoal();

    void controlLaw();
    
    std::string actual_control_mode;
    
public:
    
    /**
     * @brief constructor
     * 
     * @param module_prefix the prefix of the module
     * @param rf resource finderce
     * @param ph param helper
     */
    simple_homing( std::string module_prefix, 
                   yarp::os::ResourceFinder rf, 
                   std::shared_ptr<paramHelp::ParamHelperServer> ph );
    
    /**
     * @brief TODO
     * 
     * @return void
     */
    void update_q_homing();
    
    /**
     * @brief simple_homing control thread initialization
     * 
     * @return true on succes, false otherwise
     */
    virtual bool custom_init();
    
    /**
     * @brief simple_homing control thread main loop
     * 
     */
    virtual void run();
    
    /**
     * @brief handler called before the control thread is suspended with a "pause" command
     * 
     * @return true on success, false otherwise
     */
    virtual bool custom_pause();
    
    /**
     * @brief handler called before the control thread is resumed with a "resume" command
     * 
     * @return true on success, false otherwise
     */
    virtual bool custom_resume();
    
    
    void update_control_mode();
    
};

#endif
