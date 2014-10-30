#ifndef SIMPLE_HOMING_H_
#define SIMPLE_HOMING_H_

#include <yarp/sig/Vector.h>

#include <drc_shared/yarp_single_chain_interface.h>
#include <drc_shared/yarp_command_interface.hpp>
#include <drc_shared/yarp_status_interface.h>
#include <drc_shared/generic_thread.hpp>
#include <drc_shared/comanutils.h>

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
    
    /**
     * @brief set the speed ref for all the chains of the robot 
     * 
     * @param ref_speed speed ref 
     * @return true on success, false otherwise
     */
    bool set_ref_speed_to_all( double ref_speed );
    
    /**
     * @brief set the speed ref for all the chain specified in chain_interface param
     * 
     * @param chain_interface chain interface to set the speed ref
     * @param ref_speed speed ref 
     * @return true on success, false otherwise
     */
    bool set_ref_speed_to_chain( walkman::drc::yarp_single_chain_interface& chain_interface, double ref_speed );

    
    
    void control_and_move();
    
    bool checkGoal();

    void controlLaw();
    
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
    
};

#endif
