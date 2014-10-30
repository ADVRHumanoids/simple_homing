#include <yarp/os/all.h>
#include <stdlib.h>

#include "simple_homing.h"
#include "simple_homing_constants.h"

#define PRECISION 1 //[deg]

#define READY_STATUS "ready"
#define MOVING_STATUS "moving"
#define HOME_STATUS "home"

simple_homing::simple_homing(std::string module_prefix, 
                             yarp::os::ResourceFinder rf, 
                             std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    torso_chain_interface( "torso", module_prefix, get_robot_name() ),
    left_arm_chain_interface( "left_arm", module_prefix, get_robot_name() ),
    right_arm_chain_interface( "right_arm", module_prefix, get_robot_name() ),
    left_leg_chain_interface( "left_leg", module_prefix, get_robot_name() ),
    right_leg_chain_interface( "right_leg", module_prefix, get_robot_name() ),
    torso_homing( torso_chain_interface.getNumberOfJoints() ),
    left_arm_homing( left_arm_chain_interface.getNumberOfJoints() ),
    right_arm_homing( right_arm_chain_interface.getNumberOfJoints() ),
    left_leg_homing( left_leg_chain_interface.getNumberOfJoints() ),
    right_leg_homing( right_leg_chain_interface.getNumberOfJoints() ),
    torso( torso_chain_interface.getNumberOfJoints() ),
    left_arm( left_arm_chain_interface.getNumberOfJoints() ),
    right_arm( right_arm_chain_interface.getNumberOfJoints() ),
    left_leg( left_leg_chain_interface.getNumberOfJoints() ),
    right_leg( right_leg_chain_interface.getNumberOfJoints() ),
    command_interface( get_robot_name() + "/" + module_prefix ),
    status_interface( get_robot_name() + "/" + module_prefix ),
    generic_thread( module_prefix, rf, ph )
{
    // start the status chain_interface
    status_interface.start();
    // notify the ready status
    status_interface.setStatus( READY_STATUS );
}

bool simple_homing::set_ref_speed_to_all( double ref_speed )
{
            // torso chain
    return  set_ref_speed_to_chain( torso_chain_interface, ref_speed ) &&
            // left_arm chain
            set_ref_speed_to_chain( left_arm_chain_interface, ref_speed ) &&
            // right_arm chain
            set_ref_speed_to_chain( right_arm_chain_interface, ref_speed ) &&
            // left_leg chain
            set_ref_speed_to_chain( left_leg_chain_interface, ref_speed ) &&
            // right_leg chain
            set_ref_speed_to_chain( right_leg_chain_interface, ref_speed );
}

bool simple_homing::set_ref_speed_to_chain( walkman::drc::yarp_single_chain_interface& chain_interface, double ref_speed )
{
    // get joints number
    int num_joints = chain_interface.getNumberOfJoints();
    // set the speed references
    yarp::sig::Vector ref_speed_vec = yarp::sig::Vector( num_joints );
    bool set_success = true;
    for( int i = 0; i < num_joints && set_success; i++ ) {
        ref_speed_vec[i] = ref_speed;
        set_success = chain_interface.positionControl->setRefSpeed( i, ref_speed_vec[i] );
    }
    // success if the ref speed is setted in every joints of the chain 
    return set_success;
} 

bool simple_homing::custom_init()
{
    // real-time thread
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
    
    // param helper link param for all the chains and the max_vel param
    std::shared_ptr<paramHelp::ParamHelperServer> ph = get_param_helper();
    ph->linkParam( PARAM_ID_TORSO, torso_homing.data() );
    ph->linkParam( PARAM_ID_LEFT_ARM, left_arm_homing.data() );
    ph->linkParam( PARAM_ID_RIGHT_ARM, right_arm_homing.data() );
    ph->linkParam( PARAM_ID_LEFT_LEG, left_leg_homing.data() );
    ph->linkParam( PARAM_ID_RIGHT_LEG, right_leg_homing.data() );
    ph->linkParam( PARAM_ID_MAX_VEL, &max_vel );
    
    // initialize max q increment
    max_q_increment = max_vel * ( static_cast<double>( get_thread_period() ) / 1000 ); //[deg]
    
    // setting all the chain to position mode
    left_arm_chain_interface.setPositionMode();
    right_arm_chain_interface.setPositionMode();
    left_leg_chain_interface.setPositionMode();
    right_leg_chain_interface.setPositionMode();
    torso_chain_interface.setPositionMode();
    
    	
    // sense
    left_arm_chain_interface.encodersMotor->getEncoders( left_arm.data() );
    right_arm_chain_interface.encodersMotor->getEncoders( right_arm.data() );
    left_leg_chain_interface.encodersMotor->getEncoders( right_leg.data() );
    left_leg_chain_interface.encodersMotor->getEncoders( left_leg.data() );
    torso_chain_interface.encodersMotor->getEncoders( torso.data() );

    return true;
}

void simple_homing::control_and_move( walkman::drc::yarp_single_chain_interface& chain_interface, 
				      yarp::sig::Vector homing, 
				      yarp::sig::Vector& q )
{
    // set the speed ref for the chain -> TODO: take care of the success/failure of this function
    bool set_success = set_ref_speed_to_chain( chain_interface, max_vel );
    
    // control law
    controlLaw( homing, q );
    // position move to homing
    chain_interface.move( q );
}

void simple_homing::controlLaw(const yarp::sig::Vector& homing_vector, yarp::sig::Vector& q)
{
    unsigned int number_of_dofs = homing_vector.size();
    yarp::sig::Vector delta_q(number_of_dofs);
    for(unsigned int i = 0; i < number_of_dofs; ++i)
    {
        delta_q[i] = homing_vector[i] - q[i];
        if(fabs(delta_q[i]) > max_q_increment)
            delta_q[i] = (delta_q[i]/fabs(delta_q[i])) * max_q_increment;
        q[i] += delta_q[i];
    }
}


bool simple_homing::checkGoal(const yarp::sig::Vector& q, const yarp::sig::Vector& q_goal)
{
    for(unsigned int i = 0; i < q.size(); ++i){
        if( !( fabs( q[i] - q_goal[i] ) <= PRECISION) ) {
	    std::cout << "joint : " << i << " DELTA : " << fabs( q[i] - q_goal[i] ) << std::endl;
            return false;
	}
    }
    return true;
}

void simple_homing::run()
{   
    // if we have to go to homing position or we are moving -> control and move all the chains as specified in the homing vectors
    if( command_interface.getCommand() == "homing" || status_interface.state == MOVING_STATUS ) {
	// notify the moving status
	status_interface.setStatus( MOVING_STATUS );
	
	// check the goal
	if(checkGoal( left_arm, left_arm_homing ) &&
           checkGoal( right_arm, right_arm_homing ) &&
           checkGoal( left_leg, left_leg_homing ) &&
           checkGoal( right_leg, right_leg_homing ) &&
           checkGoal( torso, torso_homing ) ) 
	{
	    // notify the home status
	    status_interface.setStatus( HOME_STATUS );
	    // we are in homing position
	    std::cout << "Reached Home Position" << std::endl;
	}
	else 
	{
	    std::cout << "Moving" << std::endl;
	    // torso chain
	    control_and_move( torso_chain_interface, torso_homing, torso );
	    // left_arm chain
	    control_and_move( left_arm_chain_interface, left_arm_homing, left_arm );
	    // right_arm chain
	    control_and_move( right_arm_chain_interface, right_arm_homing, right_arm );
	    // left_leg chain
	    control_and_move( left_leg_chain_interface, left_leg_homing, left_leg );
	    // right_leg chain
	    control_and_move( right_leg_chain_interface, right_leg_homing, right_leg );
	}

    }
}

bool simple_homing::custom_pause()
{
    // set the ref speed to 0 for all the chains
     set_ref_speed_to_all( 0 );
}

bool simple_homing::custom_resume()
{
    // set the ref speed to max_vel for all the chains
    set_ref_speed_to_all( max_vel );
}


