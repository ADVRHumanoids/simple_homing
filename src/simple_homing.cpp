#include <yarp/os/all.h>
#include <stdlib.h>

#include "simple_homing.h"
#include "simple_homing_constants.h"

#define PRECISION 0.02 //[radians]

#define READY_STATUS "ready"
#define MOVING_STATUS "moving"
#define HOME_STATUS "home"

simple_homing::simple_homing(std::string module_prefix, 
                             yarp::os::ResourceFinder rf, 
                             std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    generic_thread( module_prefix, rf, ph ),
    coman( module_prefix, VOCAB_CM_POSITION_DIRECT ),
    torso_homing( coman.torso.getNumberOfJoints() ),
    left_arm_homing( coman.left_arm.getNumberOfJoints() ),
    right_arm_homing( coman.right_arm.getNumberOfJoints() ),
    left_leg_homing( coman.left_leg.getNumberOfJoints() ),
    right_leg_homing( coman.right_leg.getNumberOfJoints() ),
    q_homing( coman.getNumberOfJoints() ),
    q( coman.getNumberOfJoints() ),
    command_interface( module_prefix ),
    status_interface( module_prefix )
{
    // start the status chain_interface
    status_interface.start();
    // notify the ready status
    status_interface.setStatus( READY_STATUS );
}

bool simple_homing::set_ref_speed_to_all( double ref_speed )
{
            // torso chain
    return  set_ref_speed_to_chain( coman.torso, ref_speed ) &&
            // left_arm chain
            set_ref_speed_to_chain( coman.left_arm, ref_speed ) &&
            // right_arm chain
            set_ref_speed_to_chain( coman.right_arm, ref_speed ) &&
            // left_leg chain
            set_ref_speed_to_chain( coman.left_leg, ref_speed ) &&
            // right_leg chain
            set_ref_speed_to_chain( coman.right_leg, ref_speed );
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
    
    // initialize q homing
    update_q_homing();
    // initialize max q increment
    max_q_increment = max_vel * ( static_cast<double>( get_thread_period() ) / 1000.0 ); //[radians]
    	
    // sense
    q = coman.sensePosition();

    return true;
}

void simple_homing::control_and_move()
{
    // set the speed ref for the chain -> TODO: take care of the success/failure of this function
    bool set_success = set_ref_speed_to_all( max_vel );
    // control law
    controlLaw( );
    // position move to homing
    coman.move( q );
}

void simple_homing::controlLaw()
{
    unsigned int number_of_dofs = q_homing.size();
    yarp::sig::Vector delta_q(number_of_dofs);
    for(unsigned int i = 0; i < number_of_dofs; ++i)
    {
        delta_q[i] = q_homing[i] - q[i];
        if ( fabs( delta_q[i] ) > max_q_increment )
            delta_q[i] = ( delta_q[i]/fabs(delta_q[i] ) ) * max_q_increment;
        q[i] += delta_q[i];
    }
}


bool simple_homing::checkGoal()
{
    for(unsigned int i = 0; i < q.size(); ++i){
        if( !( fabs( q[i] - q_homing[i] ) <= PRECISION) ) {
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
	if( checkGoal() )
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
	    control_and_move();
	}

    }
}

void simple_homing::update_q_homing()
{
    std:: cout << "Updating q_homing ..." << std::endl;
    coman.fromRobotToIdyn( right_arm_homing, 
			   left_arm_homing, 
			   torso_homing, 
			   right_leg_homing, 
			   left_leg_homing,
			   q_homing);
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


