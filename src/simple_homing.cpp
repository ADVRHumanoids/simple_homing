#include <yarp/os/all.h>
#include <stdlib.h>
#include "simple_homing.h"

#define DEFAULT_MAX_VEL 50.0 //[deg/sec]

simple_homing::simple_homing(std::string module_prefix, 
                             yarp::os::ResourceFinder rf, 
                             std::shared_ptr< paramHelp::ParamHelperServer > ph) :  parser( rf ),
                                                                                    torso_chain_interface( "torso", module_prefix ),
                                                                                    left_arm_chain_interface( "left_arm", module_prefix ),
                                                                                    right_arm_chain_interface( "right_arm", module_prefix ),
                                                                                    left_leg_chain_interface( "left_leg", module_prefix ),
                                                                                    right_leg_chain_interface( "right_leg", module_prefix ),
                                                                                    torso_homing( torso_chain_interface.getNumberOfJoints() ),
                                                                                    left_arm_homing( left_arm_chain_interface.getNumberOfJoints() ),
                                                                                    right_arm_homing( right_arm_chain_interface.getNumberOfJoints() ),
                                                                                    left_leg_homing( left_leg_chain_interface.getNumberOfJoints() ),
                                                                                    right_leg_homing( left_leg_chain_interface.getNumberOfJoints() ),
                                                                                    max_vel( DEFAULT_MAX_VEL ),
                                                                                    set_init_config( false ),
                                                                                    generic_thread(module_prefix, rf, ph)
{
}



bool simple_homing::custom_init()
{
    // real-time thread
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
    
    // initialize homing vectors
    if( !parser.getConfiguration("torso", torso_homing) )
        std::cout << "Error loading torso homing. All zeros will be used." << std::endl;
    if( !parser.getConfiguration("left_arm", left_arm_homing) )
        std::cout << "Error loading left_arm homing. All zeros will be used." << std::endl;
    if( !parser.getConfiguration("right_arm", right_arm_homing) )
        std::cout<<"Error loading right_arm homing. All zeros will be used."<<std::endl;
    if( !parser.getConfiguration("left_leg", left_leg_homing) )
        std::cout<<"Error loading left_leg homing. All zeros will be used."<<std::endl;
    if( !parser.getConfiguration("right_leg", right_leg_homing) )
        std::cout<<"Error loading right_leg homing. All zeros will be used."<<std::endl;
    if( !parser.getMaxVelocity(max_vel) )
        std::cout<<"Error loading max vel. "<< max_vel <<" [deg/sec] will be used."<<std::endl;
    
    std::cout << torso_homing.toString() << std::endl;
    
    // param helper link param
    std::shared_ptr<paramHelp::ParamHelperServer> ph = get_param_helper();
    ph->linkParam( PARAM_ID_TORSO, torso_homing.data() );

    return true;
}

void simple_homing::controlAndMove( walkman::drc::yarp_single_chain_interface& chain_interface, yarp::sig::Vector homing )
{
    // get joints number
    int num_joints = chain_interface.getNumberOfJoints();
    // set the speed references
    yarp::sig::Vector ref_speed_vec = yarp::sig::Vector( num_joints );
    for( int i = 0; i < num_joints; i++ ) {
        ref_speed_vec[i] = max_vel;
        chain_interface.positionControl->setRefSpeed( i, ref_speed_vec[i] );
    }
    // position move to homing
    chain_interface.positionControl->positionMove( homing.data() );
       
}

void simple_homing::run()
{   
    if( !set_init_config ) {
        controlAndMove( torso_chain_interface, torso_homing );
        controlAndMove( left_arm_chain_interface, left_arm_homing );
        controlAndMove( right_arm_chain_interface, right_arm_homing );
        controlAndMove( left_leg_chain_interface, left_leg_homing );
        controlAndMove( right_leg_chain_interface, right_leg_homing );
        set_init_config = true;
        std::cout << "Reached Home Position" << std::endl;
    }
}

std::string simple_homing::computeStatus()
{
    return "status TODO";
}


