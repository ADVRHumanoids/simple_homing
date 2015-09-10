#include <yarp/os/all.h>
#include <stdlib.h>

#include "simple_homing.h"
#include "simple_homing_constants.h"

#define PRECISION 0.02 //[radians]

#define READY_STATUS "ready"
#define MOVING_STATUS "moving"
#define HOME_STATUS "home"

#define RAD2DEG    (180.0/M_PI)
#define DEG2RAD    (M_PI/180.0)

#define RECORD_TRJ true

simple_homing::simple_homing(std::string module_prefix, 
                             yarp::os::ResourceFinder rf, 
                             std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    torso_homing( robot.torso.getNumberOfJoints(), 0.0 ),
    left_arm_homing( robot.left_arm.getNumberOfJoints(), 0.0 ),
    right_arm_homing( robot.right_arm.getNumberOfJoints(), 0.0 ),
    left_leg_homing( robot.left_leg.getNumberOfJoints(), 0.0 ),
    right_leg_homing( robot.right_leg.getNumberOfJoints(), 0.0 ),
    head_homing(robot.head.getNumberOfJoints(), 0.0),
    q_homing( robot.getNumberOfJoints(), 0.0 ),
    max_vel( 0 ),
    q( robot.getNumberOfJoints() ),
    command_interface( module_prefix ),
    status_interface( module_prefix ),
    control_thread( module_prefix, rf, ph ),
    all_q()
{
    // start the status chain_interface
    status_interface.start();
    // notify the ready status
    status_interface.setStatus(READY_STATUS);
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
    ph->linkParam( PARAM_ID_HEAD, head_homing.data() );
    ph->linkParam( PARAM_ID_MAX_VEL, &max_vel );
    
    // initialize q homing
    update_q_homing();
    // initialize max q increment
    max_q_increment = max_vel * ( static_cast<double>( get_thread_period() ) / 1000.0 ); //[radians]
    	
    // sense
    q = robot.sensePosition();

    // set all boards to position control mode
    if(robot.isInPositionDirectMode())
        std::cout<<"Robot is in Position Direct Mode!"<<std::endl;
    else{
        if(!robot.setPositionDirectMode())
            std::cout << "Error setting the robot in Position Direct Mode" << std::endl;
        else
            std::cout<<"Robot succesfully set in Position Direct Mode!"<<std::endl;
    }

    return true;
}

void simple_homing::control_and_move()
{
    // control law
    controlLaw();
    // position move to homing
    robot.move( q );
}

void simple_homing::controlLaw()
{
    unsigned int number_of_dofs = q_homing.size();
    yarp::sig::Vector delta_q(number_of_dofs);
    for(unsigned int i = 0; i < number_of_dofs; ++i)
    {
        //std::cout << "Joint # " << i << " -> q = " << q[i] <<  " -> q_homing = " << q_homing[i] << std::endl;
        delta_q[i] = q_homing[i] - q[i];
        if ( fabs( delta_q[i] ) > max_q_increment )
            delta_q[i] = ( delta_q[i]/fabs(delta_q[i] ) ) * max_q_increment;
        q[i] += delta_q[i];
        //std::cout << "NEW q = " << q[i] << std::endl;
    }
}


bool simple_homing::checkGoal()
{
    for(unsigned int i = 0; i < q.size(); ++i){
        if( !( fabs( q[i] - q_homing[i] ) <= PRECISION) ) {
        //std::cout << "Joint " << i << " not in homing" <<  std::endl;
            return false;}
    }
    return true;
}

void simple_homing::recordTrj()
{
    std::ofstream FILE;
    std::ofstream FILE_left_leg;
    std::ofstream FILE_left_arm;
    std::ofstream FILE_right_leg;
    std::ofstream FILE_right_arm;
    std::ofstream FILE_torso;
    std::ofstream FILE_head;

    FILE.open("all_joints_trj.m");
    FILE<<"q = ["<<std::endl;
    FILE_left_leg.open("left_leg_joints_trj.m");
    FILE_left_leg<<"q_left_leg = ["<<std::endl;
    FILE_right_leg.open("right_leg_joints_trj.m");
    FILE_right_leg<<"q_right_leg = ["<<std::endl;
    FILE_left_arm.open("left_arm_joints_trj.m");
    FILE_left_arm<<"q_left_arm = ["<<std::endl;
    FILE_right_arm.open("right_arm_joints_trj.m");
    FILE_right_arm<<"q_right_arm = ["<<std::endl;
    FILE_torso.open("torso_joints_trj.m");
    FILE_torso<<"q_torso = ["<<std::endl;
    FILE_head.open("head_joints_trj.m");
    FILE_head<<"q_head = ["<<std::endl;

    yarp::sig::Vector left_arm(robot.left_arm.getNumberOfJoints(),0.0);
    yarp::sig::Vector right_arm(robot.right_arm.getNumberOfJoints(),0.0);
    yarp::sig::Vector left_leg(robot.left_leg.getNumberOfJoints(),0.0);
    yarp::sig::Vector right_leg(robot.right_leg.getNumberOfJoints(),0.0);
    yarp::sig::Vector torso(robot.torso.getNumberOfJoints(),0.0);
    yarp::sig::Vector head(robot.head.getNumberOfJoints(),0.0);
    for(unsigned int i = 0; i < all_q.size(); ++i){
        FILE<<all_q[i].toString()<<std::endl;

        if(head.size() > 0){
            robot.fromIdynToRobot(all_q[i],right_arm,left_arm,torso,right_leg,left_leg,head);
            FILE_head<<head.toString()<<std::endl;}
        else
            robot.fromIdynToRobot(all_q[i],right_arm,left_arm,torso,right_leg,left_leg);
        FILE_left_arm<<left_arm.toString()<<std::endl;
        FILE_right_arm<<right_arm.toString()<<std::endl;
        FILE_left_leg<<left_leg.toString()<<std::endl;
        FILE_right_leg<<right_leg.toString()<<std::endl;
        FILE_torso<<torso.toString()<<std::endl;
    }

    FILE<<"];"<<std::endl;
    FILE.close();
    FILE_left_arm<<"];"<<std::endl;
    FILE_left_arm.close();
    FILE_right_arm<<"];"<<std::endl;
    FILE_right_arm.close();
    FILE_left_leg<<"];"<<std::endl;
    FILE_left_leg.close();
    FILE_right_leg<<"];"<<std::endl;
    FILE_right_leg.close();
    FILE_head<<"];"<<std::endl;
    FILE_head.close();
    FILE_torso<<"];"<<std::endl;
    FILE_torso.close();

}

void simple_homing::run()
{   
    // if we have to go to homing position or we are moving -> control and move all the chains as specified in the homing vectors
    if( command_interface.getCommand() == "homing") {
        // sense position
        q = robot.sensePosition();

        // notify the moving status
        status_interface.setStatus(MOVING_STATUS);
    }
    
    if( status_interface.state == MOVING_STATUS ) {
        // check the goal
        if( checkGoal() )
        {
            // notify the home status
            status_interface.setStatus( HOME_STATUS );
            // we are in homing position
            std::cout << "Reached Home Position" << std::endl;
            if(RECORD_TRJ)
                recordTrj();
        }
        else
        {
            all_q.push_back(q);
            control_and_move();
        }
    }
}

void simple_homing::update_q_homing()
{
    std:: cout << "Updating q_homing ..." << std::endl;
    robot.fromRobotToIdyn( 
                    right_arm_homing, 
                    left_arm_homing, 
                    torso_homing, 
                    right_leg_homing, 
                    left_leg_homing,
                    head_homing,
                    q_homing);
}


bool simple_homing::custom_pause()
{

}

bool simple_homing::custom_resume()
{

}


