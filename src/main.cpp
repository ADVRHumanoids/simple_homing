#include <yarp/os/all.h>
#include <drc_shared/generic_module.hpp>
#include "simple_homing.h"

#define MODULE_PERIOD 1000//[millisec]

int main(int argc, char* argv[])
{
    // yarp network declaration and check
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cerr <<"yarpserver not running - run yarpserver"<< std::endl;
        exit(EXIT_FAILURE);
    }
    // yarp network initialization
    yarp.init();

    //create rf
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile( "initial_config.ini" ); 
    rf.setDefaultContext( "simple_homing" );  
    rf.configure(argc, argv);
    // create my module
    generic_module<simple_homing> simple_homing_module = generic_module<simple_homing>( argc, argv, "simple_homing", MODULE_PERIOD, rf );
    
        
    // yarp network deinitialization
    yarp.fini();
    //run the module
    return simple_homing_module.runModule();
}
