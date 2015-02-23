#include <yarp/os/all.h>
#include <GYM/generic_module.hpp>
#include <cstdlib>

#include "simple_homing_module.hpp"

// define
#define MODULE_PERIOD 1000 //[millisec]

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

    // create rf
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    // set initial_config.ini as default
    // to specify another config file, run the simple_homing with this arg: --from your_config_file.ini 
    rf.setDefaultConfigFile( "initial_config.ini" ); 
    rf.setDefaultContext( "simple_homing" );  
    rf.configure(argc, argv);
    // create my module
    simple_homing_module sp_mod = simple_homing_module( argc, argv, "simple_homing", MODULE_PERIOD, rf );

    
    // run the module
    sp_mod.runModule( rf );
            
    // yarp network deinitialization
    yarp.fini();
    exit(EXIT_SUCCESS);
}
