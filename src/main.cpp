#include <yarp/os/all.h>
#include <cstdlib>

#include <drc_shared/generic_module.hpp>
#include "simple_homing.h"

#define MODULE_PERIOD 1000//[millisec]


class simple_homing_module : public generic_module<simple_homing> {
public:
    simple_homing_module(   int argc, 
                            char* argv[],
                            std::string module_prefix, 
                            int module_period, 
                            yarp::os::ResourceFinder rf ) : generic_module<simple_homing>(  argc, 
                                                                                            argv, 
                                                                                            module_prefix, 
                                                                                            module_period,
                                                                                            rf )
    {
    }
    
    virtual std::vector< paramHelp::ParamProxyInterface* > custom_get_ph_parameters() 
    {
        std::vector<paramHelp::ParamProxyInterface *> custom_params;
        // insert other param
        custom_params.push_back( new paramHelp::ParamProxyBasic<double>(   "torso", 
                                                                            PARAM_ID_TORSO, 
                                                                            PARAM_SIZE_TORSO, 
                                                                            paramHelp::PARAM_IN_OUT, 
                                                                            NULL, 
                                                                            "torso homing configuration" ) );
        return custom_params;
    }
};


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
//     generic_module<simple_homing> sp_mod = generic_module<simple_homing>( argc, argv, "simple_homing", MODULE_PERIOD, rf );
    simple_homing_module sp_mod = simple_homing_module( argc, argv, "simple_homing", MODULE_PERIOD, rf );
        
    // yarp network deinitialization
    yarp.fini();
    //run the module
    sp_mod.runModule();
    
    exit(EXIT_SUCCESS);
}
