#ifndef _SIMPLE_HOMING_MODULE_HPP
#define _SIMPLE_HOMING_MODULE_HPP

#include <drc_shared/generic_module.hpp>
#include "simple_homing.h"
#include "simple_homing_constants.h"

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
        // insert torso param
        custom_params.push_back( new paramHelp::ParamProxyBasic<double>(   "torso", 
                                                                            PARAM_ID_TORSO, 
                                                                            PARAM_SIZE_TORSO, 
                                                                            paramHelp::PARAM_IN_OUT, 
                                                                            NULL, 
                                                                            "torso homing configuration" ) );
        // insert left_arm param
        custom_params.push_back( new paramHelp::ParamProxyBasic<double>(   "left_arm", 
                                                                            PARAM_ID_LEFT_ARM, 
                                                                            PARAM_SIZE_LEFT_ARM, 
                                                                            paramHelp::PARAM_IN_OUT, 
                                                                            NULL, 
                                                                            "left_arm homing configuration" ) );
        // insert right_arm param
        custom_params.push_back( new paramHelp::ParamProxyBasic<double>(   "right_arm", 
                                                                            PARAM_ID_RIGHT_ARM, 
                                                                            PARAM_SIZE_RIGHT_ARM, 
                                                                            paramHelp::PARAM_IN_OUT, 
                                                                            NULL, 
                                                                            "right_arm homing configuration" ) );
        // insert left_leg param
        custom_params.push_back( new paramHelp::ParamProxyBasic<double>(   "left_leg", 
                                                                            PARAM_ID_LEFT_LEG, 
                                                                            PARAM_SIZE_LEFT_LEG, 
                                                                            paramHelp::PARAM_IN_OUT, 
                                                                            NULL, 
                                                                            "left_leg homing configuration" ) );
        // insert right_leg param
        custom_params.push_back( new paramHelp::ParamProxyBasic<double>(   "right_leg", 
                                                                            PARAM_ID_RIGHT_LEG, 
                                                                            PARAM_SIZE_RIGHT_LEG, 
                                                                            paramHelp::PARAM_IN_OUT, 
                                                                            NULL, 
                                                                            "right_leg homing configuration" ) );
        return custom_params;
    }
};

#endif