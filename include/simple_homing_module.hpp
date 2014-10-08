#ifndef SIMPLE_HOMING_MODULE_HPP_
#define SIMPLE_HOMING_MODULE_HPP_

#include <drc_shared/generic_module.hpp>
#include "simple_homing.h"
#include "simple_homing_constants.h"

/**
 * @brief simple_homing module derived from generic_module
 * 
 * @author Luca Muratore (luca.muratore@iit.it)
 */
class simple_homing_module : public generic_module<simple_homing> {
public:
    
    /**
     * @brief constructor: do nothing but construct the superclass
     * 
     */
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
    
    /**
     * @brief overriden function to specify the custom params of the simple_homing for the param helper
     * 
     * @return a vector of the custom param of the simple_homing for the param helper
     */
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
        // insert max_vel param
        custom_params.push_back( new paramHelp::ParamProxyBasic<double>(    "max_vel", 
                                                                            PARAM_ID_MAX_VEL, 
                                                                            PARAM_SIZE_MAX_VEL, 
                                                                            paramHelp::PARAM_IN_OUT, 
                                                                            NULL, 
                                                                            "maximum velocity in [degree/second]" ) );
        return custom_params;
    }
};

#endif