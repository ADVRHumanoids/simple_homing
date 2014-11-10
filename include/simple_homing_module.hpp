#ifndef SIMPLE_HOMING_MODULE_HPP_
#define SIMPLE_HOMING_MODULE_HPP_

#include <GYM/generic_module.hpp>
#include <idynutils/comanutils.h>

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
	// insert control_mode param
        custom_params.push_back( new paramHelp::ParamProxyBasic<std::string>(   "control_mode", 
										PARAM_ID_CONTROL_MODE, 
										PARAM_SIZE_CONTROL_MODE, 
										paramHelp::PARAM_IN_OUT, 
										NULL, 
										"control mode configuration" ) );
        return custom_params;
    }
    
    
    virtual void custom_ph_param_value_changed_callback() 
    {	
	// get param helper
	std::shared_ptr< paramHelp::ParamHelperServer > ph = get_param_helper();
	// register all the callbacks
	ph->registerParamValueChangedCallback( PARAM_ID_TORSO, this );
	ph->registerParamValueChangedCallback( PARAM_ID_LEFT_ARM, this );
	ph->registerParamValueChangedCallback( PARAM_ID_RIGHT_ARM, this );
	ph->registerParamValueChangedCallback( PARAM_ID_LEFT_LEG, this );
	ph->registerParamValueChangedCallback( PARAM_ID_RIGHT_LEG, this );
	ph->registerParamValueChangedCallback( PARAM_ID_MAX_VEL, this );
	ph->registerParamValueChangedCallback( PARAM_ID_CONTROL_MODE, this );
    }
    
    virtual void custom_parameterUpdated(const paramHelp::ParamProxyInterface *pd)
    {
	simple_homing* thread = get_thread();
	if( thread ) {
	    if( pd->id == PARAM_ID_TORSO || 
		pd->id == PARAM_ID_LEFT_ARM ||
		pd->id == PARAM_ID_RIGHT_ARM ||
		pd->id == PARAM_ID_LEFT_LEG ||
		pd->id == PARAM_ID_RIGHT_LEG
	    ) {
		    thread->update_q_homing();
	    }
	    else if( pd->id == PARAM_ID_CONTROL_MODE ) {
		thread->update_control_mode();
	    }
	}
    }
};

#endif