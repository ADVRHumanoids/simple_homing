#ifndef _HOMING_CMD_PORT_HPP_
#define _HOMING_CMD_PORT_HPP_

#include <yarp/os/all.h>

class homing_cmd_port : public yarp::os::BufferedPort<yarp::os::Bottle> {
private:
    bool go_homing;
public:
    homing_cmd_port( bool go_homing) : go_homing( go_homing ),
                                       yarp::os::BufferedPort<yarp::os::Bottle>()
    {
    }
    
    virtual void onRead( yarp::os::Bottle& b ) 
    {
        std::string cmd_received = b.toString();
        std::cout  << "Received cmd : " << cmd_received << std::endl;
        if ( cmd_received  == "homing" ) {
            go_homing = true;
             std::cout  << "Going to home position ... " << std::endl;
        }
    }
    
    bool get_go_homing() 
    {
        return go_homing;
    }
    
    void set_go_homing( bool go_homing )
    {
        this->go_homing = go_homing;
    }
};

#endif