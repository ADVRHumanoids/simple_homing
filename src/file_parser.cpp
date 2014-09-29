#include "file_parser.h"
#include <iostream>

file_parser::file_parser():
    _file_name("initial_config.ini"),
    _file_context("simple_homing"),
    _parameter_help("help"),  //Without space does not work!
    _parameter_simulation("simulation"),
    _config_file()
{
    _config_file.setVerbose(true);
    _config_file.setDefaultConfigFile(_file_name.c_str());
    _config_file.setDefaultContext(_file_context.c_str());
    _config_file.configure("",1,NULL);
}

file_parser::file_parser(int argc, char *argv[]):
    _file_name("initial_config.ini"),
    _file_context("simple_homing"),
    _parameter_help("help"),  //Without space does not work!
    _parameter_simulation("simulation"),
    _config_file()
{
    yarp::os::Property _command_line;
    _command_line.fromCommand(argc,argv);

    if(!set_input_parameter(_command_line))
        exit(1);

    _config_file.setVerbose(true);
    _config_file.setDefaultConfigFile(_file_name.c_str());
    _config_file.setDefaultContext(_file_context.c_str());
    _config_file.configure("",argc,argv);
}

file_parser::file_parser(yarp::os::ResourceFinder rf) : _config_file( rf )
{
}


file_parser::~file_parser()
{

}

bool file_parser::set_input_parameter(yarp::os::Property &_command_line)
{

    if(_command_line.check(_parameter_help.c_str()))
    {
       help();
       return false;
    }
    if(_command_line.check(_parameter_simulation.c_str()))
    {
        _file_name = "initial_config_simulation.ini";
    }
    return true;
}


bool file_parser::getConfiguration(const std::string &chain_name, yarp::sig::Vector& home_configuration)
{
    yarp::os::Value configuration = _config_file.findGroup("HOMING").find(chain_name);

    if(!configuration.isNull())
    {
        std::cout<<"Initial configuration for "<<chain_name<<" is: [ ";
        for(unsigned int i = 0; i < home_configuration.size(); ++i){
            std::cout<<configuration.asList()->get(i).asDouble()<<" ";
            home_configuration[i] = configuration.asList()->get(i).asDouble();
        }
        std::cout<<" ] [deg]\n";

        return true;
    }
    else{
        std::cout<<"ERROR Reading home configuration!"<<std::endl;
        return false;}
}

bool file_parser::getMaxVelocity(double &max_velocity)
{
    yarp::os::Value max_vel = _config_file.findGroup("HOMING").find("max_vel");

    if(!max_vel.isNull())
    {
        std::cout<<"Max Vel is: "<<max_vel.asDouble()<<" [deg/sec]\n";
        max_velocity = max_vel.asDouble();
        return true;
    }
    else{
        std::cout<<"ERROR Reading max vel!"<<std::endl;
        return false;
    }
}

void file_parser::help()
{
    std::cout<<"simple_homing\n";
    std::cout<<"help:\n";
    std::cout<<"\nNO ARGUMENT:\n Actual folder will be used and the name of the file is initial_config.ini\n USAGE: ./simple_homing\n";
    std::cout<<"\nSET FILE PATH:\n .simple_homing --context /home/user/demo/config\n";
    std::cout<<"\nSET FILE NAME:\n .simple_homing --from my_initial_configuration.ini\n";
    std::cout<<"\nSET SIMULATION MODE:\n .simple_homing --simulation\n"<<std::endl;
}
