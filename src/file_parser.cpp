#include "file_parser.h"
#include <iostream>

file_parser::file_parser():
    _file_name("initial_config.ini"),
    _file_path("./"),
    _parameter_file_name("--name"), //Without space does not work!
    _parameter_file_path("--path"),  //Without space does not work!
    _parameter_help("--help"),  //Without space does not work!
    _initial_config_file()
{
    load_file();
}

file_parser::file_parser(int argc, char *argv[]):
    _file_name("initial_config.ini"),
    _file_path("./"),
    _parameter_file_name("--name"), //Without space does not work!
    _parameter_file_path("--path"),  //Without space does not work!
    _parameter_help("--help"),  //Without space does not work!
    _initial_config_file()
{
    if(!set_input_parameter(argc, argv))
        exit(0);
    load_file();
}

file_parser::~file_parser()
{

}

bool file_parser::set_input_parameter(int argc, char *argv[])
{
    if(argc-1 > 1)
    {
        for(unsigned int i = 1; i < argc-1; i++)
        {
            if(_parameter_file_name.compare(argv[i]) == 0){
                _file_name = argv[i+1];
            }
            else if(_parameter_file_path.compare(argv[i]) == 0){
                _file_path = argv[i+1];
            }
        }
        return true;
    }
    else if(argc-1 == 1)
    {
        if(_parameter_help.compare(argv[1]) == 0)
            help();
        return false;
    }
    return true;
}

void file_parser::load_file()
{
    std::cout<<"FILE NAME: "<<_file_name<<std::endl;
    std::cout<<"FILE PATH: "<<_file_path<<std::endl;
    std::string file_and_path = _file_path + _file_name;
    if( _file_path != "" && _initial_config_file.fromConfigFile(file_and_path.c_str()) )
    {
        std::cout << "Loading "<<file_and_path<<std::endl;
        _initial_config_file.put("initial_configuration_path",file_and_path.c_str());
    }
}

bool file_parser::getConfiguration(const std::string &chain_name, yarp::sig::Vector& home_configuration)
{
    yarp::os::Value configuration = _initial_config_file.findGroup("HOMING").find(chain_name);

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
    yarp::os::Value max_vel = _initial_config_file.findGroup("HOMING").find("max_vel");

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
    std::cout<<"\nSET FILE PATH:\n .simple_homing --path /home/user/demo/config\n";
    std::cout<<"\nSET FILE NAME:\n .simple_homing --name my_initial_configuration.ini\n"<<std::endl;
}
