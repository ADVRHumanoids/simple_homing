#ifndef _FILE_PARSER_H_
#define _FILE_PARSER_H_

#include <string>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

class file_parser
{
public:
    file_parser();
    file_parser(int argc, char* argv[]);
    ~file_parser();

    bool getConfiguration(const std::string &chain_name, yarp::sig::Vector& home_configuration);
    bool getMaxVelocity(double& max_velocity);
    void help();


private:
    std::string _file_name;
    std::string _file_path;
    std::string _parameter_file_name;
    std::string _parameter_file_path;
    std::string _parameter_help;
    yarp::os::Property _initial_config_file;

    bool set_input_parameter(int argc, char *argv[]);
    void load_file();
};

#endif
