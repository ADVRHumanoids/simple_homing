SIMPLE_HOMING
=============

The module brings part of the joints of the robot to the homing configuration. It is meant to demonstrate a very basic functionality of a generic module and the robot interfe. 

Usage (simulation):
------------
Before using the module make sure to start yarp server by:
```
yarpserver --write
```
Then run gazebo simulator and insert the robot. Now you are ready to launch the module. 
After executing the binary you have to start the module execution by writing to the yarp port "switch:i" of the module:
```
yarp write ... /simple_homing/switch:i
>> start
```
this way the module will go through the initialization procedure and start the execution of the main thread `run()`. To start the homing procedure you have to write to the port "command:i" of the module: 
```
yarp write ... /simple_homing/command:i
>> homing
```


Note on config file location:
------------
Simple homing looks first in the folder in which is launched for a file called initial_config.ini. To change the path and/or the name run simple_config with ```--path``` and/or ```--name``` options.
