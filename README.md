SIMPLE_HOMING
=============

Launch:
------
For the usage run: 
```
./simple_homing --help
```

Note on config file location:
------------
Simple homing looks first in the folder in which is launched for a file called initial_config.ini. To change the path and/or the name run simple_config with ```--path``` and/or ```--name``` options.

Usage:
------------
Once launched the module connect to port:
```
/homing/send_trj:i
```
that accept ```true``` when the homing procedure has to start.
