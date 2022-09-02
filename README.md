# crazyflie_platform
 AS2 Crazyflie platform interface.

## crazyflie_cpp
This is the main library to control and communicate with the crazyflie drone. Check the Crazyflie.h to get an idea of the main functions.

It is avaliable in GitHub: https://github.com/whoenig/crazyflie_cpp/tree/master

Some examples with this library can be found at:
https://github.com/whoenig/crazyflie_tools

## How to:
Clone the repo and:
```
git submodule update --init --recursive
```

You may change the libmotioncapture CMakeLists.txt in order to adapt to your mocap system.

Then compile as ros pkg

## TODO
- Add Acro control mode. The thrust is a uint16_t type but there is no information about the range that it has. (Whether 0- full range, % , etc).

## Info
- Run debug mode to see more information about the drone and the connection.
- You need to have connected the Crazyflie Radio to connect with the drone.
-  See [project_crazyflie](https://github.com/aerostack2-developers/project_crazyflie) to see an example application with the python interface.