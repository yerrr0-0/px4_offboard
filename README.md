# px4_offboard

### Description

디스크 조각 모음 같은 코드 조각 모음


### Code Tree

```
mumt_ws
├── build
├── install
├── log
└── src
    ├── common_interfaces
    ├── Micro-XRCE-DDS-Agent
    ├── px4_msgs
    └── px4_ros_com
        ├── include
        ├── launch
        ├── resource
        ├── scripts
        └── src
            └── examples
                ├── advertisers
                ├── listeners
                │   ├── sensor_combined_listener.cpp
                │   └── vehicle_gps_position_listener.cpp
                └── offboard (<-- this repository)
                    ├── cpp_visualizer.cpp
                    ├── offboard_control.cpp
                    ├── trajectory_circle_gz.cpp
                    └── trajectory_circle.cpp
```
