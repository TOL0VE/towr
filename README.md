# towr
source ./devel_isolated/setup.bash
roslaunch towr_ros towr_ros.launch
catkin_make_isolated -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1