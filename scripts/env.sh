# source ~/nowy_rov/ROV_ROS2/scripts/env.sh  - srodowisko ROS 2 + agent + pakiety ROV na Jetsonie
source /opt/ros/humble/setup.bash
source $HOME/nowy_rov/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
alias rov='ros2 run rov_tools rov'
alias rov_agent='ros2 launch rov_bringup rov.launch.py'
