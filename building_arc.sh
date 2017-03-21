rm -r ~/catkin_ws/src/arc_tools
rm -r ~/catkin_ws/src/arc_launch
rm -r ~/catkin_ws/src/arc_state_estimation
rm -r ~/catkin_ws/src/arc_interface
rm -r ~/catkin_ws/src/arc_viewer
rm -r ~/catkin_ws/src/arc_obstacle_detection
rm -r ~/catkin_ws/src/pure_pursuit_controller
rm -r ~/catkin_ws/src/guard


git clone git@github.com:ProjectARCeth/arc_launch.git

git clone git@github.com:ProjectARCeth/arc_tools.git

git clone git@github.com:ProjectARCeth/arc_state_estimation.git

git clone git@github.com:ProjectARCeth/arc_obstacle_detection.git

git clone git@github.com:ProjectARCeth/arc_viewer.git

git clone git@github.com:ProjectARCeth/arc_interface.git

git clone git@github.com:ProjectARCeth/pure_pursuit_controller.git
cd ~/catkin_ws/src/pure_pursuit_controller
git checkout Implementation

git clone git@github.com:ProjectARCeth/guard.git
cd ~/catkin_ws/src/guard
git checkout moritz_try


cd ~/catkin_ws/src
catkin build
