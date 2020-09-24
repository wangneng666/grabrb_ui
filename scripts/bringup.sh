roslaunch hsr_bringup co602_gripper_go.launch &
roslaunch gripper_bridge gripper_bridge1.launch &
rosrun perception_bridge perception_bridge &
sleep 5
roslaunch kinect2_bridge kinect2_bridge.launch & 
roslaunch hsr_bringup publish_kinect_calibration_.launch &
roslaunch realsense2_camera rs_camera.launch &
roslaunch hsr_bringup publish_d435i_calibration_five_finger.launch &
roslaunch vision_bridge vision_bridge_yolo6d-realsense2.launch &
rosrun dm_bridge dm_bridge &
rosrun hsr_robot_voice pick_assistant &
rosrun planner_bridge planner &
rosrun motion_bridge motion_bridge_exe &
rosservice call /setGripper "gripperName: 'SerialGripper'" &
rosservice call /connectGripper "{}" &
gnome-terminal -x bash -c "roslaunch pickplace_bridge serial_pickplace_bridge.launch " &
gnome-terminal -x bash -c "rosrun hscfsm_bridge hscfsm_bridge " & 

sleep 10
rosrun perception_bridge shelf.sh 

#kill $(ps -ef | grep kinect2 | awk '{print $2}')
