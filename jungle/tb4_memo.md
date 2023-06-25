# 目標：tb4を操作するために使うトピック, サービスを特定する。また、各ノード, トピック, サービスの役割を特定する。

## 参考資料

- https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html
- https://github.com/turtlebot/turtlebot4_simulator
- https://github.com/turtlebot/turtlebot4

---

## start

```
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

↑でtb4を起動したあと、GUIでundockすると出てくるメッセージは以下の通り

```sh
[turtlebot4_node-30] [INFO] [1687355269.206799607] [turtlebot4_node]: Undocking
[turtlebot4_node-30] [INFO] [1687355269.206832445] [turtlebot4_node]: Waiting for undock action server
[turtlebot4_node-30] [INFO] [1687355270.207214297] [turtlebot4_node]: undock action server available, sending goal
[motion_control-35] [INFO] [1687355270.207370778] [motion_control]: Received new undock goal
[turtlebot4_node-30] [INFO] [1687355270.207475911] [turtlebot4_node]: undock goal accepted by server, waiting for result
[turtlebot4_node-30] [INFO] [1687355270.438703944] [turtlebot4_node]: OAKD started
[turtlebot4_node-30] [INFO] [1687355270.438744287] [turtlebot4_node]: RPLIDAR started
[motion_control-35] [INFO] [1687355280.119642142] [motion_control]: Undock Goal Succeeded
[turtlebot4_node-30] [INFO] [1687355280.119827735] [turtlebot4_node]: undock goal succeeded
[turtlebot4_node-30] [ERROR] [1687355300.448715313] [turtlebot4_node]: Service oakd/start_camera unavailable.
[turtlebot4_node-30] [ERROR] [1687355300.458949788] [turtlebot4_node]: Service start_motor unavailable.
```

`turtlebot4_node`という名前のノードを探してみる

このときのノード一覧は以下の通り

```sh
# ros2 node list
/bumper_contact_bridge
/buttons_msg_bridge
/camera_bridge
/camera_stf
/cliff_front_left_bridge
/cliff_front_right_bridge
/cliff_side_left_bridge
/cliff_side_right_bridge
/clock_bridge
/cmd_vel_bridge
/controller_manager
/diffdrive_controller
/dock_state_publisher
/gz_ros2_control
/hazards_vector_publisher
/hmi_buttons_msg_bridge
/hmi_display_msg_bridge
/hmi_led_msg_bridge
/interface_buttons_node
/ir_intensity_front_center_left_bridge
/ir_intensity_front_center_right_bridge
/ir_intensity_front_left_bridge
/ir_intensity_front_right_bridge
/ir_intensity_left_bridge
/ir_intensity_right_bridge
/ir_intensity_side_left_bridge
/ir_intensity_vector_publisher
/joint_state_broadcaster
/joint_state_publisher
/kidnap_estimator_publisher
/lidar_bridge
/mock_publisher
/motion_control
/odom_base_tf_bridge
/pose_bridge
/pose_republisher_node
/robot_state
/robot_state_publisher
/rplidar_stf
/sensors_node
/tf_odom_std_dock_link_publisher
/turtlebot4_ignition_hmi_node
/turtlebot4_node
/ui_mgr
/wheel_status_publisher
```

`ros2 node info`で`turtlebot4_node`ノードの情報を確認する

```sh
# ros2 node info /turtlebot4_node
/turtlebot4_node
  Subscribers:
    /battery_state: sensor_msgs/msg/BatteryState
    /dock_status: irobot_create_msgs/msg/DockStatus
    /hmi/buttons: turtlebot4_msgs/msg/UserButton
    /hmi/display/message: std_msgs/msg/String
    /hmi/led: turtlebot4_msgs/msg/UserLed
    /interface_buttons: irobot_create_msgs/msg/InterfaceButtons
    /joy: sensor_msgs/msg/Joy
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /wheel_status: irobot_create_msgs/msg/WheelStatus
  Publishers:
    /function_calls: std_msgs/msg/String
    /hmi/display: turtlebot4_msgs/msg/UserDisplay
    /ip: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /turtlebot4_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlebot4_node/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlebot4_node/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlebot4_node/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlebot4_node/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlebot4_node/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:
    /e_stop: irobot_create_msgs/srv/EStop
    /oakd/start_camera: std_srvs/srv/Trigger
    /oakd/stop_camera: std_srvs/srv/Trigger
    /robot_power: irobot_create_msgs/srv/RobotPower
    /start_motor: std_srvs/srv/Empty
    /stop_motor: std_srvs/srv/Empty
  Action Servers:

  Action Clients:
    /dock: irobot_create_msgs/action/Dock
    /led_animation: irobot_create_msgs/action/LedAnimation
    /undock: irobot_create_msgs/action/Undock
    /wall_follow: irobot_create_msgs/action/WallFollow
```

もう1つ、`motion_control`ノードを確認する

```sh
# ros2 node info /motion_control 
/motion_control
  Subscribers:
    /clock: rosgraph_msgs/msg/Clock
    /cmd_vel: geometry_msgs/msg/Twist
    /dock_status: irobot_create_msgs/msg/DockStatus
    /hazard_detection: irobot_create_msgs/msg/HazardDetectionVector
    /ir_intensity: irobot_create_msgs/msg/IrIntensityVector
    /kidnap_status: irobot_create_msgs/msg/KidnapStatus
    /odom: nav_msgs/msg/Odometry
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /sim_ground_truth_dock_pose: nav_msgs/msg/Odometry
    /sim_ground_truth_pose: nav_msgs/msg/Odometry
    /tf: tf2_msgs/msg/TFMessage
    /tf_static: tf2_msgs/msg/TFMessage
  Publishers:
    /diffdrive_controller/cmd_vel_unstamped: geometry_msgs/msg/Twist
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /wheel_status: irobot_create_msgs/msg/WheelStatus
  Service Servers:
    /e_stop: irobot_create_msgs/srv/EStop
    /motion_control/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /motion_control/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /motion_control/get_parameters: rcl_interfaces/srv/GetParameters
    /motion_control/list_parameters: rcl_interfaces/srv/ListParameters
    /motion_control/set_parameters: rcl_interfaces/srv/SetParameters
    /motion_control/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /robot_power: irobot_create_msgs/srv/RobotPower
  Service Clients:

  Action Servers:
    /dock: irobot_create_msgs/action/Dock
    /drive_arc: irobot_create_msgs/action/DriveArc
    /drive_distance: irobot_create_msgs/action/DriveDistance
    /navigate_to_position: irobot_create_msgs/action/NavigateToPosition
    /rotate_angle: irobot_create_msgs/action/RotateAngle
    /undock: irobot_create_msgs/action/Undock
    /wall_follow: irobot_create_msgs/action/WallFollow
  Action Clients:
```

`motion_control`ノードの`undock`アクションが怪しいので、情報を見てみる

```sh
# ros2 action info /undock
Action: /undock
Action clients: 1
    /turtlebot4_node
Action servers: 1
    /motion_control

# ros2 interface show irobot_create_msgs/action/Undock
# Request
---
# Result
bool is_docked
---
# Feedback
```

ターミナルから以下のコマンドで直接undock操作ができる

```sh
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}\
```

メッセージ内容が謎なので、直接プログラムの覗いてみる...が場所がわからないので`/rosout`経由でソースコードの場所を探してみる  
undock操作の開始〜終了までに出力された`/rosout`の中身がこれ

```sh
stamp:
  sec: 1687357394
  nanosec: 967003821
level: 20
name: turtlebot4_node
msg: Undocking
file: ./src/turtlebot4.cpp
function: undock_function_callback
line: 467
---
stamp:
  sec: 1687357394
  nanosec: 967099972
level: 20
name: turtlebot4_node
msg: Waiting for undock action server
file: ./include/turtlebot4_node/action.hpp
function: send_goal
line: 46
---
stamp:
  sec: 1687357395
  nanosec: 967320877
level: 20
name: turtlebot4_node
msg: undock action server available, sending goal
file: ./include/turtlebot4_node/action.hpp
function: operator()
line: 60
---
stamp:
  sec: 1687357395
  nanosec: 967554080
level: 20
name: motion_control
msg: Received new undock goal
file: ./src/motion_control/docking_behavior.cpp
function: handle_undock_goal
line: 233
---
stamp:
  sec: 1687357395
  nanosec: 967699157
level: 20
name: turtlebot4_node
msg: undock goal accepted by server, waiting for result
file: ./include/turtlebot4_node/action.hpp
function: goal_response_callback
line: 149
---
stamp:
  sec: 1687357396
  nanosec: 204399721
level: 20
name: turtlebot4_node
msg: OAKD started
file: ./src/turtlebot4.cpp
function: oakd_start_function_callback
line: 585
---
stamp:
  sec: 1687357396
  nanosec: 204462739
level: 20
name: turtlebot4_node
msg: RPLIDAR started
file: ./src/turtlebot4.cpp
function: rplidar_start_function_callback
line: 557
---
stamp:
  sec: 1687357402
  nanosec: 627325148
level: 40
name: turtlebot4_node
msg: Service oakd/stop_camera unavailable.
file: ./include/turtlebot4_node/service.hpp
function: operator()
line: 109
---
stamp:
  sec: 1687357402
  nanosec: 637525796
level: 40
name: turtlebot4_node
msg: Service stop_motor unavailable.
file: ./include/turtlebot4_node/service.hpp
function: operator()
line: 174
---
stamp:
  sec: 1687357405
  nanosec: 920022310
level: 20
name: motion_control
msg: Undock Goal Succeeded
file: ./src/motion_control/docking_behavior.cpp
function: execute_undock
line: 343
---
stamp:
  sec: 1687357405
  nanosec: 920254941
level: 20
name: turtlebot4_node
msg: undock goal succeeded
file: ./include/turtlebot4_node/action.hpp
function: result_callback
line: 177
---
stamp:
  sec: 1687357426
  nanosec: 214197740
level: 40
name: turtlebot4_node
msg: Service oakd/start_camera unavailable.
file: ./include/turtlebot4_node/service.hpp
function: operator()
line: 109
---
stamp:
  sec: 1687357426
  nanosec: 224408384
level: 40
name: turtlebot4_node
msg: Service start_motor unavailable.
file: ./include/turtlebot4_node/service.hpp
function: operator()
line: 174
---
```

ちなみに、launchファイルの方の出力はこれ

```sh
[turtlebot4_node-30] [INFO] [1687357394.967003821] [turtlebot4_node]: Undocking
[turtlebot4_node-30] [INFO] [1687357394.967099972] [turtlebot4_node]: Waiting for undock action server
[turtlebot4_node-30] [INFO] [1687357395.967320877] [turtlebot4_node]: undock action server available, sending goal
[motion_control-35] [INFO] [1687357395.967554080] [motion_control]: Received new undock goal
[turtlebot4_node-30] [INFO] [1687357395.967699157] [turtlebot4_node]: undock goal accepted by server, waiting for result
[turtlebot4_node-30] [INFO] [1687357396.204399721] [turtlebot4_node]: OAKD started
[turtlebot4_node-30] [INFO] [1687357396.204462739] [turtlebot4_node]: RPLIDAR started
[turtlebot4_node-30] [ERROR] [1687357402.627325148] [turtlebot4_node]: Service oakd/stop_camera unavailable.
[turtlebot4_node-30] [ERROR] [1687357402.637525796] [turtlebot4_node]: Service stop_motor unavailable.
[motion_control-35] [INFO] [1687357405.920022310] [motion_control]: Undock Goal Succeeded
[turtlebot4_node-30] [INFO] [1687357405.920254941] [turtlebot4_node]: undock goal succeeded
[turtlebot4_node-30] [ERROR] [1687357426.214197740] [turtlebot4_node]: Service oakd/start_camera unavailable.
[turtlebot4_node-30] [ERROR] [1687357426.224408384] [turtlebot4_node]: Service start_motor unavailable.
```

わからなくなってきたので、ここで一回リセット  
ここまでは見つけた

https://github.com/iRobotEducation/create3_sim/blob/7fccefe91a300ea945aa81df09e7db37898b4eda/irobot_create_common/irobot_create_nodes/src/motion_control_node.cpp#L4

結局、`motion_control`ノードの`undock`アクションにどのような操作ができれば良いのかがわかればいいので、HMI?(gazeboを起動するとくっついてくるGUIアプリ)を調査してみる


-----

```
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py robot_name:=tb4 rviz:=true nav2:=true localization:=true
```

rqt_graph: `rosgraph_01.png`, `rosgraph_01.svg`

```sh
[rviz2-57] Start navigation
[rviz2-57] [INFO] [1687698875.592896404] [rviz2]: NavigateToPose will be called using the BT Navigator's default behavior tree.
[bt_navigator-53] [INFO] [1687698875.593344163] [bt_navigator]: Begin navigating from current location to (1.16, -5.27)
[controller_server-49] [INFO] [1687698875.624165842] [controller_server]: Received a goal, begin computing control effort.
[controller_server-49] [WARN] [1687698875.624215835] [controller_server]: No goal checker was specified in parameter 'current_goal_checker'. Server will use only plugin loaded general_goal_checker . This warning will appear once.
[controller_server-49] [INFO] [1687698876.674382668] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698877.724381865] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698878.774384425] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698879.774400538] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698880.824381345] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698881.874385020] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698882.036740940] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698882.874381220] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698883.186704800] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698883.924382777] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698884.336668173] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698884.974383831] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698885.486720734] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698885.974400493] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698886.886555804] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698887.024383582] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698888.024382409] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698888.036540529] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698889.074384319] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698889.486519978] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698890.124383898] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698890.686406219] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698891.124384739] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698892.174403202] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698892.286408283] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698893.174382095] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698893.486382786] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698894.224426819] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698895.274415917] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698896.274387709] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698897.324385769] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698898.324384062] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698899.374401891] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698900.424394558] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698900.436090520] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698901.424391463] [controller_server]: Passing new path to controller.
[motion_control-35] [WARN] [1687698901.586100374] [motion_control]: Ignoring velocities commanded while an autonomous behavior is running!
[controller_server-49] [INFO] [1687698902.474381163] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698903.474379935] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698904.524385733] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698905.574407173] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698906.574382888] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698907.624385043] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698908.624380457] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698909.674386160] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698910.724483121] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698911.724382892] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698912.774385625] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698913.774386679] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698914.824382695] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698915.874415018] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698916.874404023] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698917.924383966] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698918.924383183] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698919.974384418] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698921.024384776] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698922.024382725] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698923.074386058] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698924.074398477] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698925.124383046] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698926.174389446] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698927.174387402] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698928.224384499] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698929.224382037] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687698930.028027552] [controller_server]: Reached the goal!
[bt_navigator-53] [INFO] [1687698930.063484578] [bt_navigator]: Goal succeeded
```

second time

```sh
[rviz2-57] Start navigation
[rviz2-57] [INFO] [1687701006.808207390] [rviz2]: NavigateToPose will be called using the BT Navigator's default behavior tree.
[bt_navigator-53] [INFO] [1687701006.808640060] [bt_navigator]: Begin navigating from current location to (2.00, 2.06)
[controller_server-49] [INFO] [1687701006.839352048] [controller_server]: Received a goal, begin computing control effort.
[controller_server-49] [WARN] [1687701006.839422017] [controller_server]: No goal checker was specified in parameter 'current_goal_checker'. Server will use only plugin loaded general_goal_checker . This warning will appear once.
[controller_server-49] [INFO] [1687701007.889598208] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701008.939593724] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701009.939689678] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701010.989594228] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701011.989590253] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701013.039595106] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701014.089593737] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701015.089550972] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701016.139623935] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701017.139590689] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701018.189592326] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701019.239598718] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701020.239596336] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701021.289631350] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701022.289616539] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701023.339594490] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701024.389592793] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701025.389593419] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701026.439596018] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701027.439593720] [controller_server]: Passing new path to controller.
[controller_server-49] [INFO] [1687701027.742734310] [controller_server]: Reached the goal!
[bt_navigator-53] [INFO] [1687701027.778807117] [bt_navigator]: Goal succeeded

```

echo /rosout

```sh
$ ros2 topic echo /rosout
stamp:
  sec: 1687701006
  nanosec: 808207390
level: 20
name: rviz2
msg: NavigateToPose will be called using the BT Navigator's default behavior tree.
file: ./src/nav2_panel.cpp
function: startNavigation
line: 782
---
stamp:
  sec: 1687701006
  nanosec: 808640060
level: 20
name: bt_navigator
msg: Begin navigating from current location to (2.00, 2.06)
file: ./src/navigators/navigate_to_pose.cpp
function: initializeGoalPose
line: 207
---
stamp:
  sec: 1687701006
  nanosec: 839352048
level: 20
name: controller_server
msg: Received a goal, begin computing control effort.
file: ./src/controller_server.cpp
function: computeControl
line: 348
---
stamp:
  sec: 1687701006
  nanosec: 839422017
level: 30
name: controller_server
msg: No goal checker was specified in parameter 'current_goal_checker'. Server will use only plugin loaded general_goal_checker . Thi...
file: ./src/controller_server.cpp
function: findGoalCheckerId
line: 324
---
stamp:
  sec: 1687701007
  nanosec: 889598208
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701008
  nanosec: 939593724
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701009
  nanosec: 939689678
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701010
  nanosec: 989594228
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701011
  nanosec: 989590253
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701013
  nanosec: 39595106
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701014
  nanosec: 89593737
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701015
  nanosec: 89550972
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701016
  nanosec: 139623935
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701017
  nanosec: 139590689
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701018
  nanosec: 189592326
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701019
  nanosec: 239598718
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701020
  nanosec: 239596336
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701021
  nanosec: 289631350
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701022
  nanosec: 289616539
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701023
  nanosec: 339594490
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701024
  nanosec: 389592793
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701025
  nanosec: 389593419
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701026
  nanosec: 439596018
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701027
  nanosec: 439593720
level: 20
name: controller_server
msg: Passing new path to controller.
file: ./src/controller_server.cpp
function: updateGlobalPath
line: 525
---
stamp:
  sec: 1687701027
  nanosec: 742734310
level: 20
name: controller_server
msg: Reached the goal!
file: ./src/controller_server.cpp
function: computeControl
line: 398
---
stamp:
  sec: 1687701027
  nanosec: 778807117
level: 20
name: bt_navigator
msg: Goal succeeded
file: /opt/ros/humble/include/nav2_behavior_tree/bt_action_server_impl.hpp
function: executeCallback
line: 230
---
```