以下のlaunchファイルについて中身を調査する(launchファイルの勉強も兼ねて)
目標：どのようなlaunchファイル, ノードを呼び出し, 実行しているかを把握する

`turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py`
(`ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py`)

参考資料

- https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py
- https://github.com/ros2/launch/blob/foxy/launch/doc/source/architecture.rst


```py
# 指定したパッケージのshareディレクトリのパスを取得する関数(?)
# pythonパッケージの作成に使っているament(?)が提供しているAPIの1つ
from ament_index_python.packages import get_package_share_directory

# launchファイルはgenerate_launch_description関数がLaunchDescriptionを返す構造にしなければならない
from launch import LaunchDescription
# launchファイルを実行する際のコマンドライン引数?ROS2パラメータ?を指定するためのもの?
from launch.actions import DeclareLaunchArgument
# 他のlaunchファイルを実行するためのもの?
from launch.actions import IncludeLaunchDescription
# 他のパッケージにあるlaunchファイルを引っ張ってくるためのもの?
from launch.launch_description_sources import PythonLaunchDescriptionSource
# LaunchConfigurationは他のlaunchファイルを呼び出すときに引数(コマンドライン?ROS2パラメータ?)を設定するためのもの?
# PathJoinSubstitutionは呼び出すlaunchファイルのパスを作成するためのもの?
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


# ARGUMENTSは定数として宣言した配列
# launchファイルの
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
]

# 
for pose_element in ['x', 'y', 'z', 'yaw']:
    # ARGUMENTSは配列なのでappendメソッドが使える
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    # 自分のパッケージがディレクトリのどこに居るかを確認する
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')

    # Paths
    # ignition_launchはignition.launch.py(launchファイル)のファイルパスを保持する変数?
    ignition_launch = PathJoinSubstitution(
        # pkg_turtlebot4_ignition_bringupパッケージの, launchディレクトリにある, ignition.launch.pyという名前のlaunchファイル?
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    # robot_spawn_launchはturtlebot4_spawn.launch.py(launchファイル)のファイルパスを保持する変数?
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])

    # ignition.launch.py(launchファイル)の設定
    # IncludeLaunchDescription(呼び出すlaunchファイル, launch_arguments=[呼び出すlaunchファイルに渡す引数の設定])?
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    return ld
```