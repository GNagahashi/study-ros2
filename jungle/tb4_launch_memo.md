以下のlaunchファイルについて中身を調査する(launchファイルの勉強も兼ねて)
目標：どのようなlaunchファイル, ノードを呼び出し, 実行しているかを把握する

`turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py`
(`ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py`)

参考資料

- https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py
- https://github.com/ros2/launch/blob/foxy/launch/doc/source/architecture.rst


```py
# 指定したパッケージのshareディレクトリのパスを取得する関数
# `/opt/ros/humble/share/パッケージ名`のディレクトリを得ることができる
# つまり、指定したパッケージ直下のディレクトリ, ファイルにアクセスするためのパスが得られる
from ament_index_python.packages import get_package_share_directory

# launchファイルはgenerate_launch_description関数がLaunchDescriptionを返す構造にしなければならない
from launch import LaunchDescription
# launchファイルを実行する際の引数を設定するためのクラス
# 適切に設定すると、ros2 launchで実行するときに入力補足として出てきてくれる
# IncludeLaunchDescriptionクラス(後述)の引数として設定することもできる
# 引数にデフォルト値が設定された場合、その引数はオプション引数となる
# 一方、引数にデフォルト値を設定しない場合、非オプション引数となり、値が指定されない場合エラーになる模様
# 引数に選択肢がある場合、選択肢以外の値が設定されるとエラーが発生する
from launch.actions import DeclareLaunchArgument
# 他のlaunchファイルを実行するためのクラス
# 実行するlaunchファイルはPythonLaunchDescriptionSourceクラスで指定する
# 実行するlaunchファイルの引数はlaunch_argumentsで指定する
from launch.actions import IncludeLaunchDescription
# 指定されたファイルパスにあるlaunchファイルを引っ張ってくるためのクラス
from launch.launch_description_sources import PythonLaunchDescriptionSource
# LaunchConfiguration
#     DeclareLaunchArgument、つまりlaunchファイルの実行時に渡す引数(iamge:=/camera/image_rawみたいなやつ)を取得するためのクラス
# PathJoinSubstitution
#     プラットフォームに依存しない方法でパスを作成するためのクラス
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    # DeclareLaunchArgument(引数名, default_value=引数のデフォルト値, choices=引数に指定できる値の選択肢(リスト), description=引数の説明)
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

    # turtlebot4_ignition_bringupパッケージが持つlaunchファイル: ignition.launch.pyを実行する
    # IncludeLaunchDescription(呼び出すlaunchファイル, launch_arguments=[呼び出すlaunchファイルに渡す引数])
    # LaunchConfigurationを使ってARGUMENTSから引数に指定する値を取り出している
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    # turtlebot4_ignition_bringupパッケージが持つlaunchファイル: turtlebot4_spawn.launch.pyを実行する
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
    # このlaunchファイル(turtlebot4_ignition_bringupパッケージのturtlebot4_ignition.launch.py)を実行するときの引数を設定する
    ld = LaunchDescription(ARGUMENTS)
    # このlaunchファイルと一緒に実行するlaunchファイルを設定する
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    return ld
```