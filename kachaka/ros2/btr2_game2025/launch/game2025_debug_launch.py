from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # パッケージディレクトリの取得
    from ament_index_python.packages import get_package_share_directory

    btr2_game2025_dir = get_package_share_directory('btr2_game2025')

    setup_and_kachaka = ExecuteProcess(
        cmd=['bash', '-c', 
             'source ' + os.path.join(btr2_game2025_dir, 'scripts/setup.sh') + ' && ' +
             os.path.join(btr2_game2025_dir, 'scripts/startKachakaBridge.sh')
        ],
        shell=True,
        output='screen'
    )

    setup_and_game = ExecuteProcess(
        cmd=['bash', '-c', 
             'source ' + os.path.join(btr2_game2025_dir, 'scripts/setup.sh') + ' && ' +
             'ros2 run btr2_game2025 game2025.py'],
        shell=True
    )

    # game2025.py だけ別ターミナルで実行（tmux の例）
    game2025_debug = ExecuteProcess(
        cmd=[
            'tmux', 'new-window', '-n', 'game2025',
            'bash', '-c',
            'source ' + os.path.join(btr2_game2025_dir, 'scripts/setup.sh') + ' && ' +
            'cd ~/git/rcll/kachaka/ros2/btr2_game2025/btr2_game2025; echo $kachaka_IP; python3 game2025.py navigation; read; bash'
        ],
        shell=False
    )

    return LaunchDescription([
        # btr2_odom の起動
        Node(
            package='btr2_odom',
            executable='btr2_odom',
            name='btr2_odom',
            output='screen'
        ),

        # refbox_peer の起動
        Node(
            package='refbox_peer',
            executable='refbox_peer',
            name='refbox_peer',
            output='screen'
        ),

        # startKachakaBridge.sh（setup.sh 実行後）
        setup_and_kachaka,

        # game2025.py（setup.sh 実行後）
        # setup_and_game,
        game2025_debug,
    ])

