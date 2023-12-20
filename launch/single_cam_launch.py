import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_cam_if', executable='eos_node', name='cam', output='screen')
    ])