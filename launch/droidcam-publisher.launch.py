from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            description='DroidCam V4L2 Linux device number'
        ),
        DeclareLaunchArgument(
            'ip',
            description='DroidCam device`s WiFi IP'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='4747',
            description='DroidCam device`s port'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='image_raw',
            description='Ouput topic name for publisher'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='droidcam',
            description='Frame ID to publish image with'
        ),
        Node(
            name='droidcam_client',
            package='droidcam4ros',
            executable='droidcam-cli',
            arguments=[LaunchConfiguration('ip'), LaunchConfiguration('port')]
        ),
        Node(
            name='droidcam_publisher',
            package='droidcam4ros',
            executable='droidcam-publisher',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'output_topic': LaunchConfiguration('output_topic'),
                'frame_id': LaunchConfiguration('frame_id')
            }]
        )
        ])