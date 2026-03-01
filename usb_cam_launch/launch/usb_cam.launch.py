import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    usb_cam_params_file = os.path.join(
        get_package_share_directory('usb_cam_launch'), 
        'config',
        'cam_params.yaml'
    )



    return LaunchDescription([
        Node(
            package='usb_camera_reader',  
            executable='usb_camera_reader', 
            name='usb_camera',
            output='screen',
            parameters=[usb_cam_params_file],  
            remappings=[
                ('/image_raw', '/camera/image_raw'), 
                ('/camera_info', '/camera/camera_info'),  
            ]
        ),
    ])
