import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('roship_vessel'),
        'config',
        'lci_winch.yaml'
    )

    return LaunchDescription([
        # Uncomment and configure to launch the roship_io serial connection:
        # Node(
        #     package    = 'roship_io',
        #     executable = 'serial_connection',
        #     name       = 'serial_vessel',
        #     parameters = [{
        #         'port':      '/dev/ttyUSB0',
        #         'baud_rate': 9600,
        #     }],
        # ),

        Node(
            package    = 'roship_vessel',
            executable = 'lci_winch',
            name       = 'lci_winch',
            namespace  = 'vessel',
            parameters = [config],
        ),
    ])
