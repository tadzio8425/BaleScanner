from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Static transform: map → base_frame
    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_frame_tf',
        arguments=[
            '0', '0', '2.43719',
            '0', '1.5708', '1.5708',
            'map', 'base_frame'
        ]
    )

    # Static transform: base_frame → left_frame
    left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_frame_to_left_frame_tf',
        arguments=[
            '-0.06995', '-0.68064', '2.43719',
            '-1.5708', '1.0472','3.14159',
            'base_frame', 'left_frame'
        ]
    )

    # Static transform: base_frame → right_frame
    right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_frame_to_right_frame_tf',
        arguments=[
            '-0.06995', '2.04064', '2.43719',
            '1.5708', '-1.0472','0',
            'base_frame', 'right_frame'
        ]
    )

    # Start the reconstructor node **after static transforms**
    bale_reconstructor = Node(
        package='bale_scanner',
        executable='bale_reconstructor'
    )
    # Start the visualizer node 
    bale_isolator = Node(
        package='bale_scanner',
        executable='bale_isolator'
    )


    # Start the static mapper node
    static_service = Node(
        package='bale_scanner',
        executable='trigger_static'
    )

    static_map_publisher = Node(
        package='bale_scanner',
        executable='static_map_publisher'
    )

    #Services
    ld.add_action(static_service)

    # Add actions in order
    ld.add_action(map_tf)
    ld.add_action(left_tf)
    ld.add_action(right_tf)

    ld.add_action(static_map_publisher)
    ld.add_action(bale_reconstructor)
    ld.add_action(bale_isolator)


    return ld
