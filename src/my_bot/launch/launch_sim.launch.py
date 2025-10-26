import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Get the share directory for the ros_gz_sim package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


    # --- UPDATED LAUNCH FOR JAZZY (using gz_sim.launch.py) ---

    # 1. Start the Gazebo simulation Server
    gz_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')]), # <-- Use main launch file
                # Pass '-s' to start server-only, '-r' to load a world
                launch_arguments={'gz_args': '-s -r empty.sdf'}.items()
             )

    # 2. Start the Gazebo simulation GUI Client
    gz_client = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')]), # <-- Use main launch file
                # Pass '-g' to start GUI-only
                launch_arguments={'gz_args': '-g'}.items()
             )


    # Run the spawner node from the ros_gz_sim package.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.25'], # Set initial height
                        output='screen')
    
    
    # --- UPDATED BRIDGE ---
    # This bridge connects ROS 2 topics to the namespaced Gazebo topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge ROS 2 /cmd_vel to Gazebo /model/my_bot/cmd_vel (ROS 2 -> Gz)
            '/cmd_vel@geometry_msgs/msg/Twist@/model/my_bot/cmd_vel@gz.msgs.Twist',
            
            # Bridge Gazebo /model/my_bot/odometry to ROS 2 /odom (Gz -> ROS 2)
            '/odom@nav_msgs/msg/Odometry@/model/my_bot/odometry@gz.msgs.Odometry',
            
            # Bridge Gazebo /model/my_bot/tf to ROS 2 /tf (Gz -> ROS 2)
            '/tf@tf2_msgs/msg/TFMessage@/model/my_bot/tf@gz.msgs.TFMessage'
        ],
        # Remap /tf from the bridge to /tf_sim to avoid conflicts with robot_state_publisher
        remappings=[('/tf', '/tf_sim')], 
        output='screen'
    )


    # Launch them all!
    # We launch the server first, then the bridge and spawner, then the GUI client.
    return LaunchDescription([
        rsp,
        gz_server,
        bridge,
        spawn_entity,
        gz_client 
    ])
