from launch import LaunchDescription
from launch.descriptions import executable
from launch_ros.actions import Node

def generate_launch_description():

    demo_messaging_node = Node(
      package="rosbot_xl_cpp_examples",
      executable="demo_messaging_node",
      name="demo_messaging_node",
      remappings=[("/image", "/camera/color/image_raw")],
      parameters=[{"timer_period_s": 2}]
    )

    image_saver = Node(
        package="image_view",
        executable="image_saver",
        name="image_saver",
        remappings=[("/image", "/camera/color/image_raw"), ("/camera_info", "/camera/color/camera_info")],
        parameters=[{"save_all_image":False}] 
    )

    return LaunchDescription([demo_messaging_node, image_saver])
