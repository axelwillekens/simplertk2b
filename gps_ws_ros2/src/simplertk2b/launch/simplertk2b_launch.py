from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
 return LaunchDescription([
   Node(
      package='simplertk2b_node',
      node_executable='simplertk2b_node',
      node_name='simplertk2b_node'
   )
 ])
