from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
 return LaunchDescription([
   Node(
     package="simplertk2b",
     executable="simplertk2b_node",
     name="simplertk2b_node",
     output="screen",
     emulate_tty=True,
     parameters=[]
   )
 ])
