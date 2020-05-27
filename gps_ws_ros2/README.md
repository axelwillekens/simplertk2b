# Info
Ros version: dashing-diademata (ros2)
# Setup
:~simplertk2b/gps\_ws\_ros2$ colcon build

It is important that the shared object library is in one of the folders that is part of LD\_LIBRARY\_PATH (do echo $LD\_LIBRARY\_PATH to see the files in this volder)
:~simplertk2b/gps\_ws\_ros2$ cp src/simplertk2b/lib/libgps.so /opt/ros/dashing/lib

# Run
:~simplertk2b/gps\_ws\_ros2$ . install/local\_setup.bash
:~simplertk2b/gps\_ws\_ros2$ ros2 run simplertk2b\_node simplertk2b\_node
