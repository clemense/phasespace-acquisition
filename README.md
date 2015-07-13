# PhaseSpace Acquisition Package
This package provides a ROS node to retrieve data from the *PhaseSpace Motion Capture System*. Gathered data is sent automatically over a local network (check the server ip of the system).

The ROS publisher node, i.e. *phasespace_talker*, publishes data on a topic (by default *phasespace_topic*) using `phasespace_acquisition::PhaseSpaceMarkerArray` messages, which collect markers position for each sample. This data can be stored in log files if necessary, e.g. using a node like the *phasespace_listener* (the two log files created contain the same data in a different format).

## Usage
The publisher and the listener nodes can be started with `rosrun` command and few ROS params can be set to fit your needs: 

 - `rosrun phasespace_acquisition phasespace_talker`
 - `rosrun phasespace_acquisition phasespace_talker _server_ip:=192.168.1.230`
 - `rosrun phasespace_acquisition phasespace_listener`
 - `rosrun phasespace_acquisition phasespace_listener _verbose_mode:=true`

Otherwise a launch file can be set with the actual configuration of the glove. Two launch files with default settings are provided in the package, e.g. to start only the publisher node you can use `roslaunch phasespace_acquisition phasespace_publisher.launch` and to store also the data acquired in log files use `roslaunch phasespace_acquisition phasespace.launch`. Feel free to build others that fit your hardware configuration.

## Info and Warnings

- The *PhasespaceCore* class uses linux-specific commands, e.g. `system("mkdir -p ...")`. 
- This code has been developed for ROS Indigo on ubuntu 14.04. No warranty for other distributions.

## ROS Params
The class provides several parameters which can be set by the user at runtime to handle even distinct hardware configuration:

- topic_name *(pub/sub)*
- topic_queue_length *(pub/sub)*
- verbose_mode *(pub/sub)*
- server_ip *(pub)*
- init_marker_count *(pub)*
- init_flags *(pub)*
- log_file_base_path *(sub)*
- log_file_name *(sub)*
- log_file_name_old *(sub)*
- log_multi_files *(sub)*
