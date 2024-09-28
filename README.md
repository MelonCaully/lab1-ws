Q1)source /opt/ros/foxy/setup.bash: This command sets up your environment to use the global installation of ROS 2
source install/local_setup.bash: This command sets up your environment to use the specific packages you've built in your workspace.

Q2)For publishers, it limits the number of outgoing messages that can be buffered before being sent.
For subscribers, it controls how many incoming messages can be held in the queue before being processed.
A larger queue_size allows handling spikes in message traffic without losing messages, but it also increases memory usage. A smaller queue_size minimizes memory usage but increases the likelihood of dropping messages during high-traffic periods.

Q3)Calling ros2 launch from the directory where the launch file is: No
Calling ros2 launch when the launch file is installed with the package: Yes