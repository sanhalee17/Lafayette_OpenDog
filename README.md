# Lafayette_OpenDog
   The repository contains:
      1. OpenDog ROS launch file
      2. OpenDog Ros Python nodes for 2D forward walking
      3. Arduino ROS limit switch node
      
   # Nov.13.2019
   1.Output funcions of limit switches were added to odrive_interface.py and odrive_node.py
   2. Inverse_kinematics.py is added to the package
   # Nov.14.2019
   The Arduino limit switch node is completed. Variable names may change in the future, but it is fully functioning
   # Nov.24.2019
   The launch file (odrive_wlims.launch) has been modified. 6 odrive nodes are added with specified hex serial numbers, and 
   2d motion ROS nodes are added (IK, foot path, motor position).
