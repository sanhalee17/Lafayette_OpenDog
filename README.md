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
   # Nov.21.2019
   Limit switch is published on rising edge (modified in odrive_node.py)
   # Nov.24.2019
   The launch file (odrive_wlims.launch) has been modified. 6 odrive nodes are added with specified hex serial numbers, and 
   2d motion ROS nodes are added (IK, foot path, motor position).
   Limit switch topic variable is added in the launch file.
   # Nov. 25.2019
   Launch file is updated. It conects 6 odrives successfully
   # Nov.28.2019
   Changed opendog node organization for forward walking motion. The individual leg will have individual foot path. Still need to figure out phase shift.
   # Jan.30.2020
   Odrive connection sequence is created; four odrives will connect in linear order. Yet it does not work as ideal. Need further debugging
   # Feb.12.2020
   updated a launch file with odrive linear connection sequence, debugged forward walking foot path is updated
