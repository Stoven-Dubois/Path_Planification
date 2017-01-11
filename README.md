# Path_Planification

Trajectory planning, following and autonomous exploration for a 2D robot using the RRT algorithm

Behaviour : 
  - Control through joysticks for placing the robot at a wanted start position ('B' button)
  - Autonomous exploration starting from the position ('X' button)
  - Trajectory planning and following for going back to the start position ('Y' button)

Robot components: 
  - Pioneer body with two parallel dependant wheels
  - Hokuyo 2D laser sensor
  - 5 infrared sensors at the front
  - Wi-Fi communication
  - XBox 360 controller

Language: C++

Used tools: ROS, OpenCV, SublimeText

code_ROS: ROS code
  - control: trajectory following without orientation control, sending of the origin and desired destination for the robot, saturation of the speeds sent
  - controller: choice of the mode and joysticks control using a XBox 360 controller
  - exploration: autonomous exploration for the real robot and test exploration for the simulation
  - map_edition: map creation through gmapping, reduced map with all the useful pixels finding, noise suppression and dilatation of the obstacles
  - minilab_simulation: gazebo package for the simulation
  - planification: trajectory planification using the RRT algorithm

Launch files to use : 
  - planification/launch/planif_rrt_through_gazebo.launch for a quick simulation on Gazebo
  - planification/launch/planif_rrt_real.launch for using with a real robot
