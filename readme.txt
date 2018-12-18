You need to install python (2.7 or higher), ros kinetic, pip in your computer before running these files
After installing the above, create a catkin workspace, then copy all these files into your catking workspace
before running the program, you also need to install the python package - 
-pyquartenion
-pcanbasic (for can commands only)
-ROS package ackermann_msgs (already included in the workspace)


Then make your workspace and run the following commands


For teleoperation of the gazebo model of car:

roslaunch ackermann_vehicle_gazebo ackermann_vehiclekeyboard_teleop.launch

Use w,a,s and d to move around

----------------------------------------------------------------------------------------------------------------------


For PID testing: ( I have given the car a target heading direction towards east and a constant forward velocity)

Done for 2 different models-
 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Model no. 1- contains a model of the e2o car along with the ackermann drive system
NOTE: The dimensions of the wheelbase, wheels and chassis have been slightly changed so as to fit .dae (3D model of the car)

roslaunch test_scripts pid_auto_testing.launch

Since the above launch file contains too many nodes, it wasn't running in my laptop so I had to launch some of the nodes in seperate terminal windows:

1)roslaunch ackermann_vehicle_gazebo ackermann_vehicle_teleop.launch
2)rosrun test_scripts find_angle.py (simple test case) OR rosrun test_scripts_find_angle_turn.py (setpoint changes with time)
3)roslaunch test_scripts pid_without.launch
4)rosrun test_scripts pid_test.py



After running the above nodes, in a new terminal window:

rosrun pid autotune ( will automatically find the most appropriate values of Kp,Ki and Kd and set the pid controller accordingly, this will require a number of iterations. Please be patient and try until you get the best value possible)

						OR

Run a manually autotuned PID controller

1)roslaunch ackermann_vehicle_gazebo ackermann_vehicle_teleop.launch
2)rosrun test_scripts find_angle.py (simple test case) OR rosrun test_scripts_find_angle_turn.py (setpoint changes with time)
3)roslaunch test_scripts pid_with.launch
4)rosrun test_scripts pid_test.py


Purpose of the above mentioned functions:


You can use 4) to manually enter the values of Kp, Ki and Kd (in the steer_pid tab)

find_angle.py is used to find the current heading direction of the car using the gazebo IMU plugin ( I have also added a gaussian noise to it in order to get a more realistic value). It also publishes the /setpoint and /state which are used as inputs for the PID controller. You can change the target heading direction in this file

rqt_robot_monitor displays any kind of error that comes during the simulation

pid_test.py is used to set the target heading direction of the car and takes in a constant input velocity for the car.

rqt_plot is used to visualise the control effort, setpoint and state with respect to time. (Note, the control effort should approach zero whereas setpoint should be approaching the state value) 
  
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Model no. 2- Contains only the chassis and the wheels, the external dimensions are taken from 
https://www.cardekho.com/mahindra/mahindra-e2o-specifications.html

roslaunch test_scripts pid_auto_testing2.launch
( This one is working in my laptop)

In a new window:
rosrun pid autotune
(for autotuning)
----------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------

For CAN commands to ackermann drive conversion: ( This requires two working systems)

rosrun test_scripts pcan_read_only_try3.py (run this on the laptop)
roslaunch ackermann_vehicle_gazebo ackermann_vehicle_teleop.launch (also on the laptop)

rosrun e2o pcan_e2o_drv_trial.py (on the car's industrial pc)
rosrun e2o ps4_e2o_ctrl.py (on the car's industrial pc)

In case gazebo crashes, just open a new terminal and run the file run_gazebo.sh and then try again

----------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------

Sensor outputs

The gazebo model has 3 sensors:

1)IMU
2)stereocamera
3)Monocular camera

There respective outputs can be seen by the following commands:

- rostopic echo /imu

- rosrun image_view image_view image:=/multisense_sl/camera/right/image_raw rosrun image_view image_view image:=/multisense_sl/camera/left/image_raw

- rosrun image_view image_view image:=/mybot/camera1/image_raw



Features:

Used quarternions for rotation 
added gaussian sensor error to the imu, added a constant steer zero error
used zeigler nichols oscillation method for autotuning


----------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------

Directly control the gazebo model using a dualshock controller

write the pre-requisite packages
1)roscore
2)rosrun joy joy_node
3)rosrun ps4_ros ps4_ros
4)rostopic echo /joy
5)rosrun e2o ps4_e2o_ctrl.py
6)rostopic echo /e2octrl
7)rosrun test_scripts ps4-2-ackermann.py
8)rostopic echo /ackermann_cmd
9)roslaunch ackermann_vehicle_gazebo ackermann_vehicle_teleop.launch


controller with both pcs 
follow this https://github.com/solbach/ps4-ros

