# Reconcycle states

This repository includes python FlexBe states for the ReconCycle project

# Table of contents

- [ReconCycle states](#reconcycle-states)
- [Table of contents](#table-of-contents)
- [Introduction](#introduction)
- [Python files](#python-files)
	- [ROS 1](#ros-1)
- [Behaviors example](#behaviors-example)
- [How to](#simulation-and-flexbe-app)


# Introduction
[FlexBE](http://wiki.ros.org/flexbe/) helps us to create complex robot behaviors without the need for manually coding them.
States define what should be done by a behavior. They are grouped as statemachines, defining the control flow when the behavior is executed. 
Therefore, each state declares a set of outcomes which represent possible results of execution

This repository includes FlexBe states [python files](#python-files)


# Python files
Python files represent FlexBe states which are joined together into one behavior.

## ROS 1  
- [write_to_mongodb (FlexBe state name: WriteToMongo)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/write_to_mongodb.py)
	- input keys: ['entry_data'] = JointState(position)
	- _id to write to in mongodb (['entry_name']) e.q. entry_name="position1".


- [read_from_mongodb (FlexBe state name: ReadFromMongo)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/read_from_mongodb.py)
	- input keys: _id to read from in mongodb (['entry_name']).
	- output keys: ['joints_data'] = JointState(position)


- [call_joint_trap_vel_action_server (FlexBe state name: CallJointTrap)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/call_joint_trap_vel_action_server.py)
	- Input keys: ['joints_data'] = JointState(position)
	- Output keys:[joint_values'] = JointState(position)


- [Read_TF_Cart (FlexBe state name: ReadTFCart))](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/Read_TF_Cart.py)
	- Added target1: lookupTransform()
	- output_keys = ['t1_data'] = Pose()
	- input parameters: 
		- target_frame -> string
    	- source_frame -> string
	

- [CallAction_TF_Cart (FlexBe state name: CallActionTFCart)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/CallAction_TF_Cart.py)
	- Added call to cart_trap_vel_action_server
	- input_keys = ['t1_data'] = Pose()
	- output_keys = ['t1_out'] = Pose()
	

- [Read_TF_CartLin (FlexBe state name: ReadTFCartLin)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/Read_TF_CartLin.py)
	- Added target2: lookupTransform()
	- output_keys = ['t2_data'] = Pose()
	- input_keys = ['offset'] -> list [x,y,z]
	- input_keys =['rotation'] -> list [rotx,roty,rotz]
	- input parameters:
		- target_frame -> string
    	- source_frame -> string
	

- [CallAction_TF_CartLin (FlexBe state name: CallActionTFCartLin)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/CallAction_TF_CartLin.py)
	- Added call to cart_lin_task_action_server
	- input_keys = ['t2_data'] = Pose()
	- output_keys = ['t2_out'] -> result data
	- input parameters:
		- namespace -> string

- [Call_joint_min_jerk_action_server (FlexBe state name: CallJointMinJerk)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/Call_joint_min_jerk_action_server.py)
	- Added call to joint_min_jerk_action_server
	- output_keys = ['minjerk_out'] reply -> []
	- input_keys = ['goal_joint_pos'] list -> [j1...j7]
	- input parameters:
		- motion_duration -> float 
		- motion_timestep -> float
		- namespace -> string

- [toolchanger_rviz (FlexBe state name: SwitchToolsRVIZ)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/toolchanger_rviz.py)
	- Added call to /change_tool_frame/$tool_name
	- output_keys = None
	- input_keys = None
	- input parameters:
		- toolname -> string e.g. "screwdriver"
		- framename -> string  e.g. "tool_frame_1"

- [adjust_tool_in_rviz (FlexBe state name: AdjustToolPositionAndRotationRVIZ)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/adjust_tool_in_rviz.py)
	- Added call to /change_transform_matrix/$tool_name 
	- output_keys = None
	- input_keys = None
	- input parameters:
		- toolname -> string  e.g. "screwdirver"
		- matrix -> string e.g. '[0 0 0 1 0 0]' -> '[x,y,z,rotx,roty,rotz]'

- [load_controller_service_client (FlexBe state name: LoadControllerProxyClient)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/load_controller_service_client.py)
	- output_keys = None
	- input_keys = None
	- input parameteres:
		- desired_controller -> string
		- robot_name -> string

- [unload_controller_service_client (FlexBe state name: UnloadControllerProxyClient)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/unload_controller_service_client.py)
	- output_keys = None
	- input_keys = None
	- input parameteres:
		- desired_controller -> string
		- robot_name -> string
	
- [active_controller_service_client (FlexBe state name: ActiveControllerProxyClient)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/active_controller_service_client.py)
	- output_keys = ['active_controller']
	- input_keys = None
	- input parameteres:
		- real_controllers -> string[] array
		- robot_name -> string

- [switch_controller_service_client (FlexBe state name: SwitchControllerProxyClient)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/switch_controller_service_client.py)
	- output_keys = None
	- input_keys = None
	- input parameteres:
		- robot_name -> string
		- start_controller -> string[] array
		- stop_controller -> string[] array
		- strictness -> int32
	
- [set_load_service_client (FlexBe state name: SetLoadProxyClient)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/set_load_service_client.py)
	- output_keys = None
	- input_keys = None
	- input parameters:
		- mass         ->    float64     Mass of the load in [kg]. 
    	- F_x_center_load ->  float64[3]  Translation from flange to center of mass of load F(xcload) in [m].
    	- load_inertia ->    float64[9]  Inertia matrix I(load) in [kg x m^2], column-major.
    	- robot_name   ->    string      "panda_1" or "panda_2"

- [set_KFrame_service_client (FlexBe state name: SetKFrameProxyClient)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/set_KFrame_service_client.py)
	- output_keys = None
	- input_keys = None
	- input parameters:
		- EE_T_K      ->      float[16]   Vectorized EE-to-K transformation matrix , column-major.
    	- robot_name  ->     string      "panda_1" or "panda_2"

- [set_EEFrame_service_client (FlexBe state name: SetEEFrameProxyClient)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/set_EEFrame_service_client.py)
	- output_keys = None
	- input_keys = None
	- input parameters:
		- NE_T_EE     ->     float[16]   4x4 matrix -> Vectorized NE-to-EE transformation matrix , column-major.
    	- robot_name  ->     string      "panda_1" or "panda_2"

- [set_joint_impedance_service_client (FlexBe state name: SetJointImpedanceProxyClient)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/set_joint_impedance_service_client.py)
	- output_keys = None
	- input_keys = None
	- input parameters:
		- joint_stiffness  ->    float[7]    Joint impedance values for each joint
    	- robot_name       ->    string      "panda_1" or "panda_2"

- [set_cartesian_impedance_service_client (FlexBe state name: SetCartesianImpedanceProxyClient)](/myflexgit_flexbe_states/src/myflexgit_flexbe_states/set_cartesian_impedance_service_client.py)
	- output_keys = None
	- input_keys = None
	- input parameters:
		- cartesian_stiffness  float[6]		[x, y, z, roll, pitch, yaw]
    	- robot_name           string      	"panda_1" or "panda_2"

# Behaviors example
Behaviors are modeled as hierarchical state machines where states correspond to active actions and transitions describe the reaction to outcomes.

A [behavior file](/myflexgit_flexbe_behaviors/src/myflexgit_flexbe_behaviors/flexbefull_sm.py) is constructed from [python files](#python-files). The file is run by FlexBe behavior engine.

More information about FlexBe behavior engine is avaliable [here](https://github.com/team-vigir/flexbe_behavior_engine/blob/master/README.md).


# Simulation and FlexBE app
In order for examples to work, a pre-build panda_dockers has to be build and run:  
- https://github.com/abr-ijs/panda_dockers
- https://github.com/ReconCycle/sim_controllers_interface
- https://github.com/ReconCycle/docker_examples/tree/master/ros1_devel
- FlexBE Dockerfile
	- https://github.com/ReconCycle/docker_examples/tree/master/ros1_flexbee

Example commands for a joint_min_jerk_action_client:
- Create and image from FlexBE Dockerfile (FlexBE Dockerfile):
	- docker build --no-cache -t reconcycle/states:states .
- Run FlexBE app(panda_dockers nad sim_controllers_interface must run before running with this configuration):
	- docker run -it --name rcstate --network panda-simulator-gzweb_ros -p 9092:9092 -e ROS_MASTER_URI=http://rosmaster:11311 reconcycle/states:states roslaunch flexbe_app flexbe_full.launch
- Run just FlexBE app without gazebo simulator:
	- docker run -it --name rcstate -p 9092:9092 -e reconcycle/states:states roslaunch flexbe_app flexbe_full.launch

- When panda_dockers is running and action server is running (sim_controllers_interface), GUI can be access via browser:
	- FlexBE app @ http://localhost:9092/vnc.html in browser
	- GzWeb app @ http://localhost in browser

- For simple move panda in Gzweb simulator a test MoveJointMinJerkExample is provided. The behavior is available under Load Behavior in FlexBe app GUI.
Keep in mind that all the above docker files must be build and run before FlexBE container is created.
		
An example of states inside behavior model. ![here](https://github.com/ReconCycle/reconcycle_states/blob/main/myflexgit_flexbe_states/src/myflexgit_flexbe_states/FlexBe%20Statemachine.png).
