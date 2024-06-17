# thesis_robot_cpp
ROS2 Humble C++ package containing the <span style="color: #1E90FF;">**mujoco_gym** </span> and <span style="color: #1E90FF;">**sim_time**</span> nodes implementation.

# <span style="color: #1E90FF;">mujoco_gym node</span>

# -> Functionality
**mujoco_gym** node implements a robot server that can simulate UR5e robots controlled in cartesian velocity space. 

It based on MuJoCo for simulation and on GLFW for rendering. It is inspired in the "basic" sample that comes with the official MuJoCo distribution. The simulation step and the the rendering steps are carried out in different threads spinning at different frequencies, in particular the simulation clocks at 500Hz, while rendering does at 25Hz. This strategy is inspired  on a Q&A post that can be found [here](https://github.com/google-deepmind/mujoco/issues/1402). The simulation step is spinned using a ROS2 timer, whereas the rendering is spinned using a common C++ thread. The need to use a common C++ thread implementation is due to the use of a GLFW context.

The current implementation of this package is using MuJoCo 3.1.4, including in the package itself the libraries and header files corresponding to it. Hence, it is not needed to download anything externally. On the contrary, for the rendering functionalities GLFW is used and it must be installed in the system. It can be done with the following command:

```bash
sudo apt-get install libglfw3-dev
```
As defined later in the API description, robot instances can be started and stopped using services. Once the robot is alive, it publishes topics with different information such as actual positions or velocities. These streams are then used by the controller to produce proper actions.

The robot is internally commanded using joint velocities actuators. Thus, the commanded cartesian velocity for the robot end-effector is fed to the an Inverse Kinematics algorithm to compute the desired joint velocities. In particular we are using the inverse jacobian solver:

$
\dot{q}_{cmd} = J^{+}_{tip} \cdot \dot{x}_{cmd}
$

See frame reference description in main README.md for the ***"tip"*** frame description.

The data streams coming from the force sensor are expressed in the ***"ft_sensor"*** frame. So, in order to be consistent with the rest of cartesian magnitudes, it is expressed in the ***"world/fixed"*** frame before streaming to the topic.

$
f_{act} = R_{sensor\_ft} \cdot f_{measured} \\
t_{act} = R_{sensor\_ft} \cdot t_{measured}
$


The robot description .xml file can be found in the **robot/** folder. It is a modification of the publicly available robot description in the [MuJoCo repositories](https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e). The robot is intially spawned in the keyframe pose defined in the .xml.

<img src="../docs/images/robot_visu.png" alt="Description of the image" width="800"/>

# -> ROS2 API

![alt text](../docs/images/mujoco_gym.png)

## <span style="color: #1E90FF;">Parameters</span> 

### <span style="color: #1E90FF;">/use_sim_time</span> 
Parameter of bool type. 

If set to False node will use system clock, if set to True it will use a simulated clock published in the /clock topic.

If not explicitly defined, set to False.


## <span style="color: #228B22;">Topic subscribers</span> 

### <span style="color: #228B22;">/cmd_cart_vel</span> 
Topic stream of the type **TwistStamped**, see [offical documentation](https://docs.ros2.org/latest/api/geometry_msgs/msg/TwistStamped.html) for definition.

It must be published by the controller (**force_control_hub** node) at a expected rate of 500Hz.

It contains the commanded velocity for the robot end-effector in cartesian space, referenced to the fixed frame. The message is composed by 3 linear and 3 angular velocities.

ros2 topic pub /cmd_cart_vel geometry_msgs/TwistStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'


## <span style="color: #B22222;">Topic publishers</span>  

### <span style="color: #B22222;">/sensor_ft_raw</span>  
Topic stream of the type **WrenchStamped**, see [offical documentation](https://docs.ros2.org/latest/api/geometry_msgs/msg/WrenchStamped.html) for definition.

It is published at the simulation frequency, i.e. 500Hz. It is subscribed by the controller (**fornodece_control_hub** ).

It contains the measured force by the external force sensor measurement, referenced to the fixed frame. The message is composed by 3 forces and 3 torques.

### <span style="color: #B22222;">/act_cart_pos</span>  
Topic stream of the type **PoseStamped**, see [offical documentation](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html) for definition.

It is published at the simulation frequency, i.e. 500Hz. It is subscribed by the controller (**fornodece_control_hub** ).

It contains the actual position of the robot end-effector in cartesian space, referenced to the fixed frame. The message is composed by a 3D position and a quaterion to depict the orientation.

### <span style="color: #B22222;">/act_cart_vel</span>
Topic stream of the type **TwistStamped**, see [offical documentation](https://docs.ros2.org/latest/api/geometry_msgs/msg/TwistStamped.html) for definition.

It is published at the simulation frequency, i.e. 500Hz. It is subscribed by the controller (**fornodece_control_hub** ).

It contains the actual velocity of the robot end-effector in cartesian space, referenced to the fixed frame. The message is composed by 3 linear and 3 angular velocities.

### <span style="color: #B22222;">/act_joint state </span>
Topic stream of the type **JointStateStamped**, see [offical documentation](link) for definition.

It is published at the simulation frequency, i.e. 500Hz. It is not subscribed by any other node, it is just published for inspection and hence it could be removed in deployment stages.

It containt the actual state of the robot joints, i.e. position and velocity. The message is composed by the position and velocity of the 6 joints.

## <span style="color: #228B22;">Service clients</span> 
This node does not contains any service client.

## <span style="color: #B22222;">Service servers</span>

### <span style="color: #B22222;">/mujoco_gym/start_single_robot</span>
Service of type **StartSingleRobot**, see **thesis_interfaces** package documentation for definition.

Calling this service starts a robot instance. 

The request parameters are:
- **robot_id**: identification number for the robot that will be started.
- **enable_visu**: flag that if set to True, rendering is also started; otherwise only a simulation in the background will run.

The response parameters are:
- **spawned**: flag that is set to True if the robot was successfully started, and to False if there was any problem. A robot is only successfully started if a robot with that id was not already alive.

Console command example:
```bash
ros2 service call /mujoco_gym/start_single_robot thesis_interfaces/srv/StartSingleRobot "{robot_id: 5, enable_visu: true}"
```
NOTE: If the robot is started with visualization, the GLFW window captures the peripherals interaction: mouse to zoom and drag&drop the visualization, and "back-delete" key to restart the simulation. If there is no visualization, the peripherals are not captured, and thus they do not have any influence in the simulation.

### <span style="color: #B22222;">/mujoco_gym/stop_single_robot</span>
Service of type **StopSingleRobot**, see **thesis_interface** package documentation for definition.

Calling this service stops the robot instance.

The request parameters are:
- **robot_id**: identification number of the robot that will be stopped.

The response parameters are:
- **killed**: flat that is set to True if the robot is successfully stopped, and to False if there was any problem. A robot is only stopped if there is already a robot with that id alive.

Console command example:
```bash
ros2 service call /mujoco_gym/stop_single_robot thesis_interfaces/srv/StopSingleRobot "{robot_id: 5}"
```

### <span style="color: #B22222;">/mujoco_gym/robot_exists</span>
Service of type **RobotExists**, see **thesis_interfaces** package documentation for definition.

Calling this service checks if a robot with a certain id is alive.

The request parameters are:
**robot_id** : identification number of the robot that will be checked

The response parameters are:
**exists** : flag that is set to True if the robot a robot with the given id is already alive.

Console command example:
```bash
ros2 service call /mujoco_gym/robot_exists_id thesis_interfaces/srv/RobotExistsID "{robot_id: 5}"
```

## <span style="color: #228B22;">Action clients</span> 
This node does not include any action client.

## <span style="color: #B22222;">Action server</span>
This node does not include any action server.

# <span style="color: #1E90FF;">sim_time node</span>

# -> Functionality
**sim_time** node implementes a simulated time generator that takes the system clock and scale its rate by a factor (/sim_time_factor parameter), publishing such information in a topic called **/clock**, which will be subscribed by nodes that are configured to use simulated time. 

More information about the use of simulated time in ROS2 can be found [here](https://design.ros2.org/articles/clock_and_time.html).

# -> ROS2 API
![alt text](../docs/images/sim_time.png)

## <span style="color: #1E90FF;">Parameters</span> 

### <span style="color: #1E90FF;">/sim_time_factor</span> 
Parameter of double type. 

It defines the scale factor for the simulated time, i.e. "x" in this parameter means the simulated time is "x" times faster than the real time clock. 

If not explicitly defined, set to 1.0.

## <span style="color: #228B22;">Topic subscribers</span> 

### <span style="color: #228B22;">/parameter_event</span> 
Topic stream of the type **ParameterEvent**, see [offical documentation](https://docs.ros2.org/latest/api/rcl_interfaces/msg/ParameterEvent.html) for definition.

It must be published by roscore everytime there is a change in the parameter server.

It contains the changed parameter names and their new values.

## <span style="color: #B22222;">Topic publishers</span>  

### <span style="color: #B22222;">/clock</span>  
Topic stream of the type **Clock**, see [offical documentation](https://docs.ros2.org/latest/api/rosgraph_msgs/msg/Clock.html) for definition.

It is published at 1MHz. It will be subscribed by any other node that is using simulated time.

It contains a scaled clock message. The scale factor can be modified changing the **/sim_time_factor** parameter.

## <span style="color: #228B22;">Service clients</span> 
This node does not include any service client.

## <span style="color: #B22222;">Service servers</span>
This node does not include any service server.

## <span style="color: #228B22;">Action clients</span> 
This node does not include any action client.

## <span style="color: #B22222;">Action server</span>
This node does not include any action server.






