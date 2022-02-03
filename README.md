## wWhat tools are used:

I used moveit package to configure ur5e robot with rg2 gripper. Moveit proveides the the posibilirt of movement planning with wide range of Kinematic tasks solver and path search algorithms. 

Path Search algorithm used: KPiece
KinematikSolver: https://github.com/guihomework/trac_ik_kinematics_plugin

Trajectory search is held in joint-state

## Build && Run

### Using docker image

```bash
$ docker pull ghcr.io/remyrobotics/robotics-test:latest
$ xhost local:root
$ docker-compose up
```

Alternatively, you can build manually with the given Dockerfile.

### Building from Source

```bash
$ cd catkin_ws
$ catkin build
$ source devel/setup.bash
$ roslaunch simple_scene gazebo.launch
```

## Evaluation

You can use any 3rd party package. We would like to see how you design the system and the planning & control pipeline.

## Submission

A docker image and the source code are required. Please explain your design choices. If it is not possible due to circumstances, please send us source codes with clear instructions and explanations.

## Anwsers

>- How would you improve the efficiency and reliability?
   
   * For reaching the reliabilty in crucial tasks the redundancy may be applied:
    For instance: two computers might carry out the control and plannig simultaniously, but only one of them would have acces to the real control. In case of failure (detected by a watchdog, for example), the control may be transfered to the working computer, while another may safetly be reboot
   * If the number of movements of the robot is supposed to be the finite number (for instance there is a finite numer of target positions, or the quantity of major targets is limited) the trajectories of the robot may be computed beforehand by the algorithms guarantiing to find the optimal trjectory (Dijkstra, Astar, etc)

- What would you do if we needed multiple robots in a system?
  * The first thing I would try to do is to adjust the worskpaces of robots so that they would not intersect;
  * If it is imposible, than the planning of robot should be done for both of robots. From my point of view there are two options. Asume that we have k robots:
  	1. Carry out the planning tasks sequentially, one by one. Every task in that case takes the  the privious solutions as as collisions and held in vim(DoF + 1 (time)) space. Totaly it is required to solve k tasks in vim(DoF) space. The solution in this case can be not optimal, but the complexity is relatively low;
  	2. Carry out the planning tasks as one, thus the node of the trajectory path is in this case has concern the prarmeters of all the robots, and in that case the dim of the path search task will bee k*(DoF). This approach can more likely lead to the optimal trajectory, however it is more complex.
  
- How would you deploy the application to multiple physical locations? What is needed to scale it?
 * First of all the decent ammount of QA is required in order to reduce the quantity of upcoming problems and to reduce the need to solve technical problems. 
	


