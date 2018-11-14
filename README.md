# ros-mtconnect-2-demo

Demo for a distributed digital manufacturing setup (see `https://github.com/mtconnect/ros_mtconnect_2`) in a ROS environment setup.

Following are the steps involved in the ROS environment Setup:

1. Install ROS

2. Create a catkin workspace

3. Clone `ros_mtconnect_2*` repositories

4. Install dependencies

5. Compile the project

6. Setup environment

7. Run demos


## Install ROS

1. Start with a standard Ubuntu 16.04 installation.

2. Follow directions at `http://wiki.ros.org/kinetic/Installation/Ubuntu` to download and install ROS Kinetic. Select the `ros-kinetic-desktop-full` option.

3. Skip section 1.6, `Environment Setup`. The default environment will be set up to point to the project directory instead later.

4. Install catkin_tools, 
    
    `sudo apt-get install python-catkin-tools`


## Create a Catkin Workspace

More details on catkin at `http://wiki.ros.org/catkin/Tutorials`

1. Anywhere in the home directory, create an empty directory with arbitrary name to store the workspace of the project.

    `cd ~`
	
	`mkdir catkin_workspace`

2. Inside the new directory create an empty `src/` directory.


## Clone `ros_mtconnect_2*` repositories

1. Open a terminal and navigate to the `src/` directory of your newly created workspace.
    Run:
    
    `git@github.com:mtconnect/ros_mtconnect_2.git`
    
    `git@github.com:mtconnect/ros_mtconnect_2_demo.git`


## Install Dependencies

Source ros setup:

`. /opt/ros/kinetic/setup.bash`

Packages:

`sudo apt-get install <package_name>`

    ros-kinetic-moveit-core
    ros-kinetic-moveit-ros-planning-interface
    ros-kinetic-abb
    ros-kinetic-universal-robot
    ros-kinetic-soem
    ros-kinetic-socketcan-interface

Some packages must be cloned from their git repositories.

1. Open a terminal and navigate to the `src/` directory of the newly created workspace.
    Run:
    
    `git clone https://github.com/ros-industrial/robotiq.git`

    `git clone https://github.com/NoMagicAi/ur_modern_driver.git`


There are a bunch of ROS packages that need to be identified as explicit dependencies so they can be auto-installed.

If the build fails and then the respective ROS package/s (dependencies) must be manually installed as needed. When the compilation fails, scroll up the terminal until the first error is seen(red text). If it says something like  `cannot find industrial_coreConfig.cmake`, it means that is a missing ROS package. Run 

	`sudo apt-get install ros-kinetic-package-name`

to fix, where package-name is whatever appears in the error (e.g., "industrial_core"). Note the switch from underscores to hyphens from the ROS package name to the Ubuntu package name.


## Compile the Project

With an open terminal anywhere under the root directory of the workspace, run catkin build. Fix errors about any missing system dependencies that are not ROS packages by:

    `catkin clean`

Note:  If robotiq/robot_s_mode_control gives problems. Try the following:

    `cd src/robotiq/robot_s_model_control`
    `catkin build --this`
    `catkin build`


## Setup ROS environment

For the system to automatically point to this project's workspace and if the other ROS workspaces are not being used:

Add `source ~/path/to/workspace/devel/setup.bash` to the bottom of the `.bashrc` file inside the home directory.

Otherwise, the environment setup script has to be run in every new terminal:

Inside the workspace directory, run  `. devel/setup.bash` .

To check that the current terminal environment is correct, run `roscd` and see if it brings to the workspace directory.


## Check Environment
In a new terminal with an environment pointing to the project workspace (i.e. with the project's setup.bash file sources as discussed above), run one of the following:

	`roslaunch ceccrebot_demo_support single_robot_demo.launch`
    `roslaunch ceccrebot_demo_support multi_robot_demo.launch`

This launches a graphical environment to show the a (kinematically) simulated robot and an example workcell. If RViz starts up and a 3D visualization of a robot and other hardware is seen, everything is good to go.


## Setup for Integrating ROS with State Machine

At this point, ROS should have been installed with a workspace created. Inside the `src/` directory of that workspace there should be `ros_mtconnect_2`, `ros_mtconnect_2_demo`, `robotiq`.

For these instructions, the ROS workspace is assumed to be `~/catkin_workspace` . Replace, if different, `~/catkin_workspace` with the ROS workspace created.


**Fix hardcoded paths for your setup**

1. Open a new terminal

2. `gedit`

	`~/catkin_workspace/src/ros_mtconnect_2/mtconnect_bridge/src/mtconnect_bridge/__init__.py`
    
3. Change lines 8 and 9 to 
    
    `sys.path.append(os.path.join(os.getenv('HOME'), 'catkin_workspace/src/ros_mtconnect_2'))`
    
    `sys.path.append(os.path.join(os.getenv('HOME'), 'catkin_workspace/src/ros_mtconnect_2/simulator/src'))`

4. Save and close
   
5. `gedit`  
    
    `/catkin_workspace/src/ros_mtconnect_2_demo/ceccrebot_demo/src/nodes/mtconnect_demo.py`

6. Change line 124 to
    
	`os.chdir(os.path.join(os.getenv("HOME"), "catkin_workspace/src/ros_mtconnect_2/simulator/src"))`
	
7. Save and close


**Create Python Environment**

Some of these steps may not be necessary depending on what software is already installed on the computer. 

1. Open a terminal

2. `sudo apt install python-pip`

3. `pip install pipenv`
    
4. `cd ~/catkin_workspace/src/ros_mtconnect_2/simulator`
    
	There should be a `Pipfile` here

5. `pipenv shell`
    
	This will launch the pip environment in this terminal
	
6. `pip install transitions doublex requests mock`

7. Open a new terminal and in that terminal type:
    
    `killall pipenv`
    
8. `gedit`
	`~/.bashrc`

9. Add this line to the bottom: 
    
    `export PYTHONPATH=\$PYTHONPATH:/usr/lib/python2.7/dist-packages`


**Build the Agent**

1. Open a terminal and enter these commands

2. `sudo apt install libcppunit-dev`

3. `cd ~`

4. `git clone https://github.com/mtconnect/cppagent.git`

5. `cd ~/cppagent`

6. `mkdir build`
    
7. `cd build`

8. `cmake ..`

9. `make`

10. `cd ~/catkin_workspace/src/ros_mtconnect_2/simulator/src/deviceFiles`

11. `~/cppagent/build/agent/agent`

12. Open a web browser and navigate to `localhost:5000`  to verify that the agent is running


## Launching the Simulation

For launching live demo, two terminals will be neeeded. Both should have the ROS environment sourced as discussed in the setup instructions (`source ~/your/workspace/devel/setup.bash`). Note that since this script is written for the industrial PC associated with the demo, some changes to source files may have to be made as well. See `commit d0b067 - Local changes for startup script for details`.

1. Launch gripper. There are permissions issues with this. It must be launched with root privileges 
    
    `sudo su --preserve-environment`
    
    `roslaunch ceccrebot_demo_support robotiq_gripper.launch

2. Launch startup script
    
    `cd ~/catkin_workspace/src/ros_mtconnect_2/simulator/src`

    `sh startup.sh`

3. (Optional) To restart, `ctrl-c` both terminals and restart.

4. (Optional) Instead of using `startup.sh`, each `xterm` command within the file can be started in a separate terminals.
