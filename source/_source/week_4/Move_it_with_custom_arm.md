
# Using MoveIt Package for Robotic Arm Path Planning


In this tutorial we will creat and configure a robotic arm to use the Move_it ros package. Then we can use RVIZ to move the rootic arm to the desired location
 
Let's make sure that we  have the most up to date packages: ::
 ```
  rosdep update
  sudo apt-get update
  sudo apt-get dist-upgrade

 ```

Install MoveIt

The simplest way to install MoveIt is from pre-built binaries (Debian): ::
  
 ```
  sudo apt install ros-melodic-moveit

 ```

# Creating a Moveit package for a robotic arm

Now let's configure and a ROS move_it package for our robotic arm. Then we can use RVIZ to set goal locations and the move-it package will do inverse kinematics and path planning. 


* First download and save the URDF file that we created in last week to the home folder. You can download the urdf file from the moodle site.

  ## MoveIt Setup Assistant

  The MoveIt Setup Assistant is a graphical user interface for configuring any robot for use with MoveIt. Its primary function is generating a Semantic Robot Description Format (SRDF) file for your robot.  Additionally, it generates other necessary configuration files for use with the MoveIt pipeline. To learn more about the SRDF, you can go through the URDF/SRDF Overview page.

### Step 1: Start
  To start the MoveIt Setup Assistant:

  ```
  roslaunch moveit_setup_assistant setup_assistant.launch

  ````

  This will bring up the start screen with two choices: Create New MoveIt Configuration Package or Edit Existing MoveIt Configuration Package.

  Click on the Create New MoveIt Configuration Package button to bring up the following screen:


 Click on the browse button and navigate to the home folder and  choose the URDF file of your robotic arm  and then click Load Files. The Setup Assistant will load the files (this might take a few  minutes)


### Step 2: Generate Self-Collision Matrix


  The Default Self-Collision Matrix Generator searches for pairs of links on the robot that can safely be disabled from collision checking, decreasing motion planning processing time. These pairs of links  are disabled when they are always in collision, never in collision, in collision in the robot's default position or when the links are adjacent to each other on the kinematic chain. The sampling density specifies how many random robot positions to check for self collision.


  Click on the Self-Collisions pane selector on the left-hand side and click on the Generate Collision Matrix button. The Setup Assistant will work for a few second before presenting you the results of its computation in the main table.

### Srep 3 : Virtual Joints

Virtual joints are used primarily to attach the robot to the world. For our robot modela we will define only one virtual joint attaching the base_link of the Panda to the world world frame. This virtual joint represents the motion of the base of the robot in a plane.

  * Click on the Virtual Joints pane selector. Click on Add Virtual Joint
  * Set the joint name as "virtual_joint"
  * Set the child link as "base_link" and the parent frame name as "world".
  * Set the Joint Type as "fixed".

Click Save and you should see this screen:



### Step 4: Add Planning Groups
  Planning groups are used for semantically describing different parts of your robot, such as defining what an arm is, or an end effector.

  * Click on the Planning Groups pane selector.
  *Click on Add Group and you should see the following screen:



   Add the arm

  We will first add  as a planning group

  * Enter Group Name as `arm`
  * Choose kdl_kinematics_plugin/KDLKinematicsPlugin as the kinematics solver.
  * et Kin. Search Resolution and Kin. Search Timeout stay at their default values.

Now, click on the Add Joints button. You will see a list of joints on the left hand side. You need to choose all the joints that belong to the arm and add them to the right hand side. The joints are arranged in the order that they are stored in an internal tree structure. This makes it easy to select a serial chain of joints.


    Click on virtual_joint, hold down the Shift button on your keyboard and then click on the joint_ee. Now click on the > button to add these joints into the list of selected joints on the right.

  **picture


  Click Save to save the selected group.

  **picture

  #### Adding the gripper

  We will also add a group for the end effector. NOTE that you will do this using a different procedure than adding the arm.

  * Click on the Add Group button.
  * Enter Group Name as hand
  * Let Kin. Search Resolution and Kin. Search Timeout stay at their default values.
  * Click on the Add Links button.
  * Choose link_ee and add them to the list of Selected Links on the right hand side.
  * Click Save

### Step 5: Add Robot Poses

  The Setup Assistant allows you to add certain fixed poses into the configuration. This helps if, for example, you want to define a certain position of the robot as a Home position.

  * Click on the Robot Poses pane.
  * Click Add Pose. Choose a name for the pose. The robot will be in its Default position where the joint values are set to the mid-range of the allowed joint value range. Move the individual joints around until you are happy and then Save the pose. Note how poses are associated with particular groups. You can save individual poses for each group.
  * IMPORTANT TIP: Try to move all the joints around. If there is something wrong with the joint limits in your URDF, you should be able to see it immediately here

**picture


### Step 6: Label End Effectors
  We have already added the gripper of the Panda. Now, we will designate this group as a special group: end effectors. Designating this group as end effectors allows some special operations to happen on them internally.

  * Click on the End Effectors pane.
  * Click Add End Effector.
  * Choose hand as the End Effector Name for the gripper.
  * Select hand as the End Effector Group.
  * Select link_3 as the Parent Link for this end-effector.
  * Leave Parent Group blank.


### Step 7: Add Passive Joints
  The passive joints tab is meant to allow specification of any passive joints that might exist in a robot. These are joints that are unactuated on a robot (e.g. passive casters.) This tells the planners that they cannot (kinematically) plan for these joints because they can't be directly controlled. Our robotic arm does not have any passive joints so we will skip this step.




### Step 8: ROS Control
  ROS Control is a set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces, for more details please look at ros_control documentation

  ROS Control tab can be used to auto generate simulated controllers to actuate the joints of the robot. This will allow us to provide the correct ROS interfaces MoveIt.

  Click on the ROS Control pane selector.


***picture

  Click on Add Controller and you should see the following screen:


  * We will first add the arm position controller
  * Enter Controller Name as arm_position_controller
  * Choose position_controllers/JointPositionController as the controller type
  * Next you have to choose this controller joints, you can add joints individually or add all the joints in a planning group all together.
  * Now, click on Add Planning Group Joints.




*** picture


Choose panda_arm planning group to add all the joints in that group to the arm controller.


*** picture

### Step 9 - Simulation 

We will skip simulation tab. We will do gazebo simulation in a later stage

### Step 10 - 3D perception

We will skip 3D perception as we don't have a attached 3D sensor.

### Step 11: Generate Configuration Files
  You are almost there. One last step - generating all the configuration files that you will need to start using MoveIt

   We need to create a seperate folder inside our catkin_ws folder to save the move_it configuration. Using the UBUNTu GUI click  Files icon then go to 

    `catkin_ws` folder and go to the src folder and create a folder called `moveit_arm`.

  Now come back to move_it configuration (click on the blue arrow on the left hand side panel) and Click on the Configuration Files pane. Choose folder that we created by clicking on browse.

  Click on the Generate Package button. The Setup Assistant will now generate and write a set of launch and config files into the directory of your choosing. All the generated files will appear in the Generated Files/Folders tab and you can click on each of them for a description of what they contain.



  Congratulations!! - You are now done generating the configuration files you need for MoveIt. Now you can exit set up assistance.












