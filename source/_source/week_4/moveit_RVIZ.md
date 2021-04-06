
# MoveIt Quickstart in RViz



The quickest way to get started using MoveIt is through its RViz plugin. Rviz is the primary visualizer in ROS and an incredibly useful tool for debugging robotics. The MoveIt Rviz plugin allows you to setup virtual environments (scenes), create start and goal states for the robot interactively, test various motion planners, and visualize the output.



### Step 1: Launch the Demo and Configure the Plugin
------------------------------------------------
* First go to your `` catKin_ws `` and run ``catki_make ``.

* Also run `` rospack profile `` command 


* Launch the demo: ::

   `` roslaunch moveit_arm demo.launch rviz_tutorial:=true``

* If you are doing this for the first time, you might see an empty world in RViz and will have to add the Motion Planning Plugin. If you can see the robot then you can skip the following steps

  * You should see an empty world in RViz:

  * In the RViz Displays Tab, press *Add*:

  * From the moveit_ros_visualization folder, choose "MotionPlanning" as the DisplayType. Press "Ok".

  * You should now see the robotic arm  in RViz:

  |D|

![](../../_static/move_it_rviz/move_It_1.png)

* Once you have the Motion Planning Plugin loaded, we can configure it. In the "Global Options" tab of the "Displays" subwindow, set the **Fixed Frame** field to ``/base_link``

* Now, you can start configuring the Plugin for your robot . Click on "MotionPlanning" within "Displays".

  * Make sure the **Robot Description** field is set to ``robot_description``.

  * Make sure the **Planning Scene Topic** field is set to ``/planning_scene``.
    Click on topic name to expose topic-name drop-down.

  * In **Planning Request**, change the **Planning Group** to ``arm``.

  * In **Planned Path**, change the **Trajectory Topic** to ``/move_group/display_planned_path``.

![](../../_static/move_it_rviz/move_It_2.png)

![](../../_static/move_it_rviz/move_It_3.png)

### Step 2: Play with the Visualized Robots


There are four different overlapping visualizations:

* The robot's configuration in the ``/planning scene`` planning environment (active by default).

* The planned path for the robot (active by default).

* Green: The start state for motion planning (disabled by default).

* Orange: The goal state for motion planning (active by default).

The display states for each of these visualizations can be toggled on and off using checkboxes:

* The planning scene robot using the **Show Robot Visual** checkbox in the **Scene Robot** tab.

* The planned path using the **Show Robot Visual** checkbox in the **Planned Path** tab.

* The start state using the **Query Start State** checkbox in the **Planning Request** tab.

* The goal state using the **Query Goal State** checkbox in the **Planning Request** tab.

* Play with all these checkboxes to switch on and off different visualizations.

![](../../_static/move_it_rviz/move_It_4.png)

### Step 3: Interact with the Robot


For the next steps we will want only the scene robot, start state and goal state:

* Check the **Show Robot Visual** checkbox in the **Planned Path** tab

* Un-check the **Show Robot Visual** checkbox in the **Scene Robot** tab

* Check the **Query Goal State** checkbox in the **Planning Request** tab.

* Check the **Query Start State** checkbox in the **Planning Request** tab.

There should now be two interactive markers. One marker corresponding to the orange colored arm will be used to set the "Goal State" for motion planning and the other marker corresponding to a green colored arm are used to set the "Start State" for motion planning. If you don't see the interactive markers press **Interact** in the top menu of RViz (Note: some tools may be hidden, press **"+"** in the top menu to add the **Interact** tool as shown below).



You should now be able to use these markers to drag the arm around and change its orientation. Try it!



![](../../_static/move_it_rviz/move_It_4.png)


### Moving into collision

Note what happens when you try to move one of the arms into collision with the other. The two links that are in collision will turn red.

![](../../_static/move_it_rviz/move_It_5.png)

The "Use Collision-Aware IK" checkbox found within the MotionPlanning plugin under the Planning tab allows you to toggle the behavior of the IK solver. When the checkbox is ticked, the solver will keep attempting to find a collision-free solution for the desired end-effector pose. When it is not checked, the solver will allow collisions to happen in the solution. The links in collision will always still be visualized in red, regardless of the state of the checkbox.



### Moving out of Reachable Workspace

Note what happens when you try to move an end-effector out of its reachable workspace.

![](../../_static/move_it_rviz/move_It_6.png)

### Step 4: Use Motion Planning with the Robot


* Now, you can start motion planning with the Robtic arm  in the MoveIt RViz Plugin.

  * Move the Start State to a desired location.

  * Move the Goal State to another desired location.

  * Make sure both states are not in collision with the robot itself.

  * Make sure the Planned Path is being visualized. Also check the
    **Show Trail** checkbox in the **Planned Path** tab.

  * In the **MotionPlanning** window under the **Planning** tab, press the **Plan** button. You
  should be able to see a visualization of the arm moving and a trail.


![](../../_static/move_it_rviz/move_It_7.png)

### Step 5: Introspecting Trajectory Waypoints


You can visually introspect trajectory point by point on RViz.

* From "`Panels`" menu, select "`MotionPlanning - Slider`". You'll see a new Slider panel on RViz.

* Set your goal pose, then run `Plan`.

* Play with the "`Slider`" panel, e.g. move the slider, push "`Play`" button.

NOTE: Once you placed your EEF to a new goal, be sure to run `Plan` before running `Play` -- otherwise you'll see the waypoints for the previous goal if available.


![](../../_static/move_it_rviz/move_It_8.png)




### Step 6: Saving Your Configuration

RViz enables you to save your configuration under ``File->Save Config``. You should do this before continuing on to the next tutorials.



### License

This tutorial is modified from the original content from

 http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html 

with BSD 3 License.

BSD 3-Clause License

  Copyright (c) 2008-2013, Willow Garage, Inc.
  Copyright (c) 2015-2019, PickNik, LLC.
  All rights reserved.



