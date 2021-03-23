
# Simulating the Robotic Arm Using software codes in RVIZ
 In this exercise, we are going to write a python script to simulate the Robotic arm that we built in the last tutorial. Previously, we used  ROS inbuilt joint_state_publisher (http://wiki.ros.org/joint_state_publisher) package with GUI joint controller to simulate the robot. Now let's do the same with our own package.

First Let's create a ros package to define the URDF definitions.

1. Read the `robot_state_publisher` documentation (http://wiki.ros.org/robot_state_publisher). This is the main package that read the URDF definitions from the URDF file and converts it to `tf`        messages.  To get the joint positions (joints angles), it is subscribing to the `joint_state` topic. It has the message type of `JointState` (http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html). Read the documentation for the `JointState` message type. We need to publish the joint angles as an array. 



    
2. Let's write a python script to send joint angles to our robot. Creat  a `joint_publisher.py` script inside the `src` folder of the `serial_link_robot` package.See how I have assigned each component of the joit_state_msg. The joint name should be the same as how you defined each link in the URDF file.  The `joint_state_msg.position` variable  assigns the angle values in radians.

    ```
    #!/usr/bin/env python
    import rospy

    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header


    def publish_joint_angles():

        rospy.init_node('robot_joint_publisher', anonymous=True)
        joint_publisher =rospy.Publisher('/joint_states',JointState,queue_size=10)
        joint_state_msg = JointState()
        
        # this is the rate of publishing. Need to have rate.sleep() command inside the while loop
        rate = rospy.Rate(10) # 10hz

        joint_state_msg.header = Header()    
        joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3']
        # angles are in radians
        joint_state_msg.position = [0.5, 0.5, 1.2]
        joint_state_msg.velocity = []
        joint_state_msg.effort = []

        # this while loop will continuously publish the joint states
        while not rospy.is_shutdown():

           joint_state_msg.header.stamp = rospy.Time.now()
           joint_publisher.publish(joint_state_msg)
           # this sleep command will pause the program accordin to the rate defined above
           rate.sleep()

    if __name__ == '__main__':
        try:
            #publish joint angles by calling the main function
            publish_joint_angles()
        except rospy.ROSInterruptException: pass


    ```
  
3. Remember to make the script an executable by using the `chmod` command.
  
4. Now let's create a new launch file to launch the RVIZ simulator, robot_state_publisher. We can modify the launch file that we used in the previous tutorial. Simply remove the commands to launch the `robot_joint_publisher` package. Instead, we are going to run our script to publish joint angles. Create a new launch file with the name `joint_publisher.launch`.


    ```
    <launch>
        <!-- values passed by command line input -->
        <arg name="model" />
       

      <!-- set these parameters on Parameter Server -->
      <param name="robot_description"
      textfile="$(find serial_link_robot)/urdf/$(arg model)"
      />


      <!-- Start 2 nodes: 
        robot_state_publisher and rviz -->



      <node name="robot_state_publisher"
        pkg="robot_state_publisher"
        type="state_publisher" />


      <node name="rviz" pkg="rviz" type="rviz"
        args="-d $(find serial_link_robot)/urdf.rviz"
        required="true" />
      </launch>

    ```

5. Now you can run the launch file by running the roslaunch command in the terminal.

    ***roslaunch serial_link_robot joint_publisher.launch model:=serial_link_robot.urdf***

6. Rviz will show some error messages as joint angles are not yet publishing.


7. Now let's run the script that we created in a new terminal. This will publish the joint angles that we coded in our script. In RVIZ you will see now that the robot has moved to the positions that we commanded using our script.
 ```
 rosrun serial_link_robot joint_publisher.py

```

![](../../_static/rviz2.png)


8. Now try to understand the following script that publishes random joint angles. It will make our robot to do some dance moves.


```
#!/usr/bin/env python
import rospy

import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# generate random floating point values
from random import seed
from random import random
# seed random number generator
seed(1)


def publish_joint_angles():

    rospy.init_node('robot_joint_publisher', anonymous=True)
    joint_publisher =rospy.Publisher('/joint_states',JointState,queue_size=10)
    joint_state_msg = JointState()

    rate = rospy.Rate(10) # 10hz

    joint_state_msg.header = Header()    
    joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3']
    # angles are in radians
    joint_state_msg.position = [random()*2*math.pi, random()*2*math.pi, random()*2*math.pi]
    joint_state_msg.velocity = []
    joint_state_msg.effort = []

    while not rospy.is_shutdown():

       joint_state_msg.header.stamp = rospy.Time.now()
       joint_state_msg.position = [random(), random(), random()]
       joint_publisher.publish(joint_state_msg)
       rate.sleep()

if __name__ == '__main__':
    try:
        #publish joint angles
        publish_joint_angles()
    except rospy.ROSInterruptException: pass






```









 
