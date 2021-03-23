
# Transforming End effector position to the base_link
 In this exercise, we are going to write a python code to find the position of the End effector w.r.t to the base_link. This is a part of the assignmnet 1 , Q2 requirement. I am not going to provide the complete code but I will provide you with the code skeleton. You need to add a few extra lines to complete the Q2 requirements.



1. First we need to add extra joint (joint_ee) and a link (link_ee) to assign a coordinate frame to the end-effector position. Rememberto change the `Joint_ee` orgin according to the assignment requirements. Add  the following xml code to the urdf file before the  `</robot>` tag.

    ```
    <!-- Joint ee -->
     <joint name="joint_ee" type="fixed">
        <parent link="link_3"/>
        <child link="link_ee"/>
        <origin xyz="0 0.6 0" rpy="0 0 0" />
        
      </joint>

    <!-- Link ee -->
    <link name="link_ee">
        
      </link>


    ```

2. You can use the following code skeleton to do the transformation from the `link_ee` to `base_link`. Look at the transformation listner tutorial that we did in week 2. Complete the missing commands.



    ```
    #!/usr/bin/env python
    import rospy

    import math
    import tf
    import geometry_msgs.msg



    def tf_listner():

        rospy.init_node('robot_tf_listner')
        listener = tf.TransformListener()

        rate = rospy.Rate(10) # 10hz

        
        while not rospy.is_shutdown():

           # complet the missing command to create the Transform listner from link_ee to base_link. 
           # You need to then print the translation component to the terminal.
           
            rate.sleep()

    if __name__ == '__main__':
        try:
            #publish joint angles
            tf_listner()
        except rospy.ROSInterruptException: pass


    ```
  
3. Name the script as `robot_transform.py` and run it (make it execeutable first). Remember to launch the launch file to run the RVIZ and the robot_state_publisher.

4. Watch the live or the recoded class for more information
  




