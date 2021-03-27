See the following python script.

```

#!/usr/bin/env python
import math
import rospy
import turtlesim.srv
from geometry_msgs.msg import Twist

rospy.init_node('robot_letter', anonymous=True)
velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
speed = 1
angular_velocity= 1


# Initial velocity values for the publisher
vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0

def move():
      
    
        
    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    move_straight(2,t0)


    t0 = rospy.Time.now().to_sec()
    rotate_angle(math.pi/2, t0)

    t0 = rospy.Time.now().to_sec()
    move_straight(3,t0)

   
    t0 = rospy.Time.now().to_sec()
    move_circle(math.pi,t0)
    
    
    #create a new robot
    create_second_robot()


# a function to move the robot forward
def move_straight(move_distance,start_time):
    vel_msg.linear.x = speed
    vel_msg.angular.z = 0 
    current_distance = 0
    #Loop to move the turtle in an specified distance
    while(current_distance < move_distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            current_time=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(current_time-start_time) 
            print("curent distance %d",current_distance)
    stop_robot()

#a function to roate the robot  angle
def rotate_angle(rotate_angle,start_time):
    vel_msg.linear.x = 0
    vel_msg.angular.z = angular_velocity    
    current_angle = 0
    #Loop to move the turtle in an specified angle
    while(current_angle < rotate_angle):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            current_time=rospy.Time.now().to_sec()
            #Calculates angle PoseStamped
            current_angle= angular_velocity*(current_time-start_time) 
            print("curent distance %d",current_angle)
    stop_robot()

# force the robot to stop
def stop_robot():
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

#move the robot in a curvey path
def move_circle(rotate_angle,start_time):
    vel_msg.linear.x = 1
    vel_msg.angular.z = angular_velocity    
    current_angle = 0
    #Loop to move the turtle in an specified angle
    while(current_angle < rotate_angle):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            current_time=rospy.Time.now().to_sec()
            #Calculates angle PoseStamped
            current_angle= angular_velocity*(current_time-start_time) 
            print("curent distance %d",current_angle)
    stop_robot()

#create the second turtlebot
def create_second_robot():
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2') 

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass

```
