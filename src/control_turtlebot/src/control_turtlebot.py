import rospy
from geometry_msgs.msg import Twist

class ControlTurtlebot():
    def __init__(self):
        # initiliaze
        rospy.init_node('ControlTurtlebot', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);

        go_forward_twist = Twist()
        go_forward_twist.linear.x = 0.2

        turn_twist = Twist()
        turn_twist.angular.z = 1.2

        while not rospy.is_shutdown():
            start_time = rospy.get_rostime()
            while rospy.get_rostime().secs - start_time.secs <= 5:
                print 'going forward; current duration: {0} s'.format(
                    rospy.get_rostime().secs - start_time.secs)
                # go forward
                self.cmd_vel_pub.publish(go_forward_twist)
                r.sleep()
            start_time = rospy.get_rostime()
            while rospy.get_rostime().secs - start_time.secs <= 3:
                print 'turning; current duration: {0} s'.format(
                    rospy.get_rostime().secs - start_time.secs)
                # turn
                self.cmd_vel_pub.publish(turn_twist)
                r.sleep()
                        
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel_pub.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        ControlTurtlebot()
    except:
        rospy.loginfo("GoForward node terminated.")
