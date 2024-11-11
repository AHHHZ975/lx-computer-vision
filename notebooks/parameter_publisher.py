import rospy
from std_msgs.msg import String

def send_manual_parameter_tuning():
    # Initialize the ROS node
    rospy.init_node('manual_parameter_sender', anonymous=True)

    # Replace <robot_namespace> with your actual robot's namespace, such as "duckiebot"
    topic_name = '/{self.veh}/manual_parameter_tuning' 
    pub = rospy.Publisher(topic_name, String, queue_size=1)

    # Wait a moment to ensure the node is fully initialized
    rospy.sleep(1)

    # Send calibration values (example values: "1.0,1.0,0.3,1")
    values = "0.6,0.9,0,0"  # Replace with actual calibration values as needed
    rospy.loginfo(f"Sending parameters values: {values}")
    pub.publish(values)
    
    rospy.loginfo("Calibration values sent.")
    rospy.sleep(1)  # Give time for the message to be sent

send_manual_parameter_tuning()

