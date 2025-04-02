#!/usr/bin/env python3

import rospy
from std_msgs.msg import String  # ✅ Expecting String type

confirmation_received = {}

def confirmation_callback(msg):
    """
    Callback function to confirm food delivery.
    """
    table_info = msg.data.split()  # Extract table number from "Table X confirmed"
    
    if len(table_info) < 2:
        rospy.logwarn(f"⚠️ Invalid confirmation message received: {msg.data}")
        return
    
    table_number = int(table_info[1])

    if table_number not in confirmation_received:
        rospy.loginfo(f"✅ Order for Table {table_number} has been confirmed as delivered!")
        confirmation_received[table_number] = True
    else:
        rospy.logwarn(f"⚠️ Confirmation for Table {table_number} was already processed.")

def confirmation_handler_node():
    """
    Initializes the confirmation handler node.
    """
    rospy.init_node('confirmation_handler', anonymous=True)
    rospy.loginfo("🚀 Starting Confirmation Handler Node...")
    
    rospy.Subscriber("/order_confirmation", String, confirmation_callback)
    rospy.loginfo("📡 Subscribed to /order_confirmation topic.")

    rospy.spin()

if __name__ == "__main__":
    try:
        confirmation_handler_node()
    except rospy.ROSInterruptException:
        rospy.logerr("❌ ROS node interrupted!")

