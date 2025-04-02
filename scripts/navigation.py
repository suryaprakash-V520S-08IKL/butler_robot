#!/usr/bin/env python3

import rospy
from butler_robot.msg import Order
from std_msgs.msg import String  # ✅ Import String to fix NameError

# Global variables
confirmation_received = False
confirmation_timeout = False
table_number = None
confirmation_timer = None  # Store the timer globally

def move_robot(destination):
    rospy.loginfo(f"🚗 Moving to {destination}...")
    rospy.sleep(2)
    rospy.loginfo(f"✅ Reached {destination}")

def wait_for_confirmation_timer(event):
    global confirmation_timeout, confirmation_received, confirmation_timer

    if confirmation_received:
        return

    confirmation_timeout = True
    rospy.logwarn(f"⚠️ No confirmation received for Table {table_number}. Returning to Kitchen.")
    move_robot("Kitchen")
    move_robot("Home")
    rospy.loginfo(f"✅ Order for Table {table_number} completed!")

def order_callback(order_msg):
    global confirmation_received, confirmation_timeout, table_number, confirmation_timer
    
    table_number = order_msg.table_number

    if order_msg.status == "received":
        rospy.loginfo(f"📦 New Order: Deliver to Table {table_number}")

        move_robot("Kitchen")
        move_robot(f"Table {table_number}")

        confirmation_received = False
        confirmation_timeout = False

        if confirmation_timer:
            confirmation_timer.shutdown()
            rospy.loginfo("⏳ Previous timer canceled.")

        confirmation_timer = rospy.Timer(rospy.Duration(30), wait_for_confirmation_timer, oneshot=True)
        rospy.loginfo("⏳ New confirmation timer started.")

        while not confirmation_received and not confirmation_timeout:
            rospy.sleep(0.1)

        if confirmation_received:
            rospy.loginfo(f"✅ Order for Table {table_number} is confirmed!")
        elif confirmation_timeout:
            rospy.logwarn(f"⚠️ No confirmation received for Table {table_number}. Returning to Kitchen.")

def confirmation_callback(msg):
    global confirmation_received, confirmation_timer
    
    if msg.data == f"Table {table_number} confirmed":
        confirmation_received = True
        rospy.loginfo(f"✅ Order for Table {table_number} has been confirmed as delivered!")

        if confirmation_timer:
            confirmation_timer.shutdown()
            rospy.loginfo("⏳ Confirmation timer canceled.")

def navigation_node():
    rospy.init_node('navigation_node', anonymous=True)

    rospy.Subscriber("/order_request", Order, order_callback)
    rospy.Subscriber("/order_confirmation", String, confirmation_callback)  # ✅ Fixed import issue

    rospy.loginfo("🟢 Navigation Node is running...")
    rospy.spin()

if __name__ == "__main__":
    try:
        navigation_node()
    except rospy.ROSInterruptException:
        pass

