#!/usr/bin/env python3

import rospy
from butler_robot.msg import Order
from std_msgs.msg import String  # ✅ Correct message type

# Global set to track confirmed orders
confirmed_orders = set()

def order_confirmation_callback(msg):
    """Handles order confirmations received from the /order_confirmation topic."""
    rospy.loginfo(f"✅ Order confirmation message received: {msg.data}")

    # Convert message to lowercase and split into words
    words = msg.data.lower().split()
    
    # Check if both "confirmed" and "table" are present in the message
    if "confirmed" in words and "table" in words:
        try:
            table_index = words.index("table") + 1  # Get index after "table"
            if table_index < len(words):  # Ensure there's a number after "table"
                table_number = int(words[table_index])  # Convert to integer
                confirmed_orders.add(table_number)  # Add to confirmed set
                rospy.loginfo(f"✅ Order for Table {table_number} has been confirmed!")
            else:
                rospy.logwarn("⚠️ 'Table' found but no number after it.")
        except (ValueError, IndexError):
            rospy.logwarn("⚠️ Failed to extract a valid table number from the confirmation message.")
    else:
        rospy.logwarn("⚠️ Invalid confirmation message format. Expected format: 'confirmed table X'.")

def send_order():
    """Publishes an order request to the /order_request topic and listens for confirmations."""
    rospy.init_node('order_manager', anonymous=True)
    pub_request = rospy.Publisher('/order_request', Order, queue_size=10)
    pub_confirmation = rospy.Publisher('/order_confirmation', String, queue_size=10)  # ✅ Changed type to String
    rospy.Subscriber("/order_confirmation", String, order_confirmation_callback)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        try:
            table_number = int(input("Enter table number (1-3) or 0 to cancel: "))
            if table_number == 0:
                rospy.loginfo("Order canceled.")
                continue

            if table_number not in [1, 2, 3]:
                rospy.logwarn("Invalid table number! Please enter 1, 2, or 3.")
                continue

            if table_number in confirmed_orders:
                rospy.loginfo(f"Order for Table {table_number} has already been confirmed.")
                continue  # Skip if order is already confirmed

            # Publish order request
            order = Order()
            order.table_number = table_number
            order.status = "received"
            rospy.loginfo(f"📦 Sending order to Table {table_number}...")
            pub_request.publish(order)

            rospy.loginfo(f"Waiting for confirmation of order for Table {table_number}...")

            # Non-blocking waiting loop
            timeout = rospy.Time.now() + rospy.Duration(30)  # 30s timeout
            while table_number not in confirmed_orders and rospy.Time.now() < timeout:
                rospy.sleep(1)

            if table_number in confirmed_orders:
                rospy.loginfo(f"✅ Order for Table {table_number} is confirmed!")
            else:
                rospy.logwarn(f"⚠️ Order for Table {table_number} was not confirmed within 30s.")

            rate.sleep()

        except ValueError:
            rospy.logerr("Invalid input! Please enter a valid number.")

if __name__ == "__main__":
    send_order()
