#!/usr/bin/env python3 

"""
Description:
Publish the battery state at a specific time interval
-------
Publishing Topics:
/battery_status – sensor_msgs/BatteryState
-------
Subscription Topics:
None
-------
"""
 
import rospy
from sensor_msgs.msg import BatteryState
 
class BatteryStatePublisher:
    """
    Publish the battery state of an object at a specific time interval.
    Simulates battery discharge over time.
    """
    
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initialize the ROS node
        rospy.init_node('battery_state_pub')
        
        # Create publisher for battery state
        # Maximum queue size of 10 
        self.publisher_battery_state = rospy.Publisher(
            '/battery_status', 
            BatteryState, 
            queue_size=10
        )
        
        # Time interval in seconds
        timer_period = rospy.Duration(5.0)
        self.timer = rospy.Timer(timer_period, self.get_battery_state)
        
        # Initialize battery parameters
        self.battery_voltage = 9.0  # Initial battery voltage
        self.percent_charge_level = 1.0  # Initial percentage charge level
        self.decrement_factor = 0.99  # Used to reduce battery level each cycle
        
        # Additional status tracking
        self.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
     
    def get_battery_state(self, event=None):
        """
        Callback function to simulate battery discharge.
        Gets called at the specific time interval.
        """
        # Prepare battery state message
        msg = BatteryState()
        msg.voltage = self.battery_voltage 
        msg.percentage = self.percent_charge_level
        msg.power_supply_status = self.power_supply_status
        
        # Publish BatteryState message 
        self.publisher_battery_state.publish(msg)
        
        # Log battery status
        rospy.loginfo(f"🔋 Battery Status: "
                      f"Voltage={self.battery_voltage:.2f}V, "
                      f"Charge={self.percent_charge_level * 100:.2f}%")
        
        # Decrement the battery state 
        self.battery_voltage *= self.decrement_factor
        self.percent_charge_level *= self.decrement_factor
        
        # Check for low battery condition
        if self.percent_charge_level < 0.25:
            rospy.logwarn("⚠️ Low Battery Warning! Charge below 25%")
            self.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_LOW
        
        # Stop publishing when battery is critically low
        if self.percent_charge_level < 0.1:
            rospy.logerr("❌ Battery Critically Low! Stopping battery state publisher.")
            self.timer.shutdown()
   
def main():
    try:
        # Create the battery state publisher node
        battery_state_pub = BatteryStatePublisher()
        
        # Spin to keep the node alive
        rospy.spin()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Battery state publisher node interrupted")

if __name__ == '__main__':
    main()
    
    
    
# #!/usr/bin/env python3 

# """
# Description:
# Publish the battery state at a specific time interval
# -------
# Publishing Topics:
# /battery_status – sensor_msgs/BatteryState
# -------
# Subscription Topics:
# None
# -------
# Author: Addison Sears-Collins
# Website: AutomaticAddison.com
# Date: November 10, 2021
# """
 
# import rclpy # Import the ROS client library for Python
# from rclpy.node import Node # Enables the use of rclpy's Node class
# from sensor_msgs.msg import BatteryState # Enable use of the sensor_msgs/BatteryState message type
 
# class BatteryStatePublisher(Node):
#   """
#   Create a BatteryStatePublisher class, which is a subclass of the Node class.
#   The class publishes the battery state of an object at a specific time interval.
#   """
  
#   def __init__(self):
#     """
#     Class constructor to set up the node
#     """
   
#     # Initiate the Node class's constructor and give it a name
#     super().__init__('battery_state_pub')
     
#     # Create publisher(s)  
     
#     # This node publishes the state of the battery.
#     # Maximum queue size of 10. 
#     self.publisher_battery_state = self.create_publisher(BatteryState, '/battery_status', 10)
     
#     # Time interval in seconds
#     timer_period = 5.0 
#     self.timer = self.create_timer(timer_period, self.get_battery_state)
    
#     # Initialize battery level
#     self.battery_voltage = 9.0 # Initialize the battery voltage level
#     self.percent_charge_level = 1.0  # Initialize the percentage charge level
#     self.decrement_factor = 0.99 # Used to reduce battery level each cycle
     
#   def get_battery_state(self):
#     """
#     Callback function.
#     This function gets called at the specific time interval.
#     We decrement the battery charge level to simulate a real-world battery.
#     """
#     msg = BatteryState() # Create a message of this type 
#     msg.voltage = self.battery_voltage 
#     msg.percentage = self.percent_charge_level
#     self.publisher_battery_state.publish(msg) # Publish BatteryState message 
     
#     # Decrement the battery state 
#     self.battery_voltage = self.battery_voltage * self.decrement_factor
#     self.percent_charge_level = self.percent_charge_level * self.decrement_factor
   
# def main(args=None):
 
#   # Initialize the rclpy library
#   rclpy.init(args=args)
 
#   # Create the node
#   battery_state_pub = BatteryStatePublisher()
 
#   # Spin the node so the callback function is called.
#   # Publish any pending messages to the topics.
#   rclpy.spin(battery_state_pub)
 
#   # Destroy the node explicitly
#   # (optional - otherwise it will be done automatically
#   # when the garbage collector destroys the node object)
#   battery_state_pub.destroy_node()
 
#   # Shutdown the ROS client library for Python
#   rclpy.shutdown()
 
# if __name__ == '__main__':
#   main()