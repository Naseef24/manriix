#include <ros.h>
#include <sensor_msgs/Range.h>

// ROS Setup on Jetson AGX (Noetic)>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // cd ~/inventa_sim/src
    // catkin_create_pkg inventa_sonar std_msgs sensor_msgs rosserial_arduino roscpp

    // mkdir -p ~/inventa_sim/src/inventa_sonar/firmware/sonar_controller
    
// Install rosserial:
//   sudo apt-get update
//   sudo apt-get install ros-noetic-rosserial-arduino ros-noetic-rosserial

// Add in main launch file:>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
      // <launch>
      //   <!-- Arduino serial connection -->
      //   <node name="serial_node" pkg="rosserial_python" type="serial_node.py" output="screen">
      //     <param name="port" value="/dev/ttyACM0"/>
      //     <param name="baud" value="57600"/>
      //   </node>
        
      //   <!-- Load robot description -->
      //   <param name="robot_description" command="$(find xacro)/xacro $(find inventa_description)/urdf/robot.urdf.xacro ultrasonic_enable:=true" />
        
      //   <!-- Launch robot state publisher for TF -->
      //   <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
      // </launch>

// ROS node handle
ros::NodeHandle nh;

// Range message publishers
sensor_msgs::Range range_msg_1;
sensor_msgs::Range range_msg_2;
sensor_msgs::Range range_msg_3;
sensor_msgs::Range range_msg_4;
sensor_msgs::Range range_msg_5;
sensor_msgs::Range range_msg_6;

// Publishers
ros::Publisher pub_range_1("sonar_back_rt", &range_msg_1);
ros::Publisher pub_range_2("sonar_front_rt45", &range_msg_2);
ros::Publisher pub_range_3("sonar_front", &range_msg_3);
ros::Publisher pub_range_4("sonar_front_lt45", &range_msg_4);
ros::Publisher pub_range_5("sonar_back_lt", &range_msg_5);
ros::Publisher pub_range_6("sonar_back_lt45", &range_msg_6);

// HC-SR04 Pin definitions
#define TRIG_PIN_1 22
#define ECHO_PIN_1 23
#define TRIG_PIN_2 24
#define ECHO_PIN_2 25
#define TRIG_PIN_3 26
#define ECHO_PIN_3 27
#define TRIG_PIN_4 28
#define ECHO_PIN_4 29
#define TRIG_PIN_5 30
#define ECHO_PIN_5 31
#define TRIG_PIN_6 32
#define ECHO_PIN_6 33

// Sensor frames
char frame_id_1[] = "/back_rt_sonar_1_link";
char frame_id_2[] = "/front_rt45_sonar_2_link";
char frame_id_3[] = "/front_sonar_3_link";
char frame_id_4[] = "/front_lt45_sonar_4_link";
char frame_id_5[] = "/back_lt_sonar_5_link";
char frame_id_6[] = "/back_lt45_sonar_6_link";

// Function to read HC-SR04 sensor
float readUltrasonic(int trigPin, int echoPin) {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure echo time
  float duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
  
  // Calculate distance in meters
  float distance = duration * 0.000343 / 2.0; // Speed of sound = 343 m/s
  
  // Handle out of range or timeout (0 return from pulseIn)
  if (distance == 0 || distance > 4.0) {
    distance = 4.0; // Max range
  }
  
  return distance;
}

void setup() {
  // Initialize ROS node
  nh.initNode();
  
  // Advertise publishers
  nh.advertise(pub_range_1);
  nh.advertise(pub_range_2);
  nh.advertise(pub_range_3);
  nh.advertise(pub_range_4);
  nh.advertise(pub_range_5);
  nh.advertise(pub_range_6);
  
  // Setup ultrasonic pins
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);
  pinMode(TRIG_PIN_4, OUTPUT);
  pinMode(ECHO_PIN_4, INPUT);
  pinMode(TRIG_PIN_5, OUTPUT);
  pinMode(ECHO_PIN_5, INPUT);
  pinMode(TRIG_PIN_6, OUTPUT);
  pinMode(ECHO_PIN_6, INPUT);
  
  // Configure range messages
  range_msg_1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_1.field_of_view = 0.26; // ~15 degrees in radians
  range_msg_1.min_range = 0.02;     // 2cm
  range_msg_1.max_range = 4.0;      // 4m
  range_msg_1.header.frame_id = frame_id_1;
  
  range_msg_2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_2.field_of_view = 0.26;
  range_msg_2.min_range = 0.02;
  range_msg_2.max_range = 4.0;
  range_msg_2.header.frame_id = frame_id_2;
  
  range_msg_3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_3.field_of_view = 0.26;
  range_msg_3.min_range = 0.02;
  range_msg_3.max_range = 4.0;
  range_msg_3.header.frame_id = frame_id_3;
  
  range_msg_4.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_4.field_of_view = 0.26;
  range_msg_4.min_range = 0.02;
  range_msg_4.max_range = 4.0;
  range_msg_4.header.frame_id = frame_id_4;
  
  range_msg_5.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_5.field_of_view = 0.26;
  range_msg_5.min_range = 0.02;
  range_msg_5.max_range = 4.0;
  range_msg_5.header.frame_id = frame_id_5;
  
  range_msg_6.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_6.field_of_view = 0.26;
  range_msg_6.min_range = 0.02;
  range_msg_6.max_range = 4.0;
  range_msg_6.header.frame_id = frame_id_6;
}

void loop() {
  // Update current time
  static unsigned long last_time = 0;
  unsigned long current_time = millis();
  
  // Read and publish every 50ms (20Hz)
  if (current_time - last_time > 50) {
    last_time = current_time;
    
    // Update timestamps
    range_msg_1.header.stamp = nh.now();
    range_msg_2.header.stamp = nh.now();
    range_msg_3.header.stamp = nh.now();
    range_msg_4.header.stamp = nh.now();
    range_msg_5.header.stamp = nh.now();
    range_msg_6.header.stamp = nh.now();
    
    // Read sensors and publish with small delays between readings
    // to avoid signal interference between adjacent sensors
    
    // Sensor 1
    range_msg_1.range = readUltrasonic(TRIG_PIN_1, ECHO_PIN_1);
    pub_range_1.publish(&range_msg_1);
    delay(5);
    
    // Sensor 2
    range_msg_2.range = readUltrasonic(TRIG_PIN_2, ECHO_PIN_2);
    pub_range_2.publish(&range_msg_2);
    delay(5);
    
    // Sensor 3
    range_msg_3.range = readUltrasonic(TRIG_PIN_3, ECHO_PIN_3);
    pub_range_3.publish(&range_msg_3);
    delay(5);
    
    // Sensor 4
    range_msg_4.range = readUltrasonic(TRIG_PIN_4, ECHO_PIN_4);
    pub_range_4.publish(&range_msg_4);
    delay(5);
    
    // Sensor 5
    range_msg_5.range = readUltrasonic(TRIG_PIN_5, ECHO_PIN_5);
    pub_range_5.publish(&range_msg_5);
    delay(5);
    
    // Sensor 6
    range_msg_6.range = readUltrasonic(TRIG_PIN_6, ECHO_PIN_6);
    pub_range_6.publish(&range_msg_6);
  }
  
  nh.spinOnce();
}