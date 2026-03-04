Hardware Setup (Same as Before)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
Components Needed:

        - 6× HC-SR04 ultrasonic sensors
        - Arduino Mega (recommended for multiple sensors)
        - Jumper wires
        - 5V power supply for Arduino
        - USB cable to connect Arduino to Jetson

Connection Diagram:
        
        HC-SR04 #1 (back_rt):
        - VCC → 5V on Arduino
        - GND → GND on Arduino
        - Trig → Digital Pin 22
        - Echo → Digital Pin 23

        HC-SR04 #2 (front_rt45):
        - VCC → 5V on Arduino
        - GND → GND on Arduino
        - Trig → Digital Pin 24
        - Echo → Digital Pin 25

        HC-SR04 #3 (front):
        - VCC → 5V on Arduino
        - GND → GND on Arduino
        - Trig → Digital Pin 26
        - Echo → Digital Pin 27

        HC-SR04 #4 (front_lt45):
        - VCC → 5V on Arduino
        - GND → GND on Arduino
        - Trig → Digital Pin 28
        - Echo → Digital Pin 29

        HC-SR04 #5 (back_lt):
        - VCC → 5V on Arduino
        - GND → GND on Arduino
        - Trig → Digital Pin 30
        - Echo → Digital Pin 31

        HC-SR04 #6 (back_lt45):
        - VCC → 5V on Arduino
        - GND → GND on Arduino
        - Trig → Digital Pin 32
        - Echo → Digital Pin 33


cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .


Method 1: Using the Arduino IDE
+++++++++++++++++++++++++++++++
        Open the Arduino IDE
        Open the sketch at ~/inventa_sim/src/inventa_sonar/firmware/sonar_controller/sonar_controller.ino
        Select your board (Arduino Mega)
        Select the port
        Click Upload

Method 2: Using catkin   
++++++++++++++++++++++     
        cd ~/inventa_sim
        catkin_make inventa_sonar_sonar_controller-upload        


Run the System        
        roslaunch inventa_sonar inventa_with_sonar.launch

Monitoring the Sensors        
    # List all topics
            rostopic list | grep sonar

    # View data from one sensor
            rostopic echo /sonar_front

    # View all sensor data
            rostopic echo /sonar_back_rt
            rostopic echo /sonar_front_rt45
            rostopic echo /sonar_front
            rostopic echo /sonar_front_lt45
            rostopic echo /sonar_back_lt
            rostopic echo /sonar_back_lt45

Visualizing in RViz            
    rosrun rviz rviz
            
            -Set the Fixed Frame to base_link
            -Add a RobotModel display to see your robot
            -Add Range displays for each sonar sensor
            -Set the topics to each of your sonar topics
                 

Troubleshooting                  
+++++++++++++++    
    1. Arduino isn't detected:
            ls -l /dev/ttyACM*
            sudo chmod a+rw /dev/ttyACM0     

    2. Port issues:

            Edit the port in the launch file if Arduino is on a different port


    3. Frame ID errors:

            Make sure the frame IDs in the Arduino code match your URDF


    4. No readings:

            Check wiring connections
            Verify Arduino is getting power
            Try a simple Arduino sketch to test each sensor independently


    5. Interference between sensors:

            Increase the delay between sensor readings in the Arduino code            
