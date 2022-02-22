# arduino_encoder_mobile_robot
Navigation robot using arduino encoder
   
It is powered by ROS running on a Raspberry Pi 4B(ubuntu18.04, ROS melodic) and an Arduino mega that controls two motors with encoders
   
   
**Reference**
https://github.com/RBinsonB/Nox_robot   
https://www.youtube.com/watch?v=q1u_cC-5Sac&list=PLpwJoq86vov9U87KZG_x6w739Mlp07-BG&index=2   
https://www.youtube.com/watch?v=HLLmV9LQoac&list=PLpwJoq86vov9U87KZG_x6w739Mlp07-BG&index=3   
   
   
**Step1. Install ubuntu 18.04 on Raspberry Pi 4B**   
https://do-9un6.tistory.com/2   
   
   
**Step2. Install ROS melodic on Raspberry Pi 4B**   
https://do-9un6.tistory.com/4   
   
   
**Step3. Arduino**   
   
 Motor Driver | Arduino | Motor | External power
 -------------|---------|------ |----------------|
 AOUT | - | MotorA | -
 BOUT | - | MotorB | -
 PWMA | Pin6 | - | -
 AIN1 | Pin9 | - | -
 AIN2 | Pin8 | - | -
 PWMB | Pin7 | - | -
 BIN1 | Pin11 | - | -
 BIN2 | Pin12 | - | -
 5V | VIN | - | -
 GND | GND | - | -
 VSEL | - | - | VOUT
 | | GND | - | GND
 | | Pin2 | encoderA1 | -
 | | Pin3 | encoderA2 | -
 | | Pin18 | encoderB1 | -
 | | Pin19 | encoderB2 | -


    
```c
$ sudo apt-get install ros-melodic-rosserial
$ sudo apt-get install ros-melodic-rosserial-arduino
```
   
You can download Aruduino IDE for linux from here -> https://www.arduino.cc/en/software   
   
```c
$ cd /Downloads/arduino-1.8.18
$ sudo ./install.sh
$ sudo apt install fonts-nanurm-coding
$ sudo apt-get update
```
```c
$ cd libraries
$ rosrun rosserial_arduino make_libraries.py .
```
   
Add Arduino codes and libraries   
(You can find files from my github)   
   
Complie and upload [new_srduino_encoder05.ino] code to your Arduino mega   
   
   
**Step4. Running robot**   
```c
$ cd catkin_ws
$ catkin_make

// remote robot
$ roscore
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

// Slam & navigation
$ roslaunch nox nox_bringup.launch
$ roslaunch nox nox_slam.launch
fixed frame : map
click the 2D Nav Goal Button
Select a destination point
```
