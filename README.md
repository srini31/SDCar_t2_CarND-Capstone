
# CarND-Capstone
Self-Driving Car Engineer Nanodegree Program

---
[//]: # (Image References)
[image1]: ./results/images/0_capstone_ros_graph.png "System Diagram"
[image2]: ./results/images/1_initial_screen.png "Initial Screen"
[image3]: ./results/images/2_car_on_the_path.png "Car on Path"
[image4]: ./results/images/3_stop_red.png "Stop on Red"
[image5]: ./results/images/4_detected_index_published_at_292.png "Detected and Index published"
[image6]: ./results/images/5_detected_no_index_published_at_2580.png "Detected but No Index published"
[image7]: ./results/images/6_off_track.png "off track"

[image8]: ./results/videos/1_drive.MOV "drive down the road"
[image9]: ./results/videos/2_red_stop.MOV "stop at read start on green"

## Goals
The goal of this project is to simulate a car on the road with traffic lights. Various ROS nodes are used to implement Perception, Planning and Control modules for the project. Listed below are the nodes in each of the modules followed by a flow chart

* Perception module
  * Obstacle detection node (not implemented)
  * Traffic light detection node that uses a image classifier Neural network
* Planning
  * Waypoint loader node
  * Waypoint updater node that updates target velocities of the waypoints
* Control
  * Drive by wire (DBW) node that actuates throttle, brake and steering
  * Waypoint follower node

### Project system diagram

![sytem_diagram][image1]


## Project implementation details

### Initial screen of simulator
![initial_screen][image2]

### Car driving down the road
![car_on_path][image3]
![drive_down_the_road][image8]

### stopping at signal and progressing
![stop_red][image4]
![stop_at_red_start_on_green][image9]


### Traffic light detection classifier

The most important part of this project is to implement the light classifier based on a pre trained model. Some of the README files from Alexander Lechner and Jean-Paul Haddad had details about the implementation of the traffic light classifier. The traffic light classification tutorial helped me a lot in with chosing the suitable Neural Network and getting the datasets for training and testing. The main goal here to take the pre-trained models that recognise a wide variety of real world objects and narrow them down to just detect the traffic signals using a transfer learning approach. Using the pre-trained model and then  running it on a traffic light data set created from the simulator and further from the rosbag rqt_image_view images will refine the model and improve prediction accuracy. I was not able to detect the light and produce the bounding box when testing the rosbag file. The rosbag images of the traffic light are blurry and the color of the light is not clearly visible. Having a Udacity workspace for this part of the project would have greatly helped to learn this process.


### Issues faced

The most time consuming issue faced was related to the rospy.Rate values in the dbw_node and waypoint_updater node. Tuning this value for the workspace GPU took a lot of time and testing. When the camera was ON and the classifier was working, there seemed to be a lag with the dbw_node signals and the waypoint_updater due to which the car went off track. The reason could be that the waypoints are set for a certain target velocity where as the throttle from the dbw_node is different due to which the car wobbled, steering error went up and it finally went off track. I updated the code to always return false in the verifyFollowing() method of the class waypoint_follower/pure_pursuit_core.cpp to make sure the code continuously checks if the car is following the waypoints. However I could not measure in any way if this worked.

My solution was to create a new method in tl_detector which was based on the process_traffic_lights method. This method is called is_traffic_light_ahead (line 246) and it notifies if there is a light 300 waypoints ahead. Only after this notification, the image_cb code actually reads the image and calls the image classifier model. The advantage is that when there are no lights the additional processing is avoided and the result is that the car follows the path as expected by the waypoint updater node and as commanded by the dbw_node.

The car negotiates the first four signals well but when it reaches the fifth signal at waypoint index 2580, and if the light turns RED in the last minute, due to the light located on a curve and the processing lag, the steering becomes wayward and the car goes off track. When the lights are close to each other at waypoints (2047 and 2580), the image_cb was not publishing the upcoming red light message to the traffic_waypoint topic in time for the waypoint updater to update the waypoints. 

Not having a separate worksapce for building the classifier was not helpful as this is an important part of this project.

Image processing is one of the reason for the car wobble. Having a image processing interval to process the image only after some milliseconds was counter productive as the upcoming RED light message was not pulished by the traffic_waypoint topic on time due to this delay. But it does help by improving the responsiveness of the car to the dbw_node control commands

Here are the pictures of the rospy messages published for tracking the signals
![detected_index_published][image5]
![detected_no_index_published][image6]

Here is an example of the car going off track but it gradually corrects itself and comes back on track. However at high speeds this may not happen.
![off_track][image7]


### Code review

These are the final parameters that worked in the simulator. Any rate in the dbw_node above 15 caused the car to go off track. Lookahead waypoints greater than 50 also caused slow responsiveness
dbw_node rate = 15
waypoint updater node rate = 20
lookahead waypoints = 50


In the dbw_node (line 60), I  had to dampen the parameters for acceleration the  to slow down the car and everything seemed stable at 12 mph (~20 kph)
accel_limit = accel_limit * 0.6 
max_lat_accel = max_lat_accel * 0.6

twist_controller.py
line 130 - when the steering signal is high, mostly the car go off path so I had to dampen the steering and throttle and if the steering signal is too high, I added a brake value. These updates helped the car to slow down and get back on track when the curve in the road is too high.
    ```
    if abs(steering) > 0.8: #0.5:
        #brake = 100 #200
        throttle = throttle * 0.7 #0.6
        steering = steering * 0.6 #0.5
    elif abs(steering) > 1.25: 
        brake = 100 
    ```

tl_detector.py - line 246 - is_traffic_light_ahead() method warns about an incoming traffic light 300 waypoints ahead

yaw_controller.py - line 45 - dampen steering value. This could help when the car goes off track but could also slow down the correction when the car is trying to get back on track. Overall it was better to dampen the steering in the simulator.
    ```
    steer_angle = steer_angle * 0.6;
    ```


## References
  * https://software.intel.com/en-us/articles/traffic-light-detection-using-the-tensorflow-object-detection-api
  * https://medium.com/@anthony_sarkis/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58
  * https://machinelearningmastery.com/transfer-learning-for-deep-learning/
  * https://github.com/alex-lechner/Traffic-Light-Classification



# Udacity's initial README file
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
