#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree
from timeit import default_timer as timer 

STATE_COUNT_THRESHOLD = 3 #2 #
CAMERA_IMG_PROCESS_INT = 0.0 #0.1 #0.2 ok #  30 milliseconds camera image processing interval
WAYPOINT_DIFFERENCE = 300 #300 # Do not process the camera image if next light is > 300 waypoints ahead.

'''
In order to help you acquire an accurate ground truth data source for the traffic light classifier, the Udacity simulator publishes the current color state of all traffic lights in the simulator to the /vehicle/traffic_lights topic in addition to the light location. Use this just for simulator testing and this is not available in Carla
'''
class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.last_img_process_time = 0 # initialize before image_cb
        
        self.light_ahead = None
        #/current_pose can be used used to determine the vehicle's location.
        #/base_waypoints provides the complete list of waypoints for the course.

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light classifier by sending
        the current color state of all traffic lights in the simulator. When testing on the vehicle, 
        the color state will not be available. 
        You'll need to rely on the position of the light and the camera image to predict it.
        '''
        #data sent from simulator - message has RGY and location, good for testing
        #should this topic not be used when the NN classifier is used and tested for real vehicle
        # udacity simulator gives position and state
        # Carla - real world testing, only position is given by this topic, need to get state from NN
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        #image_raw - translate to new color scheme will lose data
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        #/vehicle/traffic_lights provides the (x, y, z) coordinates of all traffic lights.
        #/image_color which provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.
        
        #The permanent (x, y) coordinates for each traffic light's stop line are provided by the config dictionary
        # traffic_light_config is parameter in ros/launch/site.launch and ros/launch/styx.launch
        #ros/launch/styx.launch - ros/src/tl_detector/sim_traffic_light_config.yaml - is_site: true
        #ros/launch/site.launch - ros/src/tl_detector/site_traffic_light_config.yaml - is_site: false
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        #self.is_site = self.config["is_site"]
        #print("is_site %d" % self.is_site)
        
        #The node should publish the index of the waypoint for nearest upcoming red light's stop line to a single topic:
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        #publish if there is an upcoming traffic signal, so that twist_controller can reduce throttle
        #self.upcoming_traffic_light_pub = rospy.Publisher('/traffic_light_ahead', Int32, queue_size=1)
        
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d: ##?? is this required?
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d) #??

    def traffic_cb(self, msg):
        self.lights = msg.lights

    #code to publish the results of process_traffic_lights is written for you already in the image_cb method.
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args: msg (Image): image from car-mounted camera
        """
        if (self.waypoints_tree == None):
            return
        
        #''' #start TEST 2
        if (self.is_traffic_light_ahead()): # read image, run classifier, publish to waypoint updater
            rospy.logwarn("tl_detector.image_cb: YES light within {0} waypoints".format(WAYPOINT_DIFFERENCE))
            #self.upcoming_traffic_light_pub.publish(Int32(1)) 
            pass
        else: #dont read image, and dont call classifier, return from here
            #rospy.logwarn("tl_detector.image_cb: NO light within {0} waypoints".format(WAYPOINT_DIFFERENCE))
            #rospy.logwarn("tl_detector.image_cb: NO light within 300 waypoints self.last_wp: {0}".format(self.last_wp))
            self.upcoming_red_light_pub.publish(Int32(-1)) 
            #self.upcoming_traffic_light_pub.publish(Int32(-1)) 
                # publishing  self.last_wp will make car stuck when TL are too close
                # wait for some time before publishing -1 as waypoint updater may not pick up this signal 
                # it needs some time to start decelerating
            return;
        #''' #end TEST 2
        
        #''' #start TEST 1
        #time_elapsed = timer() - self.last_img_process_time
        ##rospy.loginfo ("image_cb: time_elapsed {}".format(time_elapsed))
        ##Do not process the camera image unless 20 milliseconds have passed from last processing
        #if (time_elapsed < CAMERA_IMG_PROCESS_INT):
        #    return;
        #instead of above CAMERA_IMG_PROCESS_INT logic, use the flag from process_traffic lights which is more accurate
        #that will reduce the time here but can mess up near the light, need to decelerate at that stage
        #''' #end TEST 1
             
        self.has_image = True 
        self.camera_image = msg
        #rospy.logwarn("Got Image... ")
        #rospy.logwarn("self.pose is not None: {0} ".format(self.pose is not None))
        #rospy.logwarn("self.waypoints is not None: {0} ".format(self.waypoints is not None))
        #rospy.logwarn("self.camera_image is not None: {0} ".format(self.camera_image is not None))

        self.last_img_process_time = timer()
        light_wp, state = self.process_traffic_lights() #whenever we get an image this cb is called

        rospy.logwarn("tl_detector - light_wp: {0} ".format(light_wp))
        rospy.logwarn("tl_detector - RED(state=0) state: {0}".format(state))
        #rospy.logwarn("tl_detector - self.state: {0}".format(self.state))
        #rospy.logwarn("tl_detector - self.state_count: {0}".format(self.state_count))
        
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        #stored prev state of light, for every image, determine if its state has changed and count
        #can also update code if light is yellow
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD: #threshold = 3 
            #classifier may be noisy, make sure light is going to stay before taking action
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1 
            #for publishing location of light, only RED state is important
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            #rospy.logwarn("tl_detector - publish RED light_wp: {0}".format(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp)) #
            #rospy.logwarn("tl_detector - publish self.last_wp: {0}".format(self.last_wp))
        self.state_count += 1


    def get_closest_waypoint(self, x, y): #pose
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args: pose (Pose): position to match a waypoint to
        Returns: int: index of the closest waypoint in self.waypoints
        """
        #TODO implement
        closest_idx = self.waypoints_tree.query([x,y],1)[1] #KDTree to get closest index
        return closest_idx

    #Use the camera image data to classify the color of the traffic light.
    #train a deep learning classifier to classify the entire image as containing either a red light, yellow light, green light, or no light. 
    #One resource that's available to you is the traffic light's position in 3D space via the vehicle/traffic_lights topic.
    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args: light (TrafficLight): light to classify
        Returns: int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #when running in the simulator during testing, the light state is available, no classifier is needed
        #return light.state
        #'''
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)
        #'''
        

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        closest_light = None
        line_wp_idx = None
        #List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.waypoints_tree):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y) #using KDTree
            #TODO find the closest visible traffic light (if one exists)
            #Need to get index of closet traffic line, from the light info
            
            diff = len(self.waypoints.waypoints) #num of waypoints
            for i,light in enumerate(self.lights): #iterate through traffic lights - 8 intersections, not using KDTree for small list
                #Get stop line way point index
                line = stop_line_positions[i] #Get the line of traffic light
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                #Find closest stop line way point index
                d = temp_wp_idx - car_wp_idx # num of indices between closest traffic line wp and car position
                #rospy.logwarn("tl_detector - process_traffic_lights d: {0}".format(d))
                #rospy.logwarn("tl_detector - process_traffic_lights diff: {0}".format(diff))
                if d > 0 and d < diff: #waypoint is ahead of car and < num of waypoints
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
                    #at this point we have closest light and index of the waypoint closest to the light
                    
        # do not process the camera image unless the traffic light <= 300 waypoints
        #if closest_light and ((line_wp_idx - car_wp_idx)  <= WAYPOINT_DIFFERENCE):
        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state
        
        return -1, TrafficLight.UNKNOWN #For UNKNOWN, keep car going can still test dbw system, reality need to stop the car
            
        
           
    #''' #start new code to reduce processing time by reading the image only when the car approaches the light
    def is_traffic_light_ahead(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None
        self.light_ahead = False
        #List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.waypoints_tree):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y) #using KDTree
            #TODO find the closest visible traffic light (if one exists)
            #Need to get index of closet traffic line, from the light info
            
            diff = len(self.waypoints.waypoints) #num of waypoints
            for i,light in enumerate(self.lights): #iterate through traffic lights - 8 intersections, not using KDTree for small list
                #Get stop line way point index
                line = stop_line_positions[i] #Get the line of traffic light
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                #Find closest stop line way point index
                d = temp_wp_idx - car_wp_idx # num of indices between closest traffic line wp and car position
                if d > 0 and d < diff: #waypoint is ahead of car and < num of waypoints
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
                    #at this point we have closest light and index of the waypoint closest to the light
                    
        # do not process the camera image unless the traffic light <= 300 waypoints
        if closest_light and ((line_wp_idx - car_wp_idx)  <= WAYPOINT_DIFFERENCE):
            self.light_ahead = True
            #rospy.logwarn("tl_detector light ahead line_wp_idx: {0}".format(line_wp_idx))
            #rospy.logwarn("tl_detector light ahead car_wp_idx : {0}".format(car_wp_idx))
        
        return self.light_ahead
            
    #''' #end new code to reduce processing time

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
