#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math

import numpy as np
from scipy.spatial import KDTree

'''
rostopic echo /final_waypoints
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''
#200 waypoints - going fast and out of control
LOOKAHEAD_WPS = 100 #50 #200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0
WAYPOINT_RATE = 40 #20 #15 ok# 10 #50 Hz for submission

#after step 1 open the simulator and see the points path
class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.waypoints_2d = None #initialized before subscriber to prevent race condition
        # TODO: Add other member variables you need below
        self.pose = None
        self.base_lane = None #base_waypoints
        self.waypoints_tree = None
        self.stopline_wp_idx = -1 
        
        #Step 1 - subscribet to /base_waypoints and /current_pose
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) #waypoints_cb - callback function that is called
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #step 2 - subscribe to /traffic_waypoint, not handling obstacles in this project
        #This is published by tl_detector/TLDetector - using the NN for traffic light detection
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb) #? Check definition
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb) ? Check definition
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1) #main goal of this class
        self.loop() #gives control over publishing frequency
        #rospy.spin()

    # using loop instead of spin to control publishing frequency
    # published way points go to waypoint_follower from autoware, thats running at 30 Hz
    def loop(self): #New
        rate = rospy.Rate(WAYPOINT_RATE) #change back to 50 later
        while not rospy.is_shutdown():
            if self.pose and self.base_lane: #and self.waypoints_tree base_waypoints
                #Get closest waypoint
                #closest_waypoint_idx = self.get_closest_waypoint_idx()
                #self.publish_waypoints(closest_waypoint_idx)
                self.publish_waypoints()
            rate.sleep()

    #return the index of the closest waypoint to the current position of the car
    def get_closest_waypoint_idx(self): #New
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoints_tree.query([x,y], 1)[1]
        
        #first 1 = return only one point, second 1 = index of that coord, 
        # data is in same order in KDTree as the waypoints_2d array
        #check if closest waypoint is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        
        # prev_vect - between prev idx and closest index, hyperplane is perpendicular to this vectors
        # car current_pose (position) can be ahead or behind the closest index
        # pos_vect is distance between closest_idx and current_pose
        #Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        if val > 0: #way point is behind the car, take the next waypoint
        #dot prod +ve, vectors are in same dir, car is ahead of closest waypoint
        #dot prod -ve, vectors are in opp dir, car is behind closest waypoint
            closest_idx = (closest_idx+1) % len(self.waypoints_2d)
            
        return closest_idx
    
    #Main task of this method is to publish 200 waypoints ahead of the car
    def publish_waypoints(self): # , closest_idx
        '''
        # start part 1 test
        lane = Lane() #message type is lane
        lane.header = self.base_lane.header #optional assignment #base_waypoints
        lane.waypoints = self.base_lane.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS] #as we get to end of list, lane.waypoints will gradually become smaller as closest_idx approaches the size of waypoints list
        #python slicing just returns remaining elements even if index goes out of bounds
        self.final_waypoints_pub.publish(lane)
        # end part 1 test
        '''
        #start final setup
        final_lane = self.generate_lane()            #uncomment for step 2
        self.final_waypoints_pub.publish(final_lane) #uncomment for step 2
        
    #stop/decelerate the car leading up to the traffic light, update twist.linear.x-b
    def generate_lane(self):
        lane = Lane()
        lane.header = self.base_lane.header #optional assignment #base_waypoints
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        #just use the sliced waypoints array instead of all waypoints to improve processing speed
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
            #rospy.logwarn("did not detect any traffic lights")
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            rospy.logwarn("waypt_upd, gen_lane, traffic light ahead, decelerating")
            
        return lane
        
    
    # if the waypoints list is large, its recommended to slice the list before updating the velocities
    # iterating over entire list each time is not efficient and can introduce latency
    # by the time we update, the car may have already passed those waypoints
    #** Dont udpate base_waypoints, create new array for storing the deceleration waypoints as these could be different
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints): # enumerate the sliced waypoints
            p = Waypoint()                 # create new waypoint
            p.pose = wp.pose               # position of wp is not going to change, pose also has orientation
            
            # 2 waypoints back from line so front part of car stops at line?? check
            # stopline_wp_idx - closest traffic light from tl_detector
            # closest_idx - the closest index in waypoints array to this traffic light stopline_wp_idx 
            # all idx are in waypoints array
            #stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) 
            stop_idx = max(self.stopline_wp_idx - closest_idx - 10, 0) #there is a lag in simulator, try with 6 waypoints back
            
            #dist func will add piecewise distances between each waypoint and stopline_wp_idx
            #gradually the velocity of waypoints approaching the traffic light will be reduced and finally set to 0
            #after light changes to green, car will start accelerating again
            # if i > stop_idx, dist will return 0
            dist = self.distance(waypoints, i, stop_idx)
            #the velocity gradually goes to zero for stopping, as dist becomes small, velocity becomes small
            # can use a S curve or constant to smoothly reduce velocity instead of sqrt
            vel = math.sqrt(2 * MAX_DECEL * dist) #similar to waypoint_loader code, starts slow and decelerate to stop
            if vel < 1.:
                vel = 0
            
            #keep velocity in speed limit, close to way point velocity and if vel goes down for stopping, assign that to linear.x using min function
            #min function makes sure we are either decelerating or maintaining the current speed
            #otherwise if we are away from the light, vel can increase as the distance to light is large
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x) 
            temp.append(p)
            
        return temp
     #end final step
        
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg # called frequently at 50Hz
        pass

    #base way points dont change, just store them once
    #set as latched subscriber, once callback is called, it doesnt send base_waypoints anymore, base_waypoints dont change
    #use first 200 infront of car for reference
    #KDTree - date structure that helps to look up closest way point in space efficiently @ O(log(N))
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #self.base_waypoints = waypoints
        self.base_lane = waypoints
        #make sure waypoints_2d is initialized before the subscriber
        #if (self.waypoints_tree == None): #make sure KDTree is not called on empty list
        #    return
        if not self.waypoints_2d: # convert waypoints to 2d coordinates for each coord
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints] #?
            self.waypoints_tree = KDTree(self.waypoints_2d)
        #pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.logwarn("waypt upd, traffic_cb: stopline_wp_idx: {0}".format(msg.data))
        self.stopline_wp_idx = msg.data
        #pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    #distance between two waypoints - current waypoint and the stopline_wp_idx
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
