#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

#from styx_msgs.msg import Lane, Waypoint #CTE
#from geometry_msgs.msg import PoseStamped #CTE

from twist_controller import Controller
from timeit import default_timer as timer

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

DBW_RATE = 15 #50 Hz for submission

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35) #required to stop the car
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413) #required for brake torque  
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        #
        accel_limit = accel_limit * 0.6 #updated these parameters to slow down the speed
        max_lat_accel = max_lat_accel * 0.6
        #
        # TODO: Create `Controller` object
        # self.controller = Controller(<Arguments you wish to provide>)
        parameters = {
                    'vehicle_mass' : vehicle_mass,					
                    'fuel_capacity' : fuel_capacity,					
                    'brake_deadband' : brake_deadband,
                    'decel_limit' : decel_limit,
                    'accel_limit' : accel_limit,
                    'wheel_radius' : wheel_radius,
                    'wheel_base' : wheel_base,
                    'steer_ratio' : steer_ratio,
                    'max_lat_accel' : max_lat_accel,
                    'max_steer_angle' : max_steer_angle
        }		
        self.controller = Controller(**parameters) # cleaner way to setup parameters
        '''
        self.controller = Controller(vehicle_mass = vehicle_mass,
                                    fuel_capacity = fuel_capacity, 
                                    brake_deadband = brake_deadband,
                                    decel_limit = decel_limit,
                                    accel_limit = accel_limit,
                                    wheel_radius = wheel_radius,
                                    wheel_base = wheel_base,
                                    steer_ratio = steer_ratio,
                                    max_lat_accel = max_lat_accel,
                                    max_steer_angle = max_steer_angle)'''
            

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        #rospy.Subscriber('/vehicle/twist_cmd', TwistStamped, self.twist_cb)
        #rospy.Subscriber('/vehicle/current_velocity', TwistStamped, self.velocity_cb)   
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb) 
        
        self.current_vel = None
        self.cur_ang_vel = None
        self.dbw_enabled = None
        self.linear_vel = None
        self.angular_vel = None
        self.throttle = self.brake = self.steering = None
        #self.final_waypoints_2d  = None #CTE
        #rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb) #CTE 
        #rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) #CTE
        self.last_spin_time = 0 # initialize number of times dbw_node spins
        
        self.loop()


    def loop(self):
        rate = rospy.Rate(DBW_RATE) # 50Hz, publishing at less than this rate will stop DBW in the middle of testing
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            
            #rospy.logwarn("self.current_vel: {0}".format(self.current_vel))
            #rospy.logwarn("self.linear_vel: {0}".format(self.linear_vel))
            #rospy.logwarn("self.angular_vel: {0}".format(self.angular_vel))
        
            if not None in (self.current_vel, self.linear_vel, self.angular_vel):
                #controller comes from twist_controller.py
                #cte = self.get_cte(self.final_waypoints_2d, self.pose) #CTE
                #rospy.logwarn("** not None in (self.current_vel, self.linear_vel, self.angular_vel)")
                ##time_elapsed = timer() - self.last_spin_time
                ##rospy.logwarn("dbmwNode: time_elapsed {0}".format(time_elapsed))
                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,
                                                                                    self.dbw_enabled, 
                                                                                    self.linear_vel, 
                                                                                    self.angular_vel) ##CTE , cte
                ##self.last_spin_time = timer()
                                                                                
            if(self.dbw_enabled):
                self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data #
        #rospy.logwarn("** self.dbw_enabled: {0} ".format(self.dbw_enabled))
        #rospy.logwarn("** self.dbw_enabled msg: {0} ".format(msg))
        #rospy.logwarn("** self.dbw_enabled msg.data: {0} ".format(msg.data))
    
    def twist_cb(self, msg):
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z
        
    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x
        
    def publish(self, throttle, brake, steer):
        
        #to just make th car move forward, set throttle to 1 [0,1] and steering and brake = 0
        #throttle = 1. 
        #brake = 0.
        #steer = 0.
        #rospy.logwarn("throttle: {0} ".format(throttle))
        #rospy.logwarn("brake: {0}".format(brake))
        #rospy.logwarn("steer: {0}".format(steer))
        
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    '''
    #start CTE
    def pose_cb(self, msg):
        self.pose = msg
        pass


    def final_waypoints_cb(self, msg): #CTE
        final_waypoints = msg.waypoints
        self.final_waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in final_waypoints]
        pass	


    def get_cte(self,final_waypoints_2d,current_pose): #CTE
        if final_waypoints_2d is not None and current_pose is not None:		
            #convert from quaternion to yaw
            orientation_q = current_pose.pose.orientation
            _, _, yaw = euler_from_quaternion ([orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w])	

            #get the x and y of the current pose			
            x = current_pose.pose.position.x
            y = current_pose.pose.position.y
            x_transform = []
            y_transform = []		
            cos_yaw = math.cos(-yaw)
            sin_yaw = math.sin(-yaw)

            #set the number of points to fit			
            points_to_fit = 10
            points_to_fit = min(points_to_fit,len(final_waypoints_2d))		
			
            #transform the waypoints to the vehicle frame
            for i in range(points_to_fit):			
                waypoint_x,waypoint_y = final_waypoints_2d[i]
                x_d = waypoint_x - x
                y_d = waypoint_y - y
                x_transform.append(x_d *cos_yaw - y_d*sin_yaw)
                y_transform.append(x_d *sin_yaw + y_d*cos_yaw)	

            # Fit a 3rd degree polynomial to the waypoints
            degree = 3
            coefficients = np.polyfit(x_transform, y_transform, degree)

            return np.polyval(coefficients, 0.0)				
        else:
            return 0 

    #end CTE
    '''
            
if __name__ == '__main__':
    DBWNode()

