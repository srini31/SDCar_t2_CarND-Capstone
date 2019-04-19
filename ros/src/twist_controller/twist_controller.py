import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
#from std_msgs.msg import Int32

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 30.0 # was 40.0

#called in dbw_node.py at 50 hz
class Controller(object):
    def __init__(self, *args, **kwargs): # cleaner way to setup parameters
    
    #def __init__(self, vehicle_mass, fuel_capacity, brake_deadband , decel_limit, accel_limit, 
    #   wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        
        # Retrieve parameters.
        self.vehicle_mass    = kwargs['vehicle_mass']
        self.fuel_capacity   = kwargs['fuel_capacity'] # not used in project
        self.brake_deadband  = kwargs['brake_deadband']
        self.decel_limit     = kwargs['decel_limit'] # comfort parameter, not used in project
        self.accel_limit     = kwargs['accel_limit'] # comfort parameter, not used in project
        self.wheel_radius    = kwargs['wheel_radius']
        self.wheel_base      = kwargs['wheel_base']
        self.steer_ratio     = kwargs['steer_ratio']
        self.max_lat_accel   = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.last_time       = rospy.get_time()
        
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.1, self.max_lat_accel, self.max_steer_angle)
        #0.5, 0.00001, 0.
        # trial and error values, can be changed
        kp = 0.3 #0.8 #0.3
        ki = 0.0 #0.1
        kd = 0.8 #0.06
        min_thr = 0.  # Min throttle value
        max_thr = 0.1 #0.1 #0.2 # Max throttle value
        self.throttle_controller = PID(kp, ki, kd, min_thr, max_thr)  
        
        #Account for cross track error in steering angle
        #kp_cte = 0.5  #CTE
        #ki_cte = 0.0  #CTE
        #kd_cte = 0.2  #CTE
        #self.steer_cte_controller = PID(kp_cte, ki_cte, kd_cte, -self.max_steer_angle, self.max_steer_angle)  #CTE
                    
        #filters the high freq noise in the velocity input
        tau = 0.5 # cutoff_freq = 1/(2*pi*tau)
        ts = 0.02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)
        self.steer_filter = LowPassFilter(0.2,0.1)
         
        #self.vehicle_mass = vehicle_mass
        #self.fuel_capacity = fuel_capacity  # not used in project
        #self.brake_deadband = brake_deadband
        #self.decel_limit = decel_limit   # comfort parameter, not used in project
        #self.accel_limit = accel_limit   # comfort parameter, not used in project
        #self.wheel_radius = wheel_radius # required
        
        #published by tl_detector, so that throttle can be reduced if incoming traffic light is detected
        #rospy.Subscriber('/traffic_light_ahead', Int32, self.traffic_light_ahead_cb) #? Check definition
        #self.traffic_signal_ahead = -1 
        
        rospy.logwarn("initialized Controller")
        
       
    #called in dbw_node.py
    #def control(self, *args, **kwargs):\
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel): ##CTE , cte
        # TODO: Change the arg, kwarg list to suit your needs
        
        #rospy.logwarn("control - dbw_enabled: {0}".format(dbw_enabled))
        if not dbw_enabled: # when car is stopped or taken over by driver, or when fixing the code, dbw is turned off, 
        #make sure PID error is not accumulated
        # integral term accumulates error if PID controller is not turned off, when dbw controller is turned off car may do something erratic as error is accumulated
            self.throttle_controller.reset()
            #self.steer_cte_controller.reset() #CTE
            return 0., 0., 0.
            
        current_vel = self.vel_lpf.filt(current_vel)
        
        #rospy.logwarn("Target linear velocity: {0}".format(linear_vel))
        #rospy.logwarn("Target Angular velocity: {0}".format(angular_vel))
        #rospy.logwarn("Current velocity: {0}".format(current_vel))
        #rospy.logwarn("Filtered velocity: {0}".format(self.vel_lpf.get()))
                        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
                
        ##self.last_vel = current_vel
        ##get latest from repo
        ##get sample time for each step of PID controller, current_time was saved during initialization
        #start commented for NEW
        #will move the car at 25 mph but steering is not stable
        vel_err = linear_vel - current_vel 
        throttle = self.throttle_controller.step(vel_err, sample_time) 
        #end commented for NEW
        brake = 0
        ## start NEW - this will move the car at max 4 mph but it will stay in lane
        ##self.accel_limit = 2. # causes huge steering swing -ve steer to right, +ve steer left
        ##target_v = linear_vel, target_w = angular_vel
        #vel_err = min(linear_vel, MAX_SPEED*ONE_MPH) - current_vel
        #vel_err = max(self.decel_limit*sample_time, min(self.accel_limit*sample_time, vel_err))
        #throttle = self.throttle_controller.step(vel_err, sample_time)
        #throttle = max(0.0, min(1.0, throttle))
        ## end NEW
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        steering = self.steer_filter.filt(steering) #NEW
        # Calculate the additional steering control due to CTE Error and add it to the base.
        #steering_base = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel) #CTE
        #steering_cte  = self.cte_controller.step(cte, sample_time) #CTE
        #steering_total= steering_base + steering_cte #CTE
        #steering = max(min(self.max_steer_angle, steering_total), -self.max_steer_angle) #CTE
                
        if linear_vel == 0. and current_vel < 0.1: # going very slow, stop
            throttle = 0
            brake = 400 #700 N*m to stop carla, accel = 1m/s^2
        elif throttle < 0.1 and vel_err < 0.: # -ve vel_err, car going faster than req vel, slow down
            throttle = 0
            decel = max(vel_err, self.decel_limit) #to make sure decel is within limit
            #brake = abs(decel) * self.vehicle_mass*self.wheel_radius # Torque N*m (m/s^2, kg, m)
            brake = (abs(decel) * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)
                     * self.wheel_radius)  # Torque
            #abs(decel) is needed so that brake value is not -ve; otherwise car does not stop
             
        #when there is too much steering signal, its mostly makes the car go off path, reduce the speed
        if abs(steering) > 0.8: #0.5:
            #brake = 100 #200
            throttle = throttle * 0.7 #0.6
            steering = steering * 0.6 #0.5
        elif abs(steering) > 1.25: 
            brake = 100 
            
        #slow down if a traffic signal is detected ahead
        # since image classifier is ON at this stage, there is lag between dbw_node and waypoint_follower Autoware code
        # to overcome this reduce speed to remain in control
        # this is not the correct method as throttle is reduced outside of decelerate_waypoints, this causes the car to 
        # wobble due to conflicting signals between waypoints and throttle
        #if self.traffic_signal_ahead == 1:
        #    throttle = throttle * 0.6
        
        #rospy.logwarn("control - throttle: {0}".format(throttle))
        #rospy.logwarn("control - brake: {0}".format(brake))
        #rospy.logwarn("control - steering: {0}".format(steering))
        
        # Return throttle, brake, steer
        #return 1., 0., 0. #default values to get the car moving initially before adding the code
        return throttle, brake, steering
                
        # recompute trajectory and new twist commands have a latency and the car can wander a little bit
        
    #def traffic_light_ahead_cb(self, msg):
    #    # TODO: Callback for /traffic_light_ahead message.
    #    #rospy.logwarn("twist ctrl- traffic_light_ahead_cb: {0}".format(msg.data))
    #    self.traffic_signal_ahead = msg.data
    #    # 1= signal ahead else -1
        

        
'''
waypointfollower.cpp - make sure we are following all the way points all the time
change yaw_controller to dampen steering a bit
getsteering() -- current_ang_vel - target_ang_vel and check difference
if 0, dont change steering, if not add some dampening with low pass filter

Autoware code does not recompute trajectory until the car has moved a certain distance
so the car initially wanders after setting waypoint_updater and dbw_node (steps 1 & 2)
with this simple controller by the time, the code has recomputed the new trajectory, 
twist commands and gave directions, the car has already wandered over to a new location 
and then suddenly compensates and wanders to other side viceversa

go to waypointfollower.cpp code and make sure its updating all the time
a function checks if you are following the waypoints, if not it updates
modify to check if car is following way points all the time

2) change yaw_controller to dampen steering a bit
if current and new angular velocity are same dont change, if not add some dampening to it
'''

