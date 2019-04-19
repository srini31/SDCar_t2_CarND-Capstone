from math import atan
import rospy

# min_speed = 0.1
#provides the steering commands. 0.1 lowest speed of the car
#these parameters should be forwarded to the YawController from dbw.py
class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle
        #
        self.prev_ang_vel = 0.
        self.prev_steer = 0.

    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        #''' #start version 1
        #return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
        #''' #end version 1
        
        # Adding below code seems to work
        # ''' #start version 2 NEW, 0.8 is the dampening term
        steer_angle = self.get_angle(max(current_velocity*0.8, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
        
        #rospy.logwarn("YawController self.prev_steer : {0} seconds".format(self.prev_steer))
        #rospy.logwarn("YawController steer_angle orig: {0} seconds".format(steer_angle))
        #print('YawController self.prev_steer  = ', self.prev_steer)
        #print('YawController steer_angle orig = ', steer_angle)
        #dampen the steer_angle so that the car does not go off track
        steer_angle = steer_angle * 0.6;
        #print('YawController steer_angle damp = ', steer_angle)
        #rospy.logwarn("YawController steer_angle damp: {0} seconds".format(steer_angle))
        self.prev_steer = steer_angle
        
        if current_velocity < self.min_speed:
            return 0.0
            #print('YawController steer_angle = 0.0 ')
        else:
            #return self.get_angle(max(current_velocity*0.8, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
            #print('YawController steer_angle = ', steer_angle)
            return steer_angle;
        # ''' #end version 2 NEW
        
     

        '''
        #based on dbw walkthrough, dampen angular velocity if its different from prev, otherwise dont change
        if (self.prev_ang_vel - angular_velocity) < 0.01
            return self.prev_ang_vel
        else:
            return angular_velocity *0.75
        '''