#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

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

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        min_speed = rospy.get_param('~min_speed', 0.0)

        linear_p_term = rospy.get_param('~linear_p_term', 0.9)
        linear_i_term = rospy.get_param('~linear_i_term', 0.0005)
        linear_d_term = rospy.get_param('~linear_d_term', 0.07)


        # Control scheme
        # Angle control is done by pure pursuit through Autoware
        # Linear control will be done through linear PID in twister_controller
        params = {
            'vehicle_mass': vehicle_mass,
            'fuel_capacity': fuel_capacity,
            'brake_deadband': brake_deadband,
            'decel_limit': decel_limit,
            'accel_limit': accel_limit,
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'steer_ratio': steer_ratio,
            'max_lat_accel': max_lat_accel,
            'max_steer_angle': max_steer_angle,
            'min_speed': min_speed,
            'linear_p_term': linear_p_term,
            'linear_i_term': linear_i_term,
            'linear_d_term': linear_d_term
        }


        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        self.controller = Controller(**params)

        self.dbw_enabled = False
        self._prev_dbw_enabled = self.dbw_enabled
        self.steering = .0
        self.brake = .0
        self.throttle = .0

        self.linear_velocity_setpoint = .0
        self.angular_velocity_setpoint = .0
        # Only linear
        self.current_linear_velocity = .0

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.dbw_twist_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(20) # 50Hz
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
            controller_params = {
                'linear_velocity_setpoint': self.linear_velocity_setpoint,
                # 'linear_velocity_setpoint': 4.44,
                'angular_velocity_setpoint': self.angular_velocity_setpoint,
                'current_linear_velocity': self.current_linear_velocity
            }

            if self.dbw_enabled:
                throttle, brake, steering = self.controller.control(**controller_params)
                rospy.logwarn('Linear Velo setpoint: %s: ', self.linear_velocity_setpoint)
                rospy.logwarn('Angular Velo setpoint: %s: ', self.angular_velocity_setpoint)
                rospy.logwarn('Current linear velo: %s: ', self.current_linear_velocity)
                rospy.logwarn('Throttle: %s: ', throttle)
                rospy.logwarn('Brake: %s: ', brake)
                rospy.logwarn('Steering: %s: ', steering)
                self.publish(throttle, brake, steering)
            else:
                rospy.logwarn('Should reset')
                self.controller.reset()

            rate.sleep()

    def publish(self, throttle, brake, steer):
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

    def dbw_enabled_cb(self, msg):
        # self._prev_dbw_enabled = self.dbw_enabled
        self.dbw_enabled = msg.data

    def current_velocity_cb(self, msg):
        self.current_linear_velocity = msg.twist.linear.x

    def dbw_twist_cb(self, msg):
        self.linear_velocity_setpoint = msg.twist.linear.x
        self.angular_velocity_setpoint = msg.twist.angular.z

    def edge_trigger(self):
        """
        Raising Edge
        :return:
        """
        pass

if __name__ == '__main__':
    DBWNode()
