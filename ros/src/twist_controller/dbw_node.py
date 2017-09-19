#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from styx_msgs.msg import Lane, Waypoint
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
import math
import numpy as np

import tf

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
        rospy.init_node('dbw_node', log_level=rospy.DEBUG)

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

        min_speed = rospy.get_param('~min_speed', 0.1)

        linear_p_term = rospy.get_param('~linear_p_term', 0.5)
        linear_i_term = rospy.get_param('~linear_i_term', 0.001)
        linear_d_term = rospy.get_param('~linear_d_term', 0.05)

        angular_p_term = rospy.get_param('~angular_p_term', 0.5)
        angular_i_term = rospy.get_param('~angular_i_term', 0.001)
        angular_d_term = rospy.get_param('~angular_d_term', 0.05)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Control scheme:
        # Angle control is done by pure pursuit through Autoware
        # Linear control will be done through linear PID in twist_controller?
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
            'linear_d_term': linear_d_term,

            'angular_p_term': angular_p_term,
            'angular_i_term': angular_i_term,
            'angular_d_term': angular_d_term
        }

        # TODO: Create `TwistController` object
        self.controller = Controller(**params)

        self.dbw_enabled = False
        self._prev_dbw_enabled = self.dbw_enabled
        self.steer = .0
        self.brake = .0
        self.throttle = .0

        self.linear_velocity_setpoint = .0
        self.angular_velocity_setpoint = .0
        # Only linear
        self.current_velocity = .0
        self.final_waypoints = None
        # self.steer_data = []

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.dbw_twist_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)

        self.loop()

    def loop(self):
        # rate = rospy.Rate(50) # 50Hz
        # For low performance env
        rate = rospy.Rate(1)  # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # No current_pose yet
            # polyfit_coeffs = self._get_polyfit_coeffs(self.final_waypoints, self.current_pose)
            # cte = self._get_cte(polyfit_coeffs)

            throttle, brake, steering = self.controller.control(self.linear_velocity_setpoint,
                                                                self.angular_velocity_setpoint,
                                                                self.current_velocity,
                                                                # cte
                                                                # Other params
                                                                )
            # Brake should be given in units of torque
            # Which can be calculated using the desired acceleration,
            # The weight of the vehicle, and the wheel radius
            # https://carnd.slack.com/archives/C6NVDVAQ3/p1504810396000059?thread_ts=1504735921.000376&cid=C6NVDVAQ3

            # More on brake torque value
            # https://carnd.slack.com/archives/C6NVDVAQ3/p1504061507000179
            # 20000 seems a good number

            # Test only
            # throttle = 0.02
            # brake = 0
            # steering = 0.1
            if self.edge_trigger():
                # Reset controller pid here
                self.controller.reset()

            # if self.dbw_enabled:

            rospy.logdebug("Throttle: %s: ", throttle)
            rospy.logdebug("brake: %s: ", brake)
            rospy.logdebug("steering: %s: ", steering)
            # self.publish(throttle, brake, steering)
            self.publish(throttle, 0, steering)

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
        self._prev_dbw_enabled = self.dbw_enabled
        self.dbw_enabled = msg.data

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def dbw_twist_cb(self, msg):
        self.linear_velocity_setpoint = msg.twist.linear.x
        self.angular_velocity_setpoint = msg.twist.angular.z

    def current_pose_cb(self, msg):
        self.current_pose = msg.pose

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg.waypoints

    def _get_polyfit_coeffs(self, waypoints, pose):
        """
        https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        2D case
        :param waypoints: ROS message
        :param pose: ROS message
        :return: coeffs
        """
        car_yaw = tf.transformations.eular_from_quaternion(pose.quaternion)[2]
        px = pose.position.x
        py = pose.position.y
        wp_x_in_car_coord = []
        wp_y_in_car_coord = []
        for index, wp in enumerate(waypoints):
            x = wp.pose.position.x - px
            y = wp.pose.position.y - py
            wp_x_in_car_coord.push(x * math.cos(-car_yaw) - y * math.sin(-car_yaw))
            wp_y_in_car_coord.push(x * math.sin(-car_yaw) + y * math.cos(-car_yaw))

        # 3 order polyfit
        return np.polyfit(wp_x_in_car_coord, wp_y_in_car_coord, 3)

    def _get_cte(self, poly_coeffs):
        return poly_coeffs[0]

    def edge_trigger(self):
        """
        Raising Edge
        :return:
        """
        if self._prev_dbw_enabled is False and self.dbw_enabled is True:
            return True
        else:
            return False

if __name__ == '__main__':
    DBWNode()
