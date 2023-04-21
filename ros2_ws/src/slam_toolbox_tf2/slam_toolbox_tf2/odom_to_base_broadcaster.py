import math
import numpy as np
import rclpy

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# few temp global variables
x = 0.0
y = 0.0
th = 0.0
vx = 0.1
vy = -0.1
vth = 0.1

class FramePublisher(Node):

	def __init__(self):
		super().__init__('turtle_tf2_frame_publisher')

		# Initialize the transform broadcaster
		self.tf_broadcaster = TransformBroadcaster(self)
		self.publisher = self.create_publisher(Odometry, 'odometry', 10)

		# Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
		# callback function on each message
		self.subscription = self.create_subscription(
			Odometry,
			f'imu/data',
			self.handle_robot_pose,
			1)
		self.subscription  # prevent unused variable warning


	def get_dummy_odom(self):
		global x, y, th, vx, vy, vth
		current_time = FramePublisher.get_clock().now()
		last_time = FramePublisher.get_clock.now()

		dt = (current_time - last_time).toSec()
		delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
		delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
		delta_th = vth * dt

		x += delta_x
		y += delta_y
		th += delta_th

		return x, y, th
	

	def quaternion_from_euler(ai, aj, ak):
		ai /= 2.0
		aj /= 2.0
		ak /= 2.0
		ci = math.cos(ai)
		si = math.sin(ai)
		cj = math.cos(aj)
		sj = math.sin(aj)
		ck = math.cos(ak)
		sk = math.sin(ak)
		cc = ci*ck
		cs = ci*sk
		sc = si*ck
		ss = si*sk

		q = np.empty((4, ))
		q[0] = cj*sc - sj*cs
		q[1] = cj*ss + sj*cc
		q[2] = cj*cs - sj*sc
		q[3] = cj*cc + sj*ss

		return q


	def handle_robot_pose(self, msg):
		x, y, theta = self.get_dummy_odom()

		t = TransformStamped()

		# Read message content and assign it to
		# corresponding tf variables
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = 'odom'
		t.child_frame_id = 'base_footprint'
		

		# Turtle only exists in 2D, thus we get x and y translation
		# coordinates from the message and set the z coordinate to 0
		t.transform.translation.x = x
		t.transform.translation.y = y
		t.transform.translation.z = 0

		# For the same reason, turtle can only rotate around one axis
		# and this why we set rotation in x and y to 0 and obtain
		# rotation in z axis from the message
		q = msg.orientation
		t.transform.rotation.x = q.x
		t.transform.rotation.y = q.y
		t.transform.rotation.z = q.z
		t.transform.rotation.w = q.w

		# Send the transformation
		self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    print("Running odom -> base_footprint broadcaster")
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
