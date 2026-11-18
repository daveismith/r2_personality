"""Constant-speed runs through the converter's Foxglove input: 3 s each, then silence."""
import sys, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init(); n = Node('probe_drive'); pub = n.create_publisher(Twist, '/cmd_vel_dome_foxglove', 10)
out = open(sys.argv[1], 'a', buffering=1); time.sleep(3)
for v in [float(x) for x in sys.argv[2:]]:
    out.write('%.3f RUN %.2f\n' % (time.time(), v)); t = time.time(); m = Twist(); m.angular.z = v / 1.5
    while time.time() - t < 3.0: pub.publish(m); time.sleep(0.05)
    out.write('%.3f END %.2f\n' % (time.time(), v)); time.sleep(4.5)
