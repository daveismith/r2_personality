"""Record every dome joint state, personality command, controller command and dome-arbiter log line."""
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import Log
from astromech_msgs.msg import ChannelStatus
out = open(sys.argv[1], 'a', buffering=1)
def p(*a): out.write('%.3f ' % time.time() + ' '.join(str(x) for x in a) + '\n')
rclpy.init(); n = Node('probe_floor'); q = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
def js(m):
    if 'dome_joint' in m.name:
        i = m.name.index('dome_joint'); p('J %.4f %.4f' % (m.position[i], m.velocity[i] if m.velocity else float('nan')))
last = {}
def cs(m):
    if m.channel != 'dome': return
    k = (m.state, m.active_source, m.active_action, m.active_intent_id)
    if k != last.get('d'): last['d'] = k; p('CH', *k)
n.create_subscription(JointState, '/dome_state_broadcaster/joint_states', js, q)
n.create_subscription(TwistStamped, '/dome/cmd/personality', lambda m: p('P %.3f' % m.twist.angular.z), q)
n.create_subscription(Float64MultiArray, '/dome_controller/commands', lambda m: p('C %.3f' % m.data[0]), q)
n.create_subscription(ChannelStatus, '/expression/status', cs, q)
n.create_subscription(Log, '/rosout', lambda m: p('LOG', m.name, m.msg) if m.name in ('dome_arbiter', 'dome_twist_converter', 'roamadome', 'RoamadomeControl') else None, q)
rclpy.spin(n)
