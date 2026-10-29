"""One long-lived rclpy node for the phase 2 robot items 9 and 5: logs the dome channel, the
converter's source, the personality command, the drive command and the arbiter's log lines;
publishes ExpressionIntent from lines appended to a command file:
  look <bearing> <profile> [source] [priority]     e.g. look 2.0 lazy behavior 100
  glance <amplitude> <dwell_s> [source] [priority]
  stop [source] [priority]
"""
import json, os, sys, time, uuid, math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import JointState
from astromech_msgs.msg import ExpressionIntent, ChannelStatus, DomeCommandStatus, MoodState
from rcl_interfaces.msg import Log

out = open(sys.argv[1], 'a', buffering=1)
cmdfile = sys.argv[2]
LADDER = {'idle': 10, 'behavior': 100, 'manual': 199}
def p(*a):
    out.write('%.3f ' % time.time() + ' '.join(str(x) for x in a) + '\n')

class P(Node):
    def __init__(s):
        super().__init__('probe_dome')
        rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        s.pub = s.create_publisher(ExpressionIntent, '/expression/intent', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        s.lasts = None; s.lastsrc = None; s.lastpers = None; s.lastdrive = 0.0; s.lastdrive_t = 0
        s.lastjoint = None; s.lastjoint_t = 0
        s.create_subscription(ChannelStatus, '/expression/status', s.cs, rel)
        s.create_subscription(DomeCommandStatus, '/dome/cmd/status', s.dcs, rel)
        s.create_subscription(TwistStamped, '/dome/cmd/personality', s.pers, rel)
        s.create_subscription(Float64MultiArray, '/dome_controller/commands', s.cmds, rel)
        s.create_subscription(JointState, '/dome_state_broadcaster/joint_states', s.js, rel)
        s.create_subscription(MoodState, '/mood/state', s.ms, rel)
        s.create_subscription(Log, '/rosout', s.lg, rel)
        s.create_subscription(ExpressionIntent, '/expression/intent', s.intent, rel)
        s.lm = None; s.lastcmd = None
        for t, typ in (('/diff_cont/cmd_vel', TwistStamped), ('/cmd_vel', TwistStamped), ('/cmd_vel_joy', TwistStamped)):
            try:
                s.create_subscription(typ, t, (lambda tt: (lambda m: s.drive(tt, m)))(t), rel)
            except Exception as e:
                p('SUBFAIL', t, e)
        s.pos = 0
        s.create_timer(0.2, s.poll)
    def poll(s):
        try:
            with open(cmdfile) as f:
                f.seek(s.pos); lines = f.read(); s.pos = f.tell()
        except FileNotFoundError:
            return
        for line in lines.splitlines():
            parts = line.split()
            if not parts: continue
            verb = parts[0]
            m = ExpressionIntent(); m.header.stamp = s.get_clock().now().to_msg()
            m.channel = 'dome'; m.interruptible = True
            m.intent_id = 'probe-' + uuid.uuid4().hex[:8]
            try:
                if verb == 'look':
                    m.action = 'look_at'; m.params = json.dumps({'bearing': float(parts[1]), 'profile': parts[2]}); rest = parts[3:]
                elif verb == 'glance':
                    m.action = 'glance'; m.params = json.dumps({'amplitude': float(parts[1]), 'dwell_s': float(parts[2])}); rest = parts[3:]
                elif verb == 'stop':
                    m.action = 'stop'; m.params = ''; rest = parts[1:]
                else:
                    p('CMD?', line); continue
            except Exception as e:
                p('CMDERR', line, e); continue
            m.source = rest[0] if rest else 'manual'
            m.priority = int(rest[1]) if len(rest) > 1 else LADDER[m.source]
            s.pub.publish(m)
            p('SENT', m.source, m.action, m.priority, m.params, m.intent_id)
    def intent(s, m):
        if not m.intent_id.startswith('probe-'):
            p('INTENT', m.source, m.channel, m.action, m.priority, m.params, m.intent_id)
    def cs(s, m):
        if m.channel != 'dome': return
        k = (m.state, m.owner, m.priority, m.active_source, m.active_action, m.active_intent_id, m.mood, m.dropped, m.refused)
        if k != s.lasts:
            s.lasts = k
            p('DOME', 'state=%d owner=%s prio=%d src=%s act=%s id=%s rem=%d mood=%s dropped=%d refused=%d stamp=%.3f' % (
                m.state, m.owner, m.priority, m.active_source, m.active_action, m.active_intent_id, m.active_remaining_ms, m.mood, m.dropped, m.refused, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    def dcs(s, m):
        k = (m.source, m.capped, m.velocity_rad_s != 0.0)
        if k != s.lastsrc:
            s.lastsrc = k
            p('CONV', 'source=%s v=%.3f capped=%s stamp=%.3f' % (m.source, m.velocity_rad_s, m.capped, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    def pers(s, m):
        v = m.twist.angular.z
        k = (v != 0.0, math.copysign(1, v) if v else 0)
        if k != s.lastpers:
            s.lastpers = k
            p('PERS', '%.3f' % v)
    def cmds(s, m):
        v = m.data[0] if m.data else float('nan')
        k = (v != 0.0, math.copysign(1, v) if v else 0)
        if k != s.lastcmd:
            s.lastcmd = k
            p('CTRL', '%.3f' % v)
    def js(s, m):
        if 'dome_joint' not in m.name: return
        pos = m.position[m.name.index('dome_joint')]
        t = time.time()
        if s.lastjoint is None or abs(pos - s.lastjoint) > 0.05 or t - s.lastjoint_t > 2.0:
            s.lastjoint = pos; s.lastjoint_t = t
            p('JOINT', '%.3f' % pos, 'deg=%.1f' % math.degrees(pos))
    def drive(s, topic, m):
        tw = m.twist if hasattr(m, 'twist') else m
        sp = math.hypot(tw.linear.x, tw.linear.y); yaw = abs(tw.angular.z)
        moving = sp > 0.15 or yaw > 0.3
        t = time.time()
        if moving != (s.lastdrive > 0) or (moving and t - s.lastdrive_t > 0.5):
            s.lastdrive = 1.0 if moving else 0.0; s.lastdrive_t = t
            p('DRIVE', topic, 'lin=%.3f yaw=%.3f moving=%s' % (sp, yaw, moving))
    def ms(s, m):
        k = (m.mood, m.locked)
        if k != s.lm:
            s.lm = k
            p('MOOD', m.mood, 'locked=%s str=%.2f' % (m.locked, m.mood_strength))
    def lg(s, m):
        if m.name in ('dome_arbiter', 'dome_twist_converter', 'r2_affect'):
            p('LOG', m.name, m.level, m.msg)

rclpy.init()
n = P()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
