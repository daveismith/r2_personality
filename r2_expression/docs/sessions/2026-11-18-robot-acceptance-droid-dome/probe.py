"""Phase 1 droid-dome run: log every channel's status changes, the device's LED and Magic
Panel state changes, the mood, intents and the arbiters' log lines."""
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from astromech_msgs.msg import ChannelStatus, MoodState, ExpressionIntent, LedStateArray, MagicPanelState
from rcl_interfaces.msg import Log
out = open(sys.argv[1], 'a', buffering=1)
def p(*a): out.write('%.3f ' % time.time() + ' '.join(str(x) for x in a) + '\n')
class P(Node):
    def __init__(s):
        super().__init__('probe_leds')
        rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        s.last = {}; s.lm = None; s.lastled = {}; s.lastmagic = None
        s.create_subscription(ChannelStatus, '/expression/status', s.cs, rel)
        s.create_subscription(MoodState, '/mood/state', s.ms, rel)
        s.create_subscription(ExpressionIntent, '/expression/intent', s.intent, rel)
        s.create_subscription(LedStateArray, '/r2_dome/leds/states', s.leds, rel)
        s.create_subscription(MagicPanelState, '/r2_dome/magic/states', s.magic, rel)
        s.create_subscription(Log, '/rosout', s.lg, rel)
    def cs(s, m):
        k = (m.state, m.owner, m.priority, m.active_source, m.active_action, m.active_intent_id, m.mood, m.dropped, m.refused)
        if k != s.last.get(m.channel):
            s.last[m.channel] = k
            p('CH', m.channel, 'state=%d owner=%s prio=%d lease=%.1f src=%s act=%s id=%s rem=%d mood=%s dropped=%d refused=%d' % (
                m.state, m.owner, m.priority, m.lease_remaining_s, m.active_source, m.active_action, m.active_intent_id, m.active_remaining_ms, m.mood, m.dropped, m.refused))
    def ms(s, m):
        k = (m.mood, m.locked, round(m.mood_strength, 1))
        if k != s.lm:
            s.lm = k; p('MOOD', m.mood, 'locked=%s str=%.2f' % (m.locked, m.mood_strength))
    def intent(s, m):
        p('INTENT', m.source, m.channel, m.action, m.priority, m.duration_ms, m.params, m.intent_id)
    def leds(s, m):
        for d in getattr(m, 'displays', []):
            fields = {f: getattr(d, f) for f in d.get_fields_and_field_types() if f not in ('hold_remaining_ms',)}
            k = repr(sorted(fields.items()))
            if k != s.lastled.get(d.name):
                s.lastled[d.name] = k
                p('LED', d.name, ' '.join('%s=%s' % (a, b) for a, b in sorted(fields.items())))
        top = {f: getattr(m, f) for f in m.get_fields_and_field_types() if f not in ('displays', 'header', 'surfaces')}
        k = repr(sorted(top.items()))
        if k != s.lastled.get('_top'):
            s.lastled['_top'] = k; p('LEDS', ' '.join('%s=%s' % (a, b) for a, b in sorted(top.items())))
    def magic(s, m):
        skip = ('header', 'read_age_ms', 'elapsed_ms', 'remaining_ms', 'live', 'polls')
        fields = {f: getattr(m, f) for f in m.get_fields_and_field_types() if f not in skip}
        k = repr(sorted(fields.items()))
        if k != s.lastmagic:
            s.lastmagic = k; p('MAGIC', ' '.join('%s=%s' % (a, b) for a, b in sorted(fields.items())))
    def lg(s, m):
        if m.name in ('led_arbiter', 'magic_arbiter', 'r2_affect', 'r2_dome'):
            p('LOG', m.name, m.level, m.msg)
rclpy.init(); n = P()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
