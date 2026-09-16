"""Validate one-second windows, idle NaNs and opt-out without any hardware."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from astra_body_adapter.performance import Performance
from person_interfaces.msg import RuntimeMetrics

rclpy.init()
node=Node('performance_probe');messages=[]
p=Performance(node,'metrics','test')
node.create_subscription(RuntimeMetrics,'metrics',messages.append,10)
try:
    p.inputs=30;p.outputs=25
    for value in range(1,101): p.record('processing',value)
    p.record('control_latency',math.nan);p.record('control_latency',-1.)
    deadline=time.monotonic()+2
    while not messages and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
    assert messages
    first=messages[-1]
    assert first.input_count==30 and first.output_count==25
    assert abs(first.input_fps*first.window_s-30)<1e-8
    assert first.processing_ms==50.5 and first.processing_p95_ms==95
    assert math.isnan(first.control_latency_ms)
    deadline=time.monotonic()+2
    while len(messages)<2 and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
    assert messages[-1].input_fps==0 and math.isnan(messages[-1].processing_ms)
    disabled=Node('disabled_probe',parameter_overrides=[Parameter('performance_enabled',value=False)])
    off=Performance(disabled,'disabled_metrics','test');off.record('processing',1.)
    assert not off.enabled and not hasattr(off,'publisher') and not off.samples['processing']
    disabled.destroy_node()
    print('PASS performance rates/mean/P95, empty windows, invalid sample rejection, disabled publisher')
finally:
    node.destroy_node();rclpy.shutdown()
