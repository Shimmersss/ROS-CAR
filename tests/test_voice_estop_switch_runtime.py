"""Real ROS router switch: voice ESTOP -> STOP when disabled, original default retained."""
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from roscar_interfaces.msg import VoiceCommandResult
from voice_command_router.router_node import VoiceCommandRouter

def main():
    rclpy.init();probe=Node('voice_estop_switch_fixture');calls=[];results=[]
    probe.create_subscription(VoiceCommandResult,'/voice/command_result',results.append,10)
    def service(name):
        def run(request,response):calls.append(name);response.success=True;return response
        return run
    probe.create_service(Trigger,'/control/stop',service('STOP'))
    probe.create_service(Trigger,'/control/estop',service('ESTOP'))
    def spin(router,predicate):
        end=time.monotonic()+3
        while time.monotonic()<end:
            rclpy.spin_once(probe,timeout_sec=.01);rclpy.spin_once(router,timeout_sec=.01)
            if predicate():return
        raise AssertionError('voice command did not reach expected service')
    try:
        for enabled in (False,True):
            router=VoiceCommandRouter(parameter_overrides=[Parameter('voice_estop_enabled',value=enabled)])
            try:
                spin(router,lambda:all(router._control_services[k].service_is_ready() for k in ('STOP','ESTOP')))
                before=len(results);router._on_asr(String(data='急停'))
                spin(router,lambda:len(results)>before)
                assert calls[-1]==('ESTOP' if enabled else 'STOP')
                before=len(results);router._on_command(router._new_command(action='STOP'))
                spin(router,lambda:len(results)>before);assert calls[-1]=='STOP'
            finally:router.destroy_node()
        print('PASS voice ESTOP disabled -> ordinary STOP, default ESTOP retained, ordinary STOP preserved')
    finally:probe.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
