"""Single HTTP worker bridges cloud commands to independent ROS capture actions."""
from collections import OrderedDict, deque
import json
from pathlib import Path
import threading
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
from capture_task_interfaces.action import CaptureTask
from .bridge_logic import CommandCache, remaining_ttl, validate_poll_wait, poll_request
from .state import fresh, mode_active, EXECUTOR_MAX_AGE
from .catalog import Catalog
from .cloud_client import CloudClient, CloudError, Cancelled, token_from_env
from .protocol import command, identifier, loads


class CaptureBridge(Node):
    def __init__(self,**kwargs):
        super().__init__('capture_cloud_bridge',**kwargs)
        defaults=dict(poll_wait_sec=1,cloud_url='http://127.0.0.1:8090',robot_id='roscar-01',token_file='',
                      catalog=str(Path(get_package_share_directory('capture_task'))/'config/tasks.yaml'))
        for key,value in defaults.items():
            self.declare_parameter(key,value,ParameterDescriptor(read_only=True))
        self.cfg={key:self.get_parameter(key).value for key in defaults};identifier(self.cfg['robot_id'])
        validate_poll_wait(self.cfg['poll_wait_sec'])
        self.catalog=Catalog.load(self.cfg['catalog'])
        self.cloud=CloudClient(self.cfg['cloud_url'],token_from_env(self.cfg['token_file']),timeout=2.)
        self.stop_event=threading.Event();self.lock=threading.RLock()
        self.cache=CommandCache();self.outbox=deque();self.sequences=OrderedDict()
        self.current=None;self.rejections={};self.executor_state={};self.executor_at=0.;self.mode={};self.mode_at=0.
        self.state='backoff';self.last_poll=0.;self.current_command='';self.backoff=1.;self.retry_at=0.;self.heartbeat_at=0.
        self.group=ReentrantCallbackGroup()
        self.client=ActionClient(self,CaptureTask,'/capture_task/run',callback_group=self.group)
        self.create_subscription(String,'/capture_task/result',self.on_rejection,10,callback_group=self.group)
        self.create_subscription(String,'/capture_task/state',self.on_executor_state,10,callback_group=self.group)
        self.create_subscription(String,'/execution_mode/state',self.on_mode,10,callback_group=self.group)
        self.pub=self.create_publisher(String,'/capture_task/bridge_state',10)
        self.timer=self.create_timer(.25,self.publish_state,callback_group=self.group,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.worker=threading.Thread(target=self.loop,name='capture_cloud_http',daemon=True);self.worker.start()

    def on_rejection(self,message):
        try:
            data=loads(message.data)
            if data.get('status')=='rejected':
                with self.lock:
                    self.rejections[data['task_id']]=(time.monotonic(),data['detail'])
                    if len(self.rejections)>64:
                        del self.rejections[next(iter(self.rejections))]
        except (ValueError,KeyError):
            pass

    def on_executor_state(self,message):
        try:
            self.executor_state=loads(message.data);self.executor_at=time.monotonic()
        except ValueError:
            self.executor_state={};self.executor_at=0.

    def on_mode(self,message):
        try:
            self.mode=loads(message.data);self.mode_at=time.monotonic()
        except ValueError:
            self.mode={'active':True};self.mode_at=time.monotonic()

    def executor_stale(self):
        return not fresh(self.executor_at,EXECUTOR_MAX_AGE)

    def heartbeat(self):
        stale=self.executor_stale();active=self.current;state=self.executor_state
        return dict(state='unknown' if stale else ('busy' if active or state.get('active') else 'idle'),
                    task_id='' if stale else (active['task_id'] if active else state.get('task_id','')),
                    camera_ok=False if stale else state.get('camera_ok',False),
                    tts_ok=False if stale else state.get('tts_ok',False),version='0.1.0')

    def publish_state(self):
        self.pub.publish(String(data=json.dumps(dict(state=self.state,last_poll_monotonic=self.last_poll,
                            command_id=self.current_command,outbox_size=len(self.outbox),executor_stale=self.executor_stale()))))

    def queue(self,path,data):
        with self.lock:
            if len(self.outbox)>=256:
                # Stop producing rather than grow unbounded during an outage.
                if self.current and self.current.get('handle'):
                    self.current['handle'].cancel_goal_async()
                self.outbox.popleft()
            self.outbox.append((path,data))

    def event(self,task_id,phase,detail='',result=None):
        with self.lock:
            seq=self.sequences.get(task_id,0)+1;self.sequences[task_id]=seq
            if len(self.sequences)>2048:
                self.sequences.popitem(last=False)
        data=dict(phase=phase,detail=detail,seq=seq)
        if result is not None:
            data['result']=result
        self.queue('/v1/robots/'+self.cfg['robot_id']+'/tasks/'+task_id+'/events',data)

    def ack(self,cmd,accepted,reason):
        data=dict(accepted=accepted,reason=reason)
        self.cache.put(cmd['command_id'],data)
        self.queue('/v1/robots/'+self.cfg['robot_id']+'/commands/'+cmd['command_id']+'/ack',data)

    def wait_future(self,future,timeout):
        end=time.monotonic()+timeout
        while not future.done():
            if self.stop_event.wait(.01):
                raise Cancelled()
            if time.monotonic()>=end:
                raise ValueError('goal_accept_timeout')
        return future.result()

    def dispatch(self,raw,received=None):
        received=time.monotonic() if received is None else received
        try:
            identifier(raw.get('command_id'))
        except (ValueError,AttributeError):
            return  # Cannot address an ack safely without a valid command_id.
        cached=self.cache.get(raw['command_id'])
        if cached is not None:
            self.queue('/v1/robots/'+self.cfg['robot_id']+'/commands/'+raw['command_id']+'/ack',cached);return
        try:
            cmd=command(raw,self.catalog)
        except ValueError:
            self.ack(raw,False,'invalid');return
        self.current_command=cmd['command_id']
        if cmd['type']=='speak':
            self.ack(cmd,False,'unsupported');return
        if cmd['type']=='cancel':
            with self.lock:
                active=self.current
                matched=active is not None and active['task_id']==cmd['task_id']
                if matched:
                    active['cancelled']=True
                    if active.get('handle'):
                        active['handle'].cancel_goal_async()
            self.ack(cmd,matched,'cancel_requested' if matched else 'not_active');return
        ttl=remaining_ttl(cmd,received,self.catalog.defaults['task_timeout_sec'])
        if ttl<=0:
            self.ack(cmd,False,'expired');return
        with self.lock:
            if self.executor_stale():
                self.ack(cmd,False,'unavailable');return
            if self.current or self.executor_state.get('active'):
                self.ack(cmd,False,'busy');return
            if mode_active(self.mode,self.mode_at):
                self.ack(cmd,False,'voice_mode_active');return
            if not self.client.server_is_ready():
                self.ack(cmd,False,'unavailable');return
            active=dict(task_id=cmd['task_id'],command_id=cmd['command_id'],handle=None,cancelled=False)
            self.current=active
        goal=CaptureTask.Goal(task_id=cmd['task_id'],task_type=cmd['task_type'],source='cloud',
                              params_json=json.dumps(cmd.get('params',{}),ensure_ascii=False),ttl_sec=float(ttl))
        abandoned=threading.Event()
        def late(done):
            try:
                handle=done.result()
                if handle.accepted and (abandoned.is_set() or self.stop_event.is_set() or active['cancelled']):
                    handle.cancel_goal_async()
            except Exception:
                pass
        sent=self.client.send_goal_async(goal,feedback_callback=lambda message:self.event(cmd['task_id'],message.feedback.phase,message.feedback.detail))
        sent.add_done_callback(late)
        try:
            handle=self.wait_future(sent,min(2.,ttl))
            if not handle.accepted:
                # Allow the executor's separate rejection topic to arrive.
                self.stop_event.wait(.05)
                with self.lock:
                    rejection=self.rejections.get(cmd['task_id'])
                    reason=rejection[1] if rejection and rejection[0]>=received else 'rejected'
                    self.current=None
                self.ack(cmd,False,reason);return
            active['handle']=handle
            self.ack(cmd,True,'accepted')
            def finished(done):
                try:
                    result=done.result().result
                    record=dict(status=result.status,detail=result.detail,capture_id=result.capture_id,
                                upload_json=loads('{"receipts":'+(result.upload_json or '[]')+'}')['receipts'])
                    self.event(cmd['task_id'],result.status.upper(),result.detail,record)
                    if result.detail in ('upload_http_401','upload_http_403'):
                        self.auth_failed()
                except Exception:
                    self.event(cmd['task_id'],'FAILED','result_unavailable')
                finally:
                    with self.lock:
                        if self.current is active:
                            self.current=None
            handle.get_result_async().add_done_callback(finished)
        except Exception:
            abandoned.set()
            if sent.done():
                late(sent)
            with self.lock:
                if self.current is active:
                    self.current=None
            self.ack(cmd,False,'expired' if remaining_ttl(cmd,received,self.catalog.defaults['task_timeout_sec'])<=0 else 'unavailable')

    def auth_failed(self):
        self.state='auth_failed';self.retry_at=time.monotonic()+60.

    def request(self,method,path,body=None,timeout=2.):
        try:
            code,data=self.cloud.request(method,path,body,cancel=self.stop_event,deadline=time.monotonic()+timeout,timeout=timeout)
        except CloudError as exc:
            if exc.status in (401,403):
                self.auth_failed()
            raise
        if code in (401,403):
            self.auth_failed();raise CloudError(code,'auth_failed')
        if not 200<=code<300:
            raise CloudError(code,'bridge_http_'+str(code))
        return code,data

    def flush(self,limit=4):
        for _ in range(limit):
            with self.lock:
                pending=self.outbox[0] if self.outbox else None
            if pending is None:
                return
            try:
                self.request('POST',pending[0],pending[1],timeout=1.)
            except CloudError as exc:
                if 400<=exc.status<500 and exc.status not in (401,403,429):
                    with self.lock:
                        if self.outbox and self.outbox[0] is pending:
                            self.outbox.popleft()
                    continue
                raise
            with self.lock:
                if self.outbox and self.outbox[0] is pending:
                    self.outbox.popleft()

    def loop(self):
        while not self.stop_event.is_set():
            if time.monotonic()<self.retry_at:
                self.stop_event.wait(min(.1,self.retry_at-time.monotonic()));continue
            try:
                self.flush()
                now=time.monotonic()
                if now-self.heartbeat_at>=10.:
                    self.request('POST','/v1/robots/'+self.cfg['robot_id']+'/heartbeat',self.heartbeat())
                    self.heartbeat_at=now
                # One HTTP worker: pending events must not incur another long wait.
                path,timeout=poll_request(self.cfg['robot_id'],self.cfg['poll_wait_sec'],bool(self.outbox))
                code,data=self.request('GET',path,timeout=timeout)
                self.last_poll=time.monotonic()
                if self.state!='auth_failed' or time.monotonic()>=self.retry_at:
                    self.state='online';self.backoff=1.
                if code==200:
                    self.dispatch(data, self.last_poll)
            except Cancelled:
                break
            except Exception:
                if self.state!='auth_failed':
                    self.state='backoff';self.retry_at=time.monotonic()+self.backoff;self.backoff=min(30.,self.backoff*2)
        # Best effort final flush is handled by stop while ROS is still spinning.

    def stop(self):
        with self.lock:
            if self.current:
                self.current['cancelled']=True
                if self.current.get('handle'):
                    self.current['handle'].cancel_goal_async()
        # Give the action result callback an opportunity to enqueue its final event.
        until=time.monotonic()+1.
        while self.current and time.monotonic()<until:
            time.sleep(.01)
        self.stop_event.set();self.cloud.cancel();self.worker.join(timeout=3.)
        if self.outbox:
            temporary=threading.Event()
            try:
                for path,data in list(self.outbox)[:8]:
                    self.cloud.request('POST',path,data,cancel=temporary,deadline=time.monotonic()+.1)
            except Exception:
                pass

    def destroy_node(self):
        self.stop();self.client.destroy()
        return super().destroy_node()


def main(args=None):
    from .runtime import run
    run(CaptureBridge,args,threads=4)
