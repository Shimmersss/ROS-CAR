#!/usr/bin/env python3
"""Explicit bench-only cross-machine inference trial using replayed images and fake TTS."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url',required=True);p.add_argument('--token-file',type=Path,required=True)
    p.add_argument('--image',type=Path,required=True)
    p.add_argument('--out',type=Path,default=Path('artifacts/inference-loop/wsl-ros'))
    p.add_argument('--fake-tts',action='store_true',required=True)
    args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='189' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Require isolated domain 189 + localhost; replay only, no vehicle')
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Image
    from capture_task_interfaces.srv import RunInspection
    from capture_task.executor_node import CaptureExecutor
    from capture_task.api_node import CaptureApi
    from capture_task.bridge_node import CaptureBridge
    from capture_task.fake_tts_node import FakeTTS
    from capture_task.mock_cloud import MockCloud
    import yaml
    def params(**values):return [Parameter(k,value=v) for k,v in values.items()]
    rgb=cv2.cvtColor(cv2.imread(str(args.image)),cv2.COLOR_BGR2RGB)
    args.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();client=Node('wsl_inference_bench_client')
    if client.count_publishers('/cmd_vel'):
        client.destroy_node();rclpy.shutdown();raise RuntimeError('bench_requires_zero_velocity_publishers')
    camera=Node('explicit_archived_rgb_replay');cv=CvBridge()
    publisher=camera.create_publisher(Image,'/camera/color/image_raw',qos_profile_sensor_data)
    def publish():
        message=cv.cv2_to_imgmsg(rgb,encoding='rgb8');message.header.stamp=camera.get_clock().now().to_msg()
        message.header.frame_id='archived_image_replay_not_live_camera';publisher.publish(message)
    camera.create_timer(.1,publish)
    catalog=yaml.safe_load((Path(__file__).resolve().parents[1]/'ros2_ws/src/capture_task/config/tasks.yaml').read_text())
    for task in catalog['tasks'].values():task['prepare_sec']=0.
    catalog_path=args.out/'tasks.test.yaml';catalog_path.write_text(yaml.safe_dump(catalog,allow_unicode=True))
    cloud=MockCloud(('127.0.0.1',0),args.out/'cloud',reject_unknown_task=True)
    cloud_thread=threading.Thread(target=cloud.serve_forever,daemon=True);cloud_thread.start()
    cloud_url='http://127.0.0.1:'+str(cloud.server_port)
    capture=CaptureExecutor(parameter_overrides=params(catalog=str(catalog_path),pipeline_mode='infer',
        cloud_url=cloud_url,inference_url=args.url,inference_token_file=str(args.token_file.resolve()),storage_dir=str(args.out/'records')))
    api=CaptureApi(parameter_overrides=params(catalog=str(catalog_path),pipeline_mode='infer'))
    bridge=CaptureBridge(parameter_overrides=params(catalog=str(catalog_path),pipeline_mode='infer',cloud_url=cloud_url,poll_wait_sec=0))
    tts=FakeTTS(parameter_overrides=params(test_only=True,char_seconds=.0001))
    nodes=[capture,api,bridge,tts,camera,client];executor=MultiThreadedExecutor(num_threads=14)
    for node in nodes:executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    report=dict(scope='Humble x86_64 + archived RGB replay + fake speech + real WSL models; no live camera, speaker or vehicle validation.',
        input_sha256=hashlib.sha256(args.image.read_bytes()).hexdigest(),ros={},cloud={})
    def wait(condition,seconds=180):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if condition():return
            time.sleep(.02)
        raise TimeoutError('trial_wait_timeout')
    try:
        assert api.client.wait_for_server(timeout_sec=5) and capture.speech.wait_for_server(timeout_sec=5)
        time.sleep(.5)
        for kind in ('face','tongue','helmet','pcb'):
            service=client.create_client(RunInspection,'/inspect/'+kind)
            assert service.wait_for_service(timeout_sec=5)
            began=time.monotonic();future=service.call_async(RunInspection.Request(task_id='wsl-ros-'+kind,
                announce='',prepare_sec=0.,timeout_sec=180.))
            wait(future.done);response=future.result()
            value=json.loads(response.result_json) if response.result_json else None
            entry=dict(status=response.status,detail=response.detail,seconds=time.monotonic()-began,
                analysis=value,image_bytes=len(response.annotated_image.data))
            if kind=='face':assert response.status=='failed' and entry['detail']=='inference_model_unavailable'
            elif response.status=='retake':
                assert kind=='tongue' and value['status']=='retake' and not response.annotated_image.data
            else:
                assert response.success,(kind,response.status,response.detail)
                raw=bytes(response.annotated_image.data)
                assert hashlib.sha256(raw).hexdigest()==value['annotated_image']['sha256']
                (args.out/(kind+'-annotated.jpg')).write_bytes(raw)
            report['ros'][kind]=entry
            client.destroy_client(service)
        for kind in ('tongue','helmet','pcb'):
            wait(lambda:capture.job is None and bridge.current is None and not bridge.executor_stale() and not bridge.executor_state.get('active'))
            task='wsl-cloud-'+kind;began=time.monotonic()
            with cloud.condition:
                cloud.commands.append(dict(command_id='wsl-command-'+kind,type='capture',task_id=task,task_type=kind,
                    params={'prepare_sec':0.},ttl_sec=180.));cloud.condition.notify_all()
            wait(lambda:any(e.get('result',{}).get('status') in ('analyzed','retake','failed') for e in cloud.events.get(task,[])))
            result=next(e['result'] for e in reversed(cloud.events[task]) if 'result' in e)
            if result['status']=='retake':
                assert kind=='tongue' and result['analysis']['status']=='retake'
                report['cloud'][kind]=dict(status='retake',seconds=time.monotonic()-began,analysis=result['analysis'])
                continue
            wait(lambda:any(key[0]==task for key in cloud.result_images))
            receipt=next(v for key,v in cloud.result_images.items() if key[0]==task)
            assert result['status']=='analyzed'
            assert hashlib.sha256(Path(receipt['path']).read_bytes()).hexdigest()==receipt['sha256']
            report['cloud'][kind]=dict(status=result['status'],seconds=time.monotonic()-began,
                analysis=result['analysis'],image_sha256=receipt['sha256'])
        assert cloud.upload_attempts==0 and not cloud.images
        assert client.count_publishers('/cmd_vel')==0
        report['fake_speech_texts']=list(tts.received)
        report['status']='PASS'
    finally:
        bridge.stop();api.stop();capture.stop();time.sleep(.1)
        executor.shutdown(timeout_sec=5);thread.join(5);tts.server.destroy()
        for node in reversed(nodes):node.destroy_node()
        cloud.stop();cloud_thread.join(3);rclpy.shutdown()
        (args.out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(status=report['status'],scope=report['scope'],output=str(args.out.resolve())),ensure_ascii=False))


if __name__=='__main__':
    main()
