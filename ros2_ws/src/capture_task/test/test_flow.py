import hashlib
import threading
import time
from types import SimpleNamespace
import numpy as np
import pytest
from capture_task.flow import CaptureFlow
from capture_task.cloud_client import CloudClient,CloudError,Cancelled


class Ports:
    robot=False
    def __init__(self,fail_speech=False):
        self.phases=[];self.said=[];self.fail_speech=fail_speech;self.saved=[];self.count=0
    def stage(self,phase,detail=''): self.phases.append(phase)
    def ready(self): pass
    def check(self): pass
    def speak(self,text):
        if self.fail_speech: raise ValueError('tts_not_completed')
        self.said.append(text)
    def wait(self,seconds): pass
    def window(self): return time.monotonic(),10.
    def capture(self,after,ros_after):
        assert self.said and self.phases[-1]=='CAPTURE'
        self.count+=1
        rgb=np.full((480,640,3),100,np.uint8)
        if self.count==2: rgb[::2]=200
        return SimpleNamespace(rgb=rgb,sequence=self.count,ros_stamp=11.)
    def upload(self,image,record): return dict(status='received',task_id=record['task_id'],capture_id=record['capture_id'],upload_id='u')
    def record(self,record,image): self.saved.append(record)


def test_flow_order_best_quality_policy(catalog):
    ports=Ports();job=dict(task_id='t',task_type='face')
    records=CaptureFlow().run(ports,job,catalog.tasks['face'],catalog.defaults)
    assert ports.phases==['CHECK','ANNOUNCE','PREPARE','CAPTURE','QUALITY','UPLOAD','DONE_ANNOUNCE','UPLOADED']
    assert records[0]['frame_sequence']==2
    failed=Ports(True)
    with pytest.raises(ValueError,match='tts_not_completed'):
        CaptureFlow().run(failed,job,catalog.tasks['face'],catalog.defaults)
    assert failed.count==0
    with pytest.raises(ValueError,match='quality_failed'):
        CaptureFlow().run(Ports(),job,catalog.tasks['pcb'],{**catalog.defaults,'quality_policy':'fail'})


@pytest.mark.parametrize('failure,expected,attempts',[(0,200,1),(409,200,1),(401,401,1),(500,500,3)])
def test_upload_retry_receipt(cloud,failure,expected,attempts):
    server,url=cloud;server.fail_upload=failure;client=CloudClient(url)
    image=b'\xff\xd8synthetic'
    meta={'X-Robot-Id':'r','X-Task-Type':'face','X-Quality':'{"brightness":100,"sharpness":10,"rb_ratio":1,"width":640,"height":480}'}
    if expected==200:
        result=client.upload('t','c',image,meta,threading.Event(),time.monotonic()+10)
        assert result['sha256']==hashlib.sha256(image).hexdigest()
    else:
        with pytest.raises(CloudError) as error:
            client.upload('t','c',image,meta,threading.Event(),time.monotonic()+10)
        assert error.value.status==expected
    assert server.upload_attempts==attempts


def test_upload_cancel(cloud):
    server,url=cloud;server.slow_upload=2.;client=CloudClient(url);cancel=threading.Event();errors=[]
    def upload():
        try:
            client.upload('t','c',b'\xff\xd8synthetic',{'X-Robot-Id':'r','X-Task-Type':'face','X-Quality':'{"brightness":100,"sharpness":10,"rb_ratio":1,"width":1,"height":1}'},cancel,time.monotonic()+10)
        except Exception as error: errors.append(type(error))
    worker=threading.Thread(target=upload);worker.start();time.sleep(.1);cancel.set();client.cancel();worker.join(1)
    assert not worker.is_alive() and errors==[Cancelled]
