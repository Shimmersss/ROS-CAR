import threading
import time
import pytest
from capture_task.protocol import command, loads, receipt
from capture_task.cloud_client import CloudClient, CloudError, Cancelled, token_from_env


def test_commands(catalog):
    good = dict(command_id='c1',type='capture',task_id='t1',task_type='face')
    assert command(good,catalog) == good
    for bad in ({}, {**good,'task_id':'../escape'}, {**good,'command_id':'a'*65},
                {**good,'task_type':'unknown'}, {**good,'params':{'x':1}},
                {**good,'params':{'announce':'急停'}}, {**good,'ttl_sec':True}, {**good,'other':1}):
        with pytest.raises(ValueError):
            command(bad,catalog)
    for text in ('{"a":NaN}', '{"a":1,"a":2}', '[]', 'x'*2097153):
        with pytest.raises(ValueError):
            loads(text)


def test_receipts():
    good = dict(status='received',task_id='t',capture_id='c',upload_id='u',sha256='d')
    assert receipt(good,'t','c','d',True)['sha256'] == 'd'
    for bad in ({**good,'task_id':'bad'}, {**good,'sha256':'bad'}, {**good,'upload_id':'../bad'}):
        with pytest.raises(ValueError):
            receipt(bad,'t','c','d',True)


def test_private_token(tmp_path,monkeypatch):
    monkeypatch.delenv('CAPTURE_CLOUD_TOKEN',raising=False)
    path = tmp_path/'token'; path.write_text('secret'); path.chmod(0o644)
    with pytest.raises(ValueError):
        token_from_env(str(path))
    path.chmod(0o600)
    assert token_from_env(str(path)) == 'secret'


def test_mock_poll_auth_fault(cloud):
    server,url=cloud; client=CloudClient(url)
    assert client.request('GET','/v1/robots/r/commands/next?wait=0')[0] == 204
    data=dict(command_id='c',type='cancel',task_id='t')
    assert client.admin_request('POST','/admin/commands',data)[0] == 200
    assert client.request('GET','/v1/robots/r/commands/next?wait=0')[1] == data
    server.drop_poll=True
    assert client.request('GET','/v1/robots/r/commands/next?wait=0')[0] == 503
    server.token='hidden'
    assert client.admin_request('GET','/admin/status')[0] == 401


def test_poll_cancel_closes_socket(cloud):
    _,url=cloud; client=CloudClient(url,timeout=10); cancel=threading.Event(); outcome=[]
    def run():
        try:
            client.request('GET','/v1/robots/r/commands/next?wait=20',cancel=cancel)
        except Exception as exc:
            outcome.append(type(exc))
    thread=threading.Thread(target=run);thread.start();time.sleep(.1)
    cancel.set();client.cancel();thread.join(1)
    assert not thread.is_alive() and outcome == [Cancelled]


@pytest.mark.parametrize('suffix',['','/','/roscar','/roscar/','/api/v1.0_~-'])
def test_cloud_prefix_roundtrip(cloud,suffix):
    import hashlib
    server,url=cloud;client=CloudClient(url+suffix);server.prefix=client.prefix
    assert client.prefix==suffix.removesuffix('/')
    assert client.request('GET','/v1/robots/r/commands/next?wait=0')[0]==204
    data=dict(command_id='c',type='cancel',task_id='t')
    assert client.admin_request('POST','/admin/commands',data)[0]==200
    assert client.request('GET','/v1/robots/r/commands/next?wait=0')[1]==data
    image=b'\xff\xd8synthetic';meta={'X-Robot-Id':'r','X-Task-Type':'face','X-Quality':'{"brightness":100,"sharpness":10,"rb_ratio":1,"width":1,"height":1}'}
    result=client.upload('t','c',image,meta,threading.Event(),time.monotonic()+5)
    assert result['sha256']==hashlib.sha256(image).hexdigest()
    if client.prefix:
        assert CloudClient(url).request('GET','/v1/robots/r/commands/next?wait=0')[0]==404


@pytest.mark.parametrize('url',['http://host//','http://host/roscar//','http://host/a//b','http://host/..',
    'http://host/a/./b','http://host/%2e','http://host/a%2Fb','http://host/roscar?q=1',
    'http://host/roscar#part','http://user:pass@host/roscar','http://@host/roscar','http://host/roscar?',
    'http://host/roscar#','http://host/a b','http://host/a\\b'])
def test_bad_cloud_prefix_rejected(url):
    with pytest.raises(ValueError):CloudClient(url)


def test_internal_request_path_requires_api():
    client=CloudClient('http://127.0.0.1:1/roscar')
    with pytest.raises(AssertionError):client.request('GET','/admin/status')
    with pytest.raises(AssertionError):client.request('GET','http://other/v1/x')


def test_long_poll_overrides_default_socket_timeout(monkeypatch):
    import http.client
    observed={}
    class Connection:
        def __init__(self,host,port,timeout):observed['timeout']=timeout
        def request(self,method,path,*args):observed['path']=path
        def getresponse(self):return self
        def read(self,limit):return b''
        status=204
        def close(self):pass
    monkeypatch.setattr(http.client,'HTTPConnection',Connection)
    client=CloudClient('http://host/roscar',timeout=2.)
    assert client.request('GET','/v1/robots/r/commands/next?wait=5',deadline=time.monotonic()+7,timeout=7)[0]==204
    assert 6.9<observed['timeout']<=7.
    assert observed['path']=='/roscar/v1/robots/r/commands/next?wait=5'
