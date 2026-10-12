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
    assert client.request('POST','/admin/commands',data)[0] == 200
    assert client.request('GET','/v1/robots/r/commands/next?wait=0')[1] == data
    server.drop_poll=True
    assert client.request('GET','/v1/robots/r/commands/next?wait=0')[0] == 503
    server.token='hidden'
    assert client.request('GET','/admin/status')[0] == 401


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
