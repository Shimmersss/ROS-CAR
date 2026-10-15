import hashlib
import time
from capture_task.result_outbox import ImageOutbox


def wait(condition):
    until=time.monotonic()+3
    while time.monotonic()<until:
        if condition():return
        time.sleep(.01)
    assert condition()


def test_result_image_hash_and_unknown_task_rejected(cloud):
    server,url=cloud;errors=[]
    server.reject_unknown_task=True
    outbox=ImageOutbox(url,'',lambda *args:errors.append(args))
    try:
        raw=b'\xff\xd8test-image'
        outbox.put('unknown','c',raw)
        wait(lambda:bool(errors))
        assert errors==[('unknown','result_image_return_failed')]
        assert not server.result_images
        server.issued_task_ids.add('known')
        outbox.put('known','c',raw)
        wait(lambda:bool(server.result_images))
        receipt=server.result_images[('known','c','annotated')]
        assert receipt['sha256']==hashlib.sha256(raw).hexdigest()
        assert server.upload_attempts==0
        outbox.put('known','c',raw)  # Duplicate 409 with matching hash is successful.
        wait(lambda:outbox.bytes==0)
        assert len(errors)==1
    finally:outbox.close()


def test_image_outbox_byte_limit_and_receipt_hash():
    errors=[]
    outbox=ImageOutbox('http://127.0.0.1:1','',lambda *args:errors.append(args))
    try:
        outbox.MAX_BYTES=1
        assert not outbox.put('task','capture',b'large')
        assert errors==[('task','image_outbox_full')]
    finally:outbox.close()
