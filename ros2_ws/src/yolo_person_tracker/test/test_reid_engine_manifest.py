"""Engine provenance failures are rejected before any GPU runtime is loaded."""
import hashlib,json
import pytest
from yolo_person_tracker.reid_backend import OSNetBackend

@pytest.mark.parametrize('failure',['engine_hash','source_hash','source_name','source_architecture'])
def test_engine_and_source_provenance_are_required(tmp_path,failure):
    source=tmp_path/'source.onnx';source.write_bytes(b'fixture ONNX')
    engine=tmp_path/'model.engine';engine.write_bytes(b'fixture engine')
    original=dict(architecture='osnet_x1_0_msmt17',preprocessing='rgb_256x128_imagenet',
                  sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    metadata=dict(original,sha256=hashlib.sha256(engine.read_bytes()).hexdigest(),
                  source_onnx=source.name,source_onnx_sha256=original['sha256'])
    if failure=='engine_hash':metadata['sha256']='bad'
    if failure=='source_hash':metadata['source_onnx_sha256']='bad'
    if failure=='source_name':metadata['source_onnx']='../source.onnx'
    if failure=='source_architecture':original['architecture']='osnet_x0_25_msmt17'
    (tmp_path/'source.onnx.json').write_text(json.dumps(original))
    (tmp_path/'model.engine.json').write_text(json.dumps(metadata))
    with pytest.raises(ValueError,match='OSNet'):
        OSNetBackend(str(engine))
