"""Engine provenance and explicit failure policy; no CUDA/model accuracy claim."""
import hashlib
import json
from unittest.mock import patch

import pytest

from yolo_person_tracker.sot import VitTrack


def fixture_models(tmp_path):
    source = tmp_path/'tracker.onnx'
    source.write_bytes(b'controlled source')
    original = dict(architecture='vittrack_2023sep', sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    source.with_suffix('.onnx.json').write_text(json.dumps(original))
    engine = tmp_path/'tracker.engine'
    engine.write_bytes(b'controlled engine')
    metadata = dict(original, sha256=hashlib.sha256(engine.read_bytes()).hexdigest(),
                    source_onnx=source.name, source_onnx_sha256=original['sha256'], precision='FP32-no-TF32')
    engine.with_suffix('.engine.json').write_text(json.dumps(metadata))
    return source, engine, metadata


def test_engine_uses_only_explicit_gpu_runtime(tmp_path):
    _, engine, _ = fixture_models(tmp_path)
    with patch('yolo_person_tracker.vittrack_tensorrt.VitTrackTensorRT') as gpu:
        tracker = VitTrack(str(engine))
        assert tracker.backend == 'tensorrt' and tracker.run is gpu.return_value
        gpu.assert_called_once_with(engine)


def test_engine_runtime_failure_never_falls_back(tmp_path):
    _, engine, _ = fixture_models(tmp_path)
    with patch('yolo_person_tracker.vittrack_tensorrt.VitTrackTensorRT', side_effect=ValueError('incompatible device')), \
            patch('yolo_person_tracker.sot._OnnxRuntime') as cpu:
        with pytest.raises(ValueError, match='incompatible device'):
            VitTrack(str(engine))
        cpu.assert_not_called()


@pytest.mark.parametrize('change', ['source_hash', 'engine_hash', 'source_name', 'architecture', 'precision'])
def test_invalid_engine_provenance_is_rejected_before_gpu(tmp_path, change):
    source, engine, metadata = fixture_models(tmp_path)
    if change == 'source_hash':
        source.write_bytes(b'changed source')
    elif change == 'engine_hash':
        engine.write_bytes(b'changed engine')
    else:
        metadata[{'source_name': 'source_onnx', 'architecture': 'architecture', 'precision': 'precision'}[change]] = {
            'source_name': '../tracker.onnx', 'architecture': 'other_model', 'precision': 'FP16'}[change]
        engine.with_suffix('.engine.json').write_text(json.dumps(metadata))
    with patch('yolo_person_tracker.vittrack_tensorrt.VitTrackTensorRT') as gpu:
        with pytest.raises(ValueError):
            VitTrack(str(engine))
        gpu.assert_not_called()


def test_backend_and_file_format_must_agree(tmp_path):
    source, engine, _ = fixture_models(tmp_path)
    with pytest.raises(ValueError, match='requires the TensorRT'):
        VitTrack(str(engine), backend='onnxruntime')
    with pytest.raises(ValueError, match='explicitly built local engine'):
        VitTrack(str(source), backend='tensorrt')
